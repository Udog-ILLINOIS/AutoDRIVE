// Map view in place of the sim's Front Camera Preview (right-hand panel), with two tabs:
//   Ground Truth  the active track's walls, drawn top-down from its own mesh in the style of the stack's
//                 map (redrawn when the track switches, the scene reloads or an obstacle is placed), and
//                 the car where it really is, read from its transform every frame
// The car is drawn to scale from its own meshes (BuildCarSprite), in both tabs.
// Block bar (MapBlocks.cs), a thin strip between the tabs and the figure: drag its square onto the track to put a 0.3 m block on
// the real track there; drag a block's handle to move it, or back into the bar to remove it.
//   Mapping       the occupancy grid the stack maps / localizes on, and where the stack thinks the car
//                 is on it, as the ROS node autodrive_bridge map_view sends them (club stack,
//                 planners/sigrobotics): /map whenever it is published (it grows while mapping) and
//                 TF map -> base_link at 50 Hz
// Both are fitted to the panel, x right and y up as in RViz (ROS x = Unity z, ROS y = -Unity x), and
// refitted when the track or the map changes size. Once the map's placement on the real track is known
// (MapAccuracy.cs), the Mapping tab is drawn in the Ground Truth tab's frame, area and scale, so
// switching tabs leaves the track in place and only real map and pose errors move. Until then (no poses
// from the stack yet) it has the same area and scale, the map centred in it and turned in quarter turns
// to fit best. Tabs: "Ground Truth Tab", "Mapping Tab" (also
// ./autodrive.sh ctl click "Mapping Tab").
//
// map_view connects to 127.0.0.1:4581 (-mapview-port <n>); from the ROS container that is
// host.docker.internal:4581. Frames, little-endian: type (1 byte), payload length (uint32), payload
//   'M'  uint32 width, height, float32 resolution, origin x, y, yaw, width * height int8 cells
//   'P'  float32 x, y, yaw (base_link in the map frame), age (s, how old the estimate was when sent)
//   'H'  heartbeat (empty)
// A newer connection replaces the old one. Log lines start with [MapView].
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

namespace TrackSelect
{
    public partial class MapView : MonoBehaviour
    {
        const string Tag = "[MapView] ";
        static MapView instance;
        static int port = 4581;
        static bool mappingTab;   // the selected tab, kept across scene reloads

        static readonly Color32 Unknown = new Color32(24, 26, 30, 255);
        static readonly Color32 Free = new Color32(72, 78, 88, 255);
        static readonly Color32 Wall = new Color32(236, 236, 236, 255);
        static readonly Color GtCar = new Color(0.25f, 0.9f, 0.45f);
        static readonly Color MapCar = new Color(1f, 0.6f, 0.15f);
        static readonly Color TabOn = new Color(1f, 1f, 1f, 0.28f), TabOff = new Color(1f, 1f, 1f, 0.06f);

        // ---- network (written by the reader thread, read on the main thread under `sync`)
        readonly object sync = new object();
        TcpListener listener;
        TcpClient client;
        volatile bool stopping;
        byte[] mapCells; int mapW, mapH; float mapRes, mapOx, mapOy, mapYaw; int mapSeq;
        float poseX, poseY, poseYaw; long poseTicks, mapTicks;

        // ---- UI (rebuilt per scene)
        GameObject panel, cameraImage, label;
        RectTransform view, image, marker;
        RawImage imageRaw;
        Image markerImage, gtTab, mapTab;
        Text status, accuracyText;
        RectTransform accuracyRect;
        // the text rows under the figure, as a fraction of the view's height: the HUD canvas is in screen
        // pixels (its panels stretch with the window, text does not), so every text size follows its box
        const float TextRowShare = 0.135f;
        float textRow = 16f;
        readonly List<Text> tabTexts = new List<Text>();
        float searchUntil;

        // ---- what is drawn
        Texture2D gtTex, mapTex;
        Rect gtRect;            // ROS world x, y covered by gtTex (m)
        string gtTrack;         // track gtTex was drawn for
        bool gtDirty = true;
        int mapSeqShown = -1; int mapTexW, mapTexH; float mapTexRes, mapTexOx, mapTexOy, mapTexYaw;
        Texture2D alignedTex;   // the map resampled into gtRect (world frame), through the map -> world fit
        int alignedSeq = -1;    // the map it was drawn from
        bool alignedOnTrack;    // drawn through the fit (else centred, as the map lies unaligned)
        float viewCos = 1f, viewSin, viewX, viewY;   // map -> gtRect frame it was drawn with
        Transform car;
        Sprite carSprite;      // the car seen from above, nose to +x
        Vector2 carSize;       // its length and width (m)
        Vector2 carPivot;      // where the car's origin is on it (0-1)
        float carRearAxle = 0.16f;   // m from the rear axle (base_link) forward to the car's origin

        public static void Init()
        {
            string[] args = Environment.GetCommandLineArgs();
            for (int i = 0; i < args.Length - 1; i++)
                if (args[i].ToLower() == "-mapview-port") int.TryParse(args[i + 1], out port);
            var go = new GameObject("MapView");
            DontDestroyOnLoad(go);
            instance = go.AddComponent<MapView>();
            SceneManager.sceneLoaded += (scene, mode) => instance.OnSceneLoaded();
        }

        /// <summary>Redraw the Ground Truth tab (the scene changed, e.g. an obstacle was placed).</summary>
        public static void Invalidate() { if (instance != null) instance.gtDirty = instance.blocksDirty = true; }

        void OnSceneLoaded()
        {
            panel = null; car = null; carSprite = null;
            gtDirty = true;
            searchUntil = Time.realtimeSinceStartup + 20f;
        }

        void Start()
        {
            try
            {
                listener = new TcpListener(IPAddress.Loopback, port);
                listener.Start();
                new Thread(Listen) { IsBackground = true, Name = "MapView" }.Start();
                Debug.Log(Tag + $"listening on 127.0.0.1:{port}");
            }
            catch (Exception e) { Debug.LogError(Tag + $"port {port}: {e.Message}"); }
        }

        void OnDestroy()
        {
            stopping = true;
            try { listener?.Stop(); } catch (Exception) { }
            lock (sync) client?.Close();
        }

        // ------------------------------------------------------------ network

        void Listen()
        {
            while (!stopping)
            {
                TcpClient c;
                try { c = listener.AcceptTcpClient(); }
                catch (Exception) { if (stopping) return; Thread.Sleep(100); continue; }
                lock (sync) { client?.Close(); client = c; }
                Debug.Log(Tag + "ROS map_view connected");
                new Thread(() => Read(c)) { IsBackground = true, Name = "MapView reader" }.Start();
            }
        }

        void Read(TcpClient c)
        {
            try
            {
                c.NoDelay = true;
                c.ReceiveTimeout = 3000;   // map_view sends a heartbeat every second
                var r = new BinaryReader(c.GetStream());
                while (!stopping)
                {
                    byte kind = r.ReadByte();
                    int n = (int)r.ReadUInt32();
                    if (n < 0 || n > 64 << 20) throw new IOException("bad frame length " + n);
                    byte[] p = r.ReadBytes(n);
                    if (p.Length < n) throw new EndOfStreamException();
                    long now = System.Diagnostics.Stopwatch.GetTimestamp();
                    lock (sync)
                    {
                        if (client != c) return;   // replaced by a newer connection
                        if (kind == 'M' && n >= 24)
                        {
                            mapW = (int)BitConverter.ToUInt32(p, 0); mapH = (int)BitConverter.ToUInt32(p, 4);
                            mapRes = BitConverter.ToSingle(p, 8); mapOx = BitConverter.ToSingle(p, 12);
                            mapOy = BitConverter.ToSingle(p, 16); mapYaw = BitConverter.ToSingle(p, 20);
                            if ((long)mapW * mapH != n - 24) throw new IOException($"map {mapW}x{mapH} in {n - 24} bytes");
                            mapCells = new byte[n - 24];
                            Buffer.BlockCopy(p, 24, mapCells, 0, n - 24);
                            mapSeq++; mapTicks = now;
                        }
                        else if (kind == 'P' && n >= 12)
                        {
                            poseX = BitConverter.ToSingle(p, 0); poseY = BitConverter.ToSingle(p, 4);
                            poseYaw = BitConverter.ToSingle(p, 8); poseTicks = now;
                            float age = n >= 16 ? BitConverter.ToSingle(p, 12) : 0f;
                            estimates.Enqueue(new Sample { t = now - (long)(age * System.Diagnostics.Stopwatch.Frequency),
                                                           x = poseX, y = poseY, yaw = poseYaw });
                        }
                    }
                }
            }
            catch (Exception e) { if (!stopping) Debug.Log(Tag + "ROS map_view disconnected: " + e.Message); }
            finally
            {
                c.Close();
                lock (sync) if (client == c) client = null;
            }
        }

        static double Age(long ticks) =>
            ticks == 0 ? double.PositiveInfinity
                       : (System.Diagnostics.Stopwatch.GetTimestamp() - ticks) / (double)System.Diagnostics.Stopwatch.Frequency;

        // ------------------------------------------------------------ per frame

        void LateUpdate()
        {
            if (panel == null)
            {
                if (Time.realtimeSinceStartup > searchUntil) return;
                try { Build(); }
                catch (Exception e) { Debug.LogError(Tag + "UI: " + e); searchUntil = 0f; }
                if (panel == null) return;
            }
            try
            {
                if (gtTrack != null && gtTrack != TrackSelector.Current) ClearMapping();
                if (gtDirty || gtTrack != TrackSelector.Current) DrawGroundTruth();
                UpdateMapTexture();
                UpdateAccuracy();
                if (mappingTab) ShowMapping(); else ShowGroundTruth();
            }
            catch (Exception e) { Debug.LogError(Tag + e); gtDirty = false; gtTrack = TrackSelector.Current; }
        }

        void ShowGroundTruth()
        {
            if (gtTex == null) { Show(null, 1f, 1f, false, 0f, 0f, 0f, "no track mesh"); return; }
            if (!FindCar()) { Show(gtTex, gtRect.width / gtRect.height, gtRect.width, false, 0f, 0f, 0f, gtTrack + ": no car"); return; }
            // the car's own origin: the middle of its footprint, which the marker is drawn around
            Vector3 p = car.position, f = car.forward;
            float x = p.z, y = -p.x, yaw = Mathf.Atan2(-f.x, f.z);
            Show(gtTex, gtRect.width / gtRect.height, gtRect.width, true,
                 (x - gtRect.xMin) / gtRect.width, (y - gtRect.yMin) / gtRect.height, yaw,
                 $"{gtTrack}   x {x:0.00}  y {y:0.00}  yaw {yaw * Mathf.Rad2Deg:0}°");
        }

        void ShowMapping()
        {
            bool connected; double poseAge, mapAge;
            float x, y, yaw;
            lock (sync)
            {
                connected = client != null;
                poseAge = Age(poseTicks); mapAge = Age(mapTicks);
                x = poseX; y = poseY; yaw = poseYaw;
            }
            if (!connected)
            {
                if (alignedTex != null && gtTex != null)
                    Show(alignedTex, gtRect.width / gtRect.height, gtRect.width, false, 0f, 0f, 0f, $"waiting for ROS map_view (bridge launch) on :{port}");
                else
                    Show(mapTex, mapTexW / (float)Math.Max(mapTexH, 1), 1f, false, 0f, 0f, 0f, $"waiting for ROS map_view (bridge launch) on :{port}");
                return;
            }
            if (mapTex == null) { Show(null, 1f, 1f, false, 0f, 0f, 0f, "connected; waiting for /map"); return; }
            // base_link is the rear axle: the marker is drawn around the car's middle, ahead of it
            FindCar();
            bool fresh = poseAge < 0.5;
            string text = $"map {mapTexW}×{mapTexH} @ {mapTexRes:0.00} m   ";
            text += poseAge < double.PositiveInfinity ? $"x {x:0.00}  y {y:0.00}  yaw {yaw * Mathf.Rad2Deg:0}°" : "no TF map → base_link";
            if (poseAge < double.PositiveInfinity && !fresh) text += $"  ({poseAge:0} s old)";
            if (mapAge > 3.0) text += $"   map {mapAge:0} s old";
            if (alignedTex != null && gtTex != null)
            {
                if (!alignedOnTrack) text += "   not aligned yet";
                // in the Ground Truth area and scale: the estimate through the transform the map was drawn with
                float wx = viewCos * x - viewSin * y + viewX, wy = viewSin * x + viewCos * y + viewY;
                float wyaw = yaw + Mathf.Atan2(viewSin, viewCos);
                wx += carRearAxle * Mathf.Cos(wyaw); wy += carRearAxle * Mathf.Sin(wyaw);
                Show(alignedTex, gtRect.width / gtRect.height, gtRect.width, poseAge < double.PositiveInfinity && carSprite != null,
                     (wx - gtRect.xMin) / gtRect.width, (wy - gtRect.yMin) / gtRect.height, wyaw, text, fresh ? 1f : 0.35f);
                return;
            }
            float cx = x + carRearAxle * Mathf.Cos(yaw), cy = y + carRearAxle * Mathf.Sin(yaw);
            // map frame -> grid cells (the grid's origin can be rotated)
            float dx = cx - mapTexOx, dy = cy - mapTexOy, c = Mathf.Cos(mapTexYaw), s = Mathf.Sin(mapTexYaw);
            float u = (c * dx + s * dy) / mapTexRes, v = (-s * dx + c * dy) / mapTexRes;
            Show(mapTex, mapTexW / (float)mapTexH, mapTexW * mapTexRes, poseAge < double.PositiveInfinity && carSprite != null,
                 u / mapTexW, v / mapTexH, yaw - mapTexYaw, text, fresh ? 1f : 0.35f);
        }

        // Fit `tex` (width / height = aspect, `across` m wide) into the view, the car at (u, v) of it (0-1).
        void Show(Texture2D tex, float aspect, float across, bool showCar, float u, float v, float yaw,
                  string text, float carAlpha = 1f)
        {
            imageRaw.texture = tex;
            imageRaw.enabled = tex != null;
            // the figure at the top of the view, the accuracy line under it, the status line at the bottom
            Rect r = view.rect;
            ScaleText(r);
            float barH = LayoutBar(r);   // the block bar, a strip across the top, above the figure
            float height = Mathf.Max(r.height - barH - 2f * textRow - 6f, 10f);
            if (!(aspect > 0f) || float.IsInfinity(aspect)) aspect = 1f;
            float width = r.width;
            // turned a quarter (x up) when that shows it larger, e.g. a tall track in the wide panel
            float w = Mathf.Min(width, height * aspect), wTurned = Mathf.Min(height, width * aspect);
            bool turned = wTurned > w * 1.05f;
            if (turned) w = wTurned;
            float shown = turned ? w : w / aspect;   // its height on screen
            image.sizeDelta = new Vector2(w, w / aspect);
            image.localEulerAngles = new Vector3(0f, 0f, turned ? 90f : 0f);
            image.anchoredPosition = new Vector2(0f, -barH - 2f - shown / 2f);
            figureIsWorld = tex != null && (tex == gtTex || (tex == alignedTex && alignedOnTrack));
            figurePxPerM = w / across;
            UpdateHandles();
            accuracyRect.anchoredPosition = new Vector2(0f, -barH - 4f - shown);
            accuracyText.text = AccuracyText();
            marker.gameObject.SetActive(showCar && tex != null);
            if (marker.gameObject.activeSelf)
            {
                marker.anchorMin = marker.anchorMax = new Vector2(u, v);
                marker.anchoredPosition = Vector2.zero;
                // to scale; at least 6 px long on a very large map, so it does not vanish
                float pxPerM = Mathf.Max(w / across, 6f / carSize.x);
                marker.sizeDelta = carSize * pxPerM;
                marker.pivot = carPivot;
                marker.localEulerAngles = new Vector3(0f, 0f, yaw * Mathf.Rad2Deg);
                Color col = mappingTab ? MapCar : GtCar;
                col.a = carAlpha;
                markerImage.color = col;
            }
            status.text = text;
            gtTab.color = mappingTab ? TabOff : TabOn;
            mapTab.color = mappingTab ? TabOn : TabOff;
        }

        // ------------------------------------------------------------ Mapping tab

        // After a track switch the old track's map and pose are gone; a running stack sends its map
        // again within its publish period (and its pose right away).
        void ClearMapping()
        {
            lock (sync) { mapCells = null; mapTicks = 0; poseTicks = 0; }
            ClearAccuracy();
            if (mapTex != null) Destroy(mapTex);
            mapTex = null;
            mapTexW = mapTexH = 0;
            if (alignedTex != null) Destroy(alignedTex);
            alignedTex = null;
        }

        void UpdateMapTexture()
        {
            byte[] cells; int w, h;
            lock (sync)
            {
                if (mapSeq == mapSeqShown || mapCells == null) return;
                mapSeqShown = mapSeq;
                cells = mapCells; w = mapW; h = mapH;
                mapTexRes = mapRes; mapTexOx = mapOx; mapTexOy = mapOy; mapTexYaw = mapYaw;
            }
            if (w == 0 || h == 0) return;
            if (mapTex == null || mapTex.width != w || mapTex.height != h)
            {
                if (mapTex != null) Destroy(mapTex);
                mapTex = NewTexture(w, h);
            }
            // as the stack's saved map (perception save_map, map_server trinary): occupied from 65 %, free up
            // to 25 %, unknown in between. Cartographer keeps cells it has seen only a few times near 50 %
            // (or ~20 % once mostly seen free); shading those showed as pale smears off the walls.
            var px = new Color32[w * h];
            for (int i = 0; i < px.Length; i++)
            {
                sbyte p = (sbyte)cells[i];
                px[i] = p >= OccupiedPct ? Wall : p >= 0 && p <= FreePct ? Free : Unknown;
            }
            mapTex.SetPixels32(px);
            mapTex.Apply(false);
            mapTexW = w; mapTexH = h;
        }

        const int OccupiedPct = 65, FreePct = 25;   // save_map's OCCUPIED_THRESH / FREE_THRESH

        // The map resampled into the Ground Truth area at half its cell size (finer, so the turn does not
        // show as jagged steps), each pixel the map cell under it through the map -> world fit. Cells
        // outside gtRect (the track plus 0.3 m) are not shown.
        void DrawAligned()
        {
            byte[] cells; int mw, mh; float res, ox, oy, myaw; int seq;
            lock (sync) { cells = mapCells; mw = mapW; mh = mapH; res = mapRes; ox = mapOx; oy = mapOy; myaw = mapYaw; seq = mapSeq; }
            if (cells == null || gtTex == null || res <= 0f) return;
            int w = Math.Min(gtTex.width * 2, 2048), h = Math.Min(gtTex.height * 2, 2048);
            float sx = gtRect.width / w, sy = gtRect.height / h;
            if (fitted) { viewCos = fitCos; viewSin = fitSin; viewX = fitX; viewY = fitY; }
            else if (!CentreMap(cells, mw, mh, res, ox, oy, myaw)) return;
            alignedOnTrack = fitted;
            // gtRect frame -> map (inverse view transform), then map -> grid (inverse origin)
            float ic = viewCos, isn = -viewSin, mc = Mathf.Cos(myaw), ms = -Mathf.Sin(myaw);
            if (alignedTex == null || alignedTex.width != w || alignedTex.height != h)
            {
                if (alignedTex != null) Destroy(alignedTex);
                alignedTex = NewTexture(w, h);
            }
            var px = new Color32[w * h];
            for (int j = 0; j < h; j++)
                for (int i = 0; i < w; i++)
                {
                    float x = gtRect.xMin + (i + 0.5f) * sx - viewX, y = gtRect.yMin + (j + 0.5f) * sy - viewY;
                    float mx = ic * x - isn * y - ox, my = isn * x + ic * y - oy;
                    int ci = Mathf.FloorToInt((mc * mx - ms * my) / res), cj = Mathf.FloorToInt((ms * mx + mc * my) / res);
                    if (ci < 0 || cj < 0 || ci >= mw || cj >= mh) { px[j * w + i] = Unknown; continue; }
                    sbyte p = (sbyte)cells[cj * mw + ci];
                    px[j * w + i] = p >= OccupiedPct ? Wall : p >= 0 && p <= FreePct ? Free : Unknown;
                }
            alignedTex.SetPixels32(px);
            alignedTex.Apply(false);
            alignedSeq = seq;
        }

        // The view transform for a map not yet placed on the track: its known cells' box centred on gtRect,
        // turned a quarter if that fits it better. False if the map has no known cells.
        bool CentreMap(byte[] cells, int mw, int mh, float res, float ox, float oy, float myaw)
        {
            int i0 = mw, i1 = -1, j0 = mh, j1 = -1;
            for (int j = 0; j < mh; j++)
                for (int i = 0; i < mw; i++)
                    if ((sbyte)cells[j * mw + i] >= 0) { i0 = Math.Min(i0, i); i1 = Math.Max(i1, i); j0 = Math.Min(j0, j); j1 = Math.Max(j1, j); }
            if (i1 < 0) return false;
            // the box's corners in the map frame
            float c = Mathf.Cos(myaw), s = Mathf.Sin(myaw);
            Vector2 lo = new Vector2(float.MaxValue, float.MaxValue), hi = -lo;
            foreach (var (gi, gj) in new[] { (i0, j0), (i1 + 1, j0), (i0, j1 + 1), (i1 + 1, j1 + 1) })
            {
                var p = new Vector2(ox + c * gi * res - s * gj * res, oy + s * gi * res + c * gj * res);
                lo = Vector2.Min(lo, p); hi = Vector2.Max(hi, p);
            }
            Vector2 size = hi - lo, mid = (lo + hi) / 2f;
            float straight = Mathf.Max(size.x / gtRect.width, size.y / gtRect.height);
            float turned = Mathf.Max(size.y / gtRect.width, size.x / gtRect.height);
            float th = turned < straight ? Mathf.PI / 2f : 0f;
            viewCos = Mathf.Cos(th); viewSin = Mathf.Sin(th);
            viewX = gtRect.center.x - (viewCos * mid.x - viewSin * mid.y);
            viewY = gtRect.center.y - (viewSin * mid.x + viewCos * mid.y);
            return true;
        }

        static Texture2D NewTexture(int w, int h) =>
            new Texture2D(w, h, TextureFormat.RGBA32, false) { filterMode = FilterMode.Bilinear, wrapMode = TextureWrapMode.Clamp };

        // ------------------------------------------------------------ Ground Truth tab

        const float GtCell = 0.05f;   // m per pixel, as the stack's maps (coarser for large tracks)
        const int GtMaxSide = 1024;

        void DrawGroundTruth()
        {
            gtDirty = false;
            gtTrack = TrackSelector.Current;
            Transform infra = SceneManager.GetActiveScene().GetRootGameObjects()
                .Where(g => g.name == "Infrastructure").Select(g => g.transform).FirstOrDefault();
            Transform track = infra == null ? null : infra.Cast<Transform>().FirstOrDefault(t => t.name == gtTrack + " Track");
            MeshFilter mf = track == null ? null : track.GetComponent<MeshFilter>();
            Renderer rend = track == null ? null : track.GetComponent<Renderer>();
            if (mf == null || mf.sharedMesh == null || rend == null) { Debug.LogWarning(Tag + $"no mesh for track '{gtTrack}'"); return; }
            var sw = System.Diagnostics.Stopwatch.StartNew();

            // ROS world extent of the track, with a margin
            Bounds b = rend.bounds;
            const float margin = 0.3f;
            float extent = Mathf.Max(b.size.z, b.size.x) + 2f * margin;
            Mesh mesh = mf.sharedMesh;
            MeshCollider mc = track.GetComponent<MeshCollider>();
            if (!mesh.isReadable && mc != null && mc.sharedMesh != null && mc.sharedMesh.isReadable) mesh = mc.sharedMesh;
            Collider col = track.GetComponent<Collider>();
            if (!mesh.isReadable && col == null) { Debug.LogWarning(Tag + $"track '{gtTrack}': mesh not readable and no collider"); return; }
            Raster r = Rasterize(track, b, mesh, col, Mathf.Max(GtCell, extent / GtMaxSide));
            int w = r.w, h = r.h;
            byte[] grid = r.g;
            bool filled = r.filled;
            gtRect = new Rect(r.x0, r.y0, w * r.cell, h * r.cell);

            // drawn like the stack's map: of the walls, only the surface facing the track (what the lidar
            // sees); the rest of a wall, and everything off the track, unknown. Without a drivable area
            // (walls not closed around the car), every wall cell.
            var px = new Color32[w * h];
            for (int j = 0; j < h; j++)
                for (int i = 0; i < w; i++)
                {
                    int k = j * w + i;
                    bool wall = r.Surface(i, j);
                    px[k] = wall ? Wall : grid[k] == 2 ? Free : Unknown;
                }
            if (gtTex != null) Destroy(gtTex);
            gtTex = NewTexture(w, h);
            gtTex.SetPixels32(px);
            gtTex.Apply(false);
            // the walls to measure the stack's map against: finer when the mesh can be read
            Raster truth = mesh.isReadable ? Rasterize(track, b, mesh, col, Mathf.Max(TruthCell, extent / TruthMaxSide)) : r;
            SetTruth(truth);
            // what a perfect map at this cell size scores: the cells' centres are off the surface by up to half a cell
            double floor = 0; int nf = 0;
            for (int j = 0; j < h; j++)
                for (int i = 0; i < w; i++)
                    if (r.Surface(i, j)) { floor += DistanceToWall(r.x0 + (i + 0.5f) * r.cell, r.y0 + (j + 0.5f) * r.cell); nf++; }
            Debug.Log(Tag + $"ground truth '{gtTrack}': {w}x{h} px at {r.cell:0.000} m from {r.how}, " +
                      $"{r.obstacles} obstacle(s), drivable area {(filled ? "filled" : "not filled (no closed walls around the car)")}; " +
                      $"walls to measure against at {truth.cell:0.000} m (a perfect {r.cell:0.00} m map scores " +
                      $"{(nf == 0 ? 0 : floor / nf) * 100:0.0} cm); {sw.ElapsedMilliseconds} ms");
        }

        // The track top-down in ROS world x, y: 0 unknown, 1 wall, 2 drivable (flood filled from the car).
        class Raster
        {
            public byte[] g; public int w, h, obstacles; public float x0, y0, cell; public bool filled; public string how;

            // a wall cell facing the track (what a lidar sees; every wall cell when nothing was filled)
            public bool Surface(int i, int j) => g[j * w + i] == 1 && (!filled || NextToFree(g, w, h, i, j));
        }

        static Raster Rasterize(Transform track, Bounds b, Mesh mesh, Collider col, float cell)
        {
            const float margin = 0.3f;
            float x0 = b.min.z - margin, x1 = b.max.z + margin, y0 = -b.max.x - margin, y1 = -b.min.x + margin;
            int w = Mathf.CeilToInt((x1 - x0) / cell), h = Mathf.CeilToInt((y1 - y0) / cell);
            var grid = new byte[w * h];   // 0 unknown, 1 wall, 2 free
            string how;
            if (mesh.isReadable)
            {
                // every triangle, filled, and its edges: the walls' vertical faces project to lines
                Vector3[] verts = mesh.vertices;
                var pts = new Vector2[verts.Length];
                for (int i = 0; i < verts.Length; i++)
                {
                    Vector3 p = track.TransformPoint(verts[i]);
                    pts[i] = new Vector2((p.z - x0) / cell, (-p.x - y0) / cell);
                }
                int[] tris = mesh.triangles;
                for (int i = 0; i + 2 < tris.Length; i += 3)
                {
                    Vector2 a = pts[tris[i]], c = pts[tris[i + 1]], d = pts[tris[i + 2]];
                    FillTriangle(grid, w, h, a, c, d);
                    Line(grid, w, h, a, c); Line(grid, w, h, c, d); Line(grid, w, h, d, a);
                }
                how = $"{tris.Length / 3} triangles";
            }
            else
            {
                // mesh data not readable on the CPU: test each cell against the track's own collider, over
                // the middle half of the walls' height (the ground can be on the same layer)
                int mask = 1 << track.gameObject.layer;
                float mid = (b.max.y + b.min.y) / 2f;
                var half = new Vector3(cell / 2f, (b.max.y - b.min.y) / 4f, cell / 2f);
                var hits = new Collider[16];
                for (int j = 0; j < h; j++)
                    for (int i = 0; i < w; i++)
                    {
                        float x = x0 + (i + 0.5f) * cell, y = y0 + (j + 0.5f) * cell;
                        int n = Physics.OverlapBoxNonAlloc(new Vector3(-y, mid, x), half, hits, Quaternion.identity, mask,
                                                           QueryTriggerInteraction.Ignore);
                        for (int k = 0; k < n; k++) if (hits[k] == col) { grid[j * w + i] = 1; break; }
                    }
                Thicken(grid, w, h);   // the collider is hollow: a cell inside a wall touches neither face
                how = "collider checks";
            }
            // obstacles placed through the control port (SimControl), as boxes
            int obstacles = 0;
            foreach (GameObject o in Resources.FindObjectsOfTypeAll<GameObject>()
                         .Where(g => g.name == SimControl.ObstacleName && g.scene.isLoaded && g.activeInHierarchy))
            {
                Bounds ob = o.GetComponent<Renderer>() != null ? o.GetComponent<Renderer>().bounds : new Bounds(o.transform.position, o.transform.localScale);
                int ia = Mathf.FloorToInt((ob.min.z - x0) / cell), ib = Mathf.CeilToInt((ob.max.z - x0) / cell);
                int ja = Mathf.FloorToInt((-ob.max.x - y0) / cell), jb = Mathf.CeilToInt((-ob.min.x - y0) / cell);
                for (int j = Math.Max(ja, 0); j < Math.Min(jb, h); j++)
                    for (int i = Math.Max(ia, 0); i < Math.Min(ib, w); i++) grid[j * w + i] = 1;
                obstacles++;
            }
            // the drivable area: flood fill from the car, unless it leaks out to the border (a gap)
            GameObject carGo = SceneManager.GetActiveScene().GetRootGameObjects().FirstOrDefault(g => g.name == "Roboracer");
            bool filled = false;
            if (carGo != null)
            {
                Vector3 p = carGo.transform.position;
                filled = FloodFree(grid, w, h, (int)((p.z - x0) / cell), (int)((-p.x - y0) / cell));
            }
            return new Raster { g = grid, w = w, h = h, x0 = x0, y0 = y0, cell = cell, filled = filled, how = how, obstacles = obstacles };
        }

        static void Set(byte[] g, int w, int h, int i, int j)
        {
            if (i >= 0 && j >= 0 && i < w && j < h) g[j * w + i] = 1;
        }

        static bool NextToFree(byte[] g, int w, int h, int i, int j)
        {
            for (int dj = -1; dj <= 1; dj++)
                for (int di = -1; di <= 1; di++)
                {
                    int a = i + di, b = j + dj;
                    if (a >= 0 && b >= 0 && a < w && b < h && g[b * w + a] == 2) return true;
                }
            return false;
        }

        static void Line(byte[] g, int w, int h, Vector2 a, Vector2 b)
        {
            int n = Mathf.CeilToInt(Mathf.Max(Mathf.Abs(b.x - a.x), Mathf.Abs(b.y - a.y))) + 1;
            for (int k = 0; k <= n; k++)
            {
                Vector2 p = Vector2.Lerp(a, b, k / (float)n);
                Set(g, w, h, (int)p.x, (int)p.y);
            }
        }

        static void FillTriangle(byte[] g, int w, int h, Vector2 a, Vector2 b, Vector2 c)
        {
            float area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
            if (Mathf.Abs(area) < 1e-3f) return;   // edge on (a wall face): the edges cover it
            int i0 = Math.Max(0, (int)Mathf.Min(a.x, Mathf.Min(b.x, c.x))), i1 = Math.Min(w - 1, (int)Mathf.Max(a.x, Mathf.Max(b.x, c.x)));
            int j0 = Math.Max(0, (int)Mathf.Min(a.y, Mathf.Min(b.y, c.y))), j1 = Math.Min(h - 1, (int)Mathf.Max(a.y, Mathf.Max(b.y, c.y)));
            float sign = Mathf.Sign(area);
            for (int j = j0; j <= j1; j++)
                for (int i = i0; i <= i1; i++)
                {
                    var p = new Vector2(i + 0.5f, j + 0.5f);
                    if (sign * Cross(a, b, p) >= 0 && sign * Cross(b, c, p) >= 0 && sign * Cross(c, a, p) >= 0) g[j * w + i] = 1;
                }
        }

        static float Cross(Vector2 a, Vector2 b, Vector2 p) => (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);

        // walls one pixel thicker on each side
        static void Thicken(byte[] g, int w, int h)
        {
            var src = (byte[])g.Clone();
            for (int j = 0; j < h; j++)
                for (int i = 0; i < w; i++)
                    if (src[j * w + i] == 1)
                        for (int dj = -1; dj <= 1; dj++)
                            for (int di = -1; di <= 1; di++) Set(g, w, h, i + di, j + dj);
        }

        static bool FloodFree(byte[] g, int w, int h, int si, int sj)
        {
            if (si <= 0 || sj <= 0 || si >= w - 1 || sj >= h - 1 || g[sj * w + si] != 0) return false;
            var stack = new Stack<int>();
            var done = new List<int>();
            stack.Push(sj * w + si);
            g[sj * w + si] = 2;
            while (stack.Count > 0)
            {
                int k = stack.Pop();
                done.Add(k);
                int i = k % w, j = k / w;
                if (i == 0 || j == 0 || i == w - 1 || j == h - 1)
                {
                    foreach (int d in done) g[d] = 0;          // leaked: undo
                    foreach (int d in stack) g[d] = 0;
                    return false;
                }
                foreach (int n in new[] { k - 1, k + 1, k - w, k + w })
                    if (g[n] == 0) { g[n] = 2; stack.Push(n); }
            }
            return true;
        }

        // ------------------------------------------------------------ UI

        // The panel "Vehicle Front Camera Preview" holds the camera picture ("Front Camera Preview",
        // a RawImage) and its title ("Label"). The picture is hidden (the camera itself keeps running:
        // it is a sensor) and the map view takes its place; two tabs take the title's.
        void Build()
        {
            GameObject p = Resources.FindObjectsOfTypeAll<GameObject>()
                .FirstOrDefault(g => g.name == "Vehicle Front Camera Preview" && g.scene.isLoaded);
            if (p == null) return;   // LateUpdate tries again
            Transform cam = p.transform.Find("Front Camera Preview"), lab = p.transform.Find("Label");
            if (cam == null || lab == null) { Debug.LogWarning(Tag + "Front Camera Preview layout changed: no map view"); searchUntil = 0f; return; }
            cameraImage = cam.gameObject; label = lab.gameObject;
            // a little larger than the camera preview was (anchors 0.07-0.93 x 0.33-0.54 of the HUD), into the
            // gaps beside it and above and below it (the HUD text ends at 0.56, the lap times start at 0.29)
            var prt = (RectTransform)p.transform;
            prt.anchorMin = new Vector2(0.04f, 0.30f); prt.anchorMax = new Vector2(0.96f, 0.555f);
            RawImage camRaw = cameraImage.GetComponent<RawImage>();
            if (camRaw != null) camRaw.enabled = false;
            Text labelText = label.GetComponent<Text>();

            var camRt = (RectTransform)cam;
            view = NewRect("Map View", p.transform, camRt.anchorMin, camRt.anchorMax);
            view.offsetMin = camRt.offsetMin; view.offsetMax = camRt.offsetMax;
            view.gameObject.AddComponent<RectMask2D>();
            Image bg = view.gameObject.AddComponent<Image>();
            bg.color = new Color(Unknown.r / 255f, Unknown.g / 255f, Unknown.b / 255f, 1f);

            image = NewRect("Map Image", view, new Vector2(0.5f, 1f), new Vector2(0.5f, 1f));
            imageRaw = image.gameObject.AddComponent<RawImage>();
            marker = NewRect("Car Marker", image, new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f));
            markerImage = marker.gameObject.AddComponent<Image>();
            markerImage.raycastTarget = false;
            imageRaw.raycastTarget = false;

            var srt = NewRect("Map Status", view, new Vector2(0f, 0f), new Vector2(1f, 0f));
            srt.pivot = new Vector2(0.5f, 0f);
            srt.sizeDelta = new Vector2(-8f, textRow);
            srt.anchoredPosition = new Vector2(0f, 2f);
            status = srt.gameObject.AddComponent<Text>();
            status.font = labelText != null ? labelText.font : Resources.GetBuiltinResource<Font>("Arial.ttf");
            status.resizeTextForBestFit = true; status.resizeTextMinSize = 4;
            status.alignment = TextAnchor.LowerLeft;
            status.color = new Color(1f, 1f, 1f, 0.85f);
            status.raycastTarget = false;
            Shadow sh = srt.gameObject.AddComponent<Shadow>();
            sh.effectColor = new Color(0f, 0f, 0f, 0.9f);

            accuracyRect = NewRect("Map Accuracy", view, new Vector2(0f, 1f), new Vector2(1f, 1f));
            accuracyRect.pivot = new Vector2(0.5f, 1f);
            accuracyRect.sizeDelta = new Vector2(-8f, textRow);
            accuracyText = accuracyRect.gameObject.AddComponent<Text>();
            accuracyText.font = status.font;
            accuracyText.fontStyle = FontStyle.Bold;
            accuracyText.resizeTextForBestFit = true; accuracyText.resizeTextMinSize = 4;
            accuracyText.alignment = TextAnchor.MiddleCenter;
            accuracyText.color = Color.white;
            accuracyText.raycastTarget = false;

            tabTexts.Clear();
            BuildBar();
            var labRt = (RectTransform)lab;
            gtTab = Tab("Ground Truth Tab", "Ground Truth", p.transform, labRt, labelText, 0f, 0.5f, false);
            mapTab = Tab("Mapping Tab", "Mapping", p.transform, labRt, labelText, 0.5f, 1f, true);
            label.SetActive(false);
            panel = p;
            Debug.Log(Tag + "map view in place of the Front Camera Preview");
        }

        // Text sizes from the boxes' current heights: they grow and shrink with the window. Best fit only
        // shrinks them further when a line is too long for its box.
        void ScaleText(Rect r)
        {
            textRow = Mathf.Max(6f, r.height * TextRowShare);
            ((RectTransform)status.transform).sizeDelta = new Vector2(-8f, textRow);
            accuracyRect.sizeDelta = new Vector2(-8f, textRow);
            SetSize(status, Mathf.RoundToInt(textRow * 0.72f));
            SetSize(accuracyText, Mathf.RoundToInt(textRow * 0.8f));
            foreach (Text t in tabTexts)
                SetSize(t, Mathf.RoundToInt(((RectTransform)t.transform.parent).rect.height * 0.6f));
        }

        static void SetSize(Text t, int size)
        {
            size = Math.Max(size, 4);
            if (t.fontSize == size) return;   // a change rebuilds the text's mesh
            t.fontSize = size; t.resizeTextMaxSize = size;
        }

        static RectTransform NewRect(string name, Transform parent, Vector2 anchorMin, Vector2 anchorMax)
        {
            var go = new GameObject(name, typeof(RectTransform));
            go.layer = parent.gameObject.layer;
            var rt = (RectTransform)go.transform;
            rt.SetParent(parent, false);
            rt.anchorMin = anchorMin; rt.anchorMax = anchorMax;
            rt.offsetMin = rt.offsetMax = Vector2.zero;
            return rt;
        }

        // A tab over [x0, x1] of the title's row, its text styled like the title.
        Image Tab(string name, string text, Transform parent, RectTransform title, Text style, float x0, float x1, bool mapping)
        {
            var rt = NewRect(name, parent, new Vector2(Mathf.Lerp(title.anchorMin.x, title.anchorMax.x, x0), title.anchorMin.y),
                             new Vector2(Mathf.Lerp(title.anchorMin.x, title.anchorMax.x, x1), title.anchorMax.y));
            rt.offsetMin = new Vector2(x0 > 0f ? 1f : 0f, 0f); rt.offsetMax = new Vector2(x1 < 1f ? -1f : 0f, 0f);
            Image img = rt.gameObject.AddComponent<Image>();
            Button b = rt.gameObject.AddComponent<Button>();
            b.targetGraphic = img;
            b.onClick.AddListener(() => mappingTab = mapping);
            var trt = NewRect("Label", rt, Vector2.zero, Vector2.one);
            Text t = trt.gameObject.AddComponent<Text>();
            t.font = style != null ? style.font : Resources.GetBuiltinResource<Font>("Arial.ttf");
            t.fontSize = style != null ? style.fontSize : 14;
            t.fontStyle = style != null ? style.fontStyle : FontStyle.Bold;
            t.color = style != null ? style.color : Color.white;
            t.alignment = TextAnchor.MiddleCenter;
            t.resizeTextForBestFit = true; t.resizeTextMinSize = 4;
            tabTexts.Add(t);
            t.raycastTarget = false;
            t.text = text;
            return img;
        }

        bool FindCar()
        {
            if (car == null)
            {
                GameObject go = SceneManager.GetActiveScene().GetRootGameObjects().FirstOrDefault(g => g.name == "Roboracer");
                car = go == null ? null : go.transform;
                if (car == null) return false;
            }
            if (carSprite == null)
            {
                BuildCarSprite();
                markerImage.sprite = carSprite;
            }
            return true;
        }

        // The car seen from above, to scale: each of its meshes as its box in the car's frame (Unity x
        // right, z forward), the body white (tinted per tab) under a dark outline, the tyres dark. A
        // tyre's box depends on how far it has rolled, so its length is its diameter, twice the
        // height of its centre. The rear tyres give the rear axle (base_link).
        void BuildCarSprite()
        {
            var body = new List<Rect>();
            var tyres = new List<Rect>();
            var rear = new List<float>();
            float x0 = float.MaxValue, x1 = float.MinValue, z0 = float.MaxValue, z1 = float.MinValue;
            foreach (MeshFilter mf in car.GetComponentsInChildren<MeshFilter>())
            {
                Renderer rd = mf.GetComponent<Renderer>();
                if (mf.sharedMesh == null || rd == null || !rd.enabled) continue;
                Bounds b = mf.sharedMesh.bounds;
                Vector3 lo = Vector3.positiveInfinity, hi = Vector3.negativeInfinity;
                for (int i = 0; i < 8; i++)
                {
                    var c = new Vector3((i & 1) == 0 ? b.min.x : b.max.x, (i & 2) == 0 ? b.min.y : b.max.y, (i & 4) == 0 ? b.min.z : b.max.z);
                    Vector3 q = car.InverseTransformPoint(mf.transform.TransformPoint(c));
                    lo = Vector3.Min(lo, q); hi = Vector3.Max(hi, q);
                }
                if (hi.x - lo.x > 1f || hi.z - lo.z > 1f) continue;   // not part of the car's body
                if (mf.name.StartsWith("Wheel-"))
                {
                    float cz = (lo.z + hi.z) / 2f, d = Mathf.Clamp(lo.y + hi.y, 0.05f, 0.25f);
                    lo.z = cz - d / 2f; hi.z = cz + d / 2f;
                    tyres.Add(Rect.MinMaxRect(lo.x, lo.z, hi.x, hi.z));
                    if (mf.name.EndsWith("solid1") && cz < 0f) rear.Add(cz);
                }
                else body.Add(Rect.MinMaxRect(lo.x, lo.z, hi.x, hi.z));
                x0 = Mathf.Min(x0, lo.x); x1 = Mathf.Max(x1, hi.x); z0 = Mathf.Min(z0, lo.z); z1 = Mathf.Max(z1, hi.z);
            }
            if (body.Count + tyres.Count == 0) { x0 = -0.15f; x1 = 0.15f; z0 = -0.27f; z1 = 0.27f; body.Add(Rect.MinMaxRect(x0, z0, x1, z1)); }
            if (rear.Count > 0) carRearAxle = -rear.Average();

            const float ppm = 200f;   // sprite pixels per metre; 1 px of outline all round
            int W = Mathf.CeilToInt((z1 - z0) * ppm) + 2, H = Mathf.CeilToInt((x1 - x0) * ppm) + 2;
            var px = new Color32[W * H];
            // sprite x = forward (Unity z), sprite y = left (Unity -x)
            void Fill(Rect r, Color32 col)
            {
                int i0 = Mathf.FloorToInt((r.yMin - z0) * ppm) + 1, i1 = Mathf.CeilToInt((r.yMax - z0) * ppm) + 1;
                int j0 = Mathf.FloorToInt((x1 - r.xMax) * ppm) + 1, j1 = Mathf.CeilToInt((x1 - r.xMin) * ppm) + 1;
                for (int j = Math.Max(j0, 0); j < Math.Min(j1, H); j++)
                    for (int i = Math.Max(i0, 0); i < Math.Min(i1, W); i++) px[j * W + i] = col;
            }
            foreach (Rect r in body) Fill(r, new Color32(255, 255, 255, 255));
            foreach (Rect r in tyres) Fill(r, new Color32(34, 34, 38, 255));
            var outline = new Color32(16, 16, 18, 255);
            var src = (Color32[])px.Clone();
            for (int j = 0; j < H; j++)
                for (int i = 0; i < W; i++)
                    if (src[j * W + i].a == 0 &&
                        ((i > 0 && src[j * W + i - 1].a > 0) || (i < W - 1 && src[j * W + i + 1].a > 0) ||
                         (j > 0 && src[(j - 1) * W + i].a > 0) || (j < H - 1 && src[(j + 1) * W + i].a > 0)))
                        px[j * W + i] = outline;
            var tex = new Texture2D(W, H, TextureFormat.RGBA32, false) { filterMode = FilterMode.Bilinear, wrapMode = TextureWrapMode.Clamp };
            tex.SetPixels32(px);
            tex.Apply();
            carSize = new Vector2(W / ppm, H / ppm);
            carPivot = new Vector2((-z0 * ppm + 1f) / W, (x1 * ppm + 1f) / H);
            carSprite = Sprite.Create(tex, new Rect(0, 0, W, H), carPivot, ppm);
            Debug.Log(Tag + $"car marker: {carSize.x:0.00} x {carSize.y:0.00} m from {body.Count} parts and {tyres.Count} tyre meshes, " +
                      $"rear axle {carRearAxle:0.000} m behind the car's origin");
        }
    }
}
