// Remote control and a Map button for the AutoDRIVE RoboRacer sim (2026-iros build).
//
// Launch arguments (besides the sim's own -ip / -port and TrackSelector's -track):
//   -autoconnect          keep the sim connected to the bridge (-ip:-port, default 127.0.0.1:4567),
//                         as if Connect were pressed, also across bridge restarts (see Supervise)
//   -control-port <n>     control port, default 4580, on 127.0.0.1 only
//
// Control port: one command per TCP connection, one line in, one JSON line out, e.g.
//   printf 'status\n' | nc 127.0.0.1 4580        (./autodrive.sh ctl status)
//   status | tracks | track <name> | connect [ip [port]] | disconnect | reconnect | kick
//   | click <button> | screenshot [path] | tree [^]<object> [depth] | clickat <x> <y>
// click <button> is the way to press the sim's buttons: the pointer events of a mouse click, sent to
// the visible object with that name (Menu, "Driving Mode", Map, "Map Porto", "Scene Reset", ...). It
// does not move the real mouse. clickat sends them to whatever UI is drawn at x, y (0-1 of the window,
// from its top left): only for checking the layout, since positions change with the window size and scaling.
// screenshot saves only the sim's own picture.
// Car back to its start pose: the bridge's /autodrive/reset_command (Autonomous only; also zeroes the
// command), or track <current track> (any mode, no bridge needed; the bridge does not see it).
// Scene Reset reloads the whole scene.
// Commands run on Unity's main thread, so they wait while the sim is paused.
//
// Map button: a row in the Menu panel (top-left button) right after Camera, cloned from the sim's
// own buttons; it opens a list of the tracks beside the panel, and picking one switches to it with
// the car at its start pose (TrackSelector.Switch), without restarting the sim.
// Log lines start with [SimControl].
using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Reflection;
using System.Text;
using System.Threading;
using SocketIO;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.SceneManagement;
using UnityEngine.UI;
using Debug = UnityEngine.Debug;

namespace TrackSelect
{
    public class SimControl : MonoBehaviour
    {
        const string Tag = "[SimControl] ";
        static SimControl instance;
        static int controlPort = 4580;
        static bool autoConnect;

        class Request
        {
            public string Line, Reply;
            public readonly ManualResetEventSlim Done = new ManualResetEventSlim(false);
        }

        static readonly ConcurrentQueue<Request> queue = new ConcurrentQueue<Request>();
        TcpListener listener;
        volatile bool stopping;
        // render rate and "Bridge" message rate over the last second (status: frames_per_s vs fps;
        // a ratio near 2 or more means more than one frame exchange runs at once)
        float rateWindowStart; int rateFrames0, rateMessages0;
        float fps, messagesPerS;
        SocketIOComponent hooked;           // SocketIO component whose "Bridge" events we count
        long lastFrameTicks;                // Stopwatch ticks of the last "Bridge" message from the bridge
        int totalMessages;                  // "Bridge" messages since start
        int disconnects, disconnectsAtEnable; // the client's "disconnect" events (its WebSocket closed)
        // connection supervision (Supervise); times are in runTime, which stops while the sim is paused
        bool want;                          // keep connected (-autoconnect, connect, or the Connect button)
        bool expectEnabled;                 // what we last set the Connect objects to
        float runTime, lastFrameRun, enabledAtRun, lastKickRun, lastProbeRun;
        int lastTotal;
        volatile int probe;                 // 0 idle, 1 running, 2 bridge answered, 3 no bridge
        GameObject mapButton, mapList;
        float mapSearchUntil;               // realtime until which BuildMapButton keeps looking for the Menu panel

        public static void Init()
        {
            string[] args = Environment.GetCommandLineArgs();
            for (int i = 0; i < args.Length; i++)
            {
                string a = args[i].ToLower();
                if (a == "-autoconnect") autoConnect = true;
                if (a == "-control-port" && i + 1 < args.Length) int.TryParse(args[i + 1], out controlPort);
            }
            var go = new GameObject("SimControl");
            DontDestroyOnLoad(go);
            instance = go.AddComponent<SimControl>();
            SceneManager.sceneLoaded += (scene, mode) => instance.OnSceneLoaded();
        }

        void Start()
        {
            try
            {
                listener = new TcpListener(IPAddress.Loopback, controlPort);
                listener.Start();
                new Thread(Serve) { IsBackground = true, Name = "SimControl" }.Start();
                Debug.Log(Tag + $"control port 127.0.0.1:{controlPort}");
            }
            catch (Exception e) { Debug.LogError(Tag + $"control port {controlPort}: {e.Message}"); }
        }

        void OnDestroy()
        {
            stopping = true;
            try { listener?.Stop(); } catch (Exception) { }
        }

        void OnSceneLoaded()
        {
            hooked = null;
            expectEnabled = false;   // the new scene's Connect objects start disabled
            if (autoConnect) want = true;
            mapButton = mapList = null;
            mapSearchUntil = Time.realtimeSinceStartup + 20f;   // built in Update once the menu is found
        }

        // ------------------------------------------------------------ control port

        void Serve()
        {
            while (!stopping)
            {
                TcpClient client;
                try { client = listener.AcceptTcpClient(); }
                catch (Exception e)
                {
                    // a client gone before accept (e.g. nc -w) throws here too: only stopping ends the loop
                    if (stopping) return;
                    Debug.LogWarning(Tag + "control port accept: " + e.Message);
                    Thread.Sleep(100);
                    continue;
                }
                try
                {
                    using (client)
                    {
                        client.ReceiveTimeout = 5000;
                        var stream = client.GetStream();
                        string line = new StreamReader(stream, Encoding.UTF8).ReadLine();
                        if (string.IsNullOrWhiteSpace(line)) continue;
                        var req = new Request { Line = line.Trim() };
                        queue.Enqueue(req);
                        string reply = req.Done.Wait(10000) ? req.Reply
                            : "{\"ok\":false,\"error\":\"sim did not answer in 10 s (paused?)\"}";
                        byte[] bytes = Encoding.UTF8.GetBytes(reply + "\n");
                        stream.Write(bytes, 0, bytes.Length);
                    }
                }
                catch (Exception) { }   // client went away
            }
        }

        void Update()
        {
            if (hooked == null) Hook();
            runTime += Mathf.Min(Time.unscaledDeltaTime, 0.1f);
            int total = Volatile.Read(ref totalMessages);
            if (total != lastTotal) { lastTotal = total; lastFrameRun = runTime; }
            try { Supervise(); } catch (Exception e) { Debug.LogError(Tag + "connection: " + e); }
            float now = Time.realtimeSinceStartup;
            if (now - rateWindowStart >= 1f)
            {
                int messages = Volatile.Read(ref totalMessages);
                fps = (Time.frameCount - rateFrames0) / (now - rateWindowStart);
                messagesPerS = (messages - rateMessages0) / (now - rateWindowStart);
                rateWindowStart = now; rateFrames0 = Time.frameCount; rateMessages0 = messages;
            }
            if (mapButton == null && Time.realtimeSinceStartup < mapSearchUntil)
            {
                try { BuildMapButton(); }
                catch (Exception e) { Debug.LogError(Tag + "Map button: " + e); mapSearchUntil = 0f; }
                if (mapButton == null && Time.realtimeSinceStartup >= mapSearchUntil - 0.05f) LogNoMenu();
            }
            while (queue.TryDequeue(out Request req))
            {
                try { req.Reply = Handle(req.Line); }
                catch (Exception e) { req.Reply = Json(false, "error", e.Message); }
                req.Done.Set();
            }
        }

        string Handle(string line)
        {
            string[] w = line.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
            string cmd = w[0].ToLower();
            switch (cmd)
            {
                case "status": return Status();
                case "tracks":
                    return "{\"ok\":true,\"current\":" + Str(TrackSelector.Current) + ",\"tracks\":[" +
                           string.Join(",", TrackSelector.Tracks.Select(t => "{\"name\":" + Str(TrackSelector.Key(t)) +
                                                                             ",\"display\":" + Str(t) + "}")) + "]}";
                case "track":
                    if (w.Length < 2) return Json(false, "error", "usage: track <name>");
                    string chosen = SwitchTrack(string.Join(" ", w.Skip(1)));
                    return chosen == null ? Json(false, "error", "unknown track; see 'tracks'") : Json(true, "track", chosen);
                case "connect": return Json(true, "result", Connect(w.Length > 1 ? w[1] : null, w.Length > 2 ? w[2] : null));
                case "disconnect": return Json(true, "result", Disconnect());
                case "reconnect":
                    want = true;
                    Disable("reconnect command");
                    return Json(true, "result", "disconnected; connecting again when the bridge answers");
                case "kick": return Json(true, "result", Kick());
                case "click":   // click <object name>: press that UI button as a mouse click would
                    {
                        if (w.Length < 2) return Json(false, "error", "usage: click <object name>");
                        string name = string.Join(" ", w.Skip(1));
                        // names repeat (the Menu button and the Menu panel it opens): take the visible
                        // object with that name that handles clicks itself
                        GameObject[] named = Resources.FindObjectsOfTypeAll<GameObject>()
                            .Where(g => g.name == name && g.scene.isLoaded && g.activeInHierarchy).ToArray();
                        if (named.Length == 0) return Json(false, "error", "no visible object named " + name);
                        GameObject[] clickable = named.Where(g => ExecuteEvents.GetEventHandler<IPointerClickHandler>(g) == g).ToArray();
                        if (clickable.Length > 1)
                            return Json(false, "error", $"{clickable.Length} clickable objects named {name}; use tree to tell them apart");
                        return Click(clickable.Length == 1 ? clickable[0] : named[0], null);
                    }
                case "clickat":   // clickat <x> <y>: click whatever UI is drawn there (0-1 of the window, from
                                  // top left). Layout checks only: use click <button> to press buttons
                    {
                        if (w.Length < 3 || !float.TryParse(w[1], NumberStyles.Float, CultureInfo.InvariantCulture, out float fx)
                                         || !float.TryParse(w[2], NumberStyles.Float, CultureInfo.InvariantCulture, out float fy))
                            return Json(false, "error", "usage: clickat <x> <y>   (0-1 of the window, from its top left)");
                        var pos = new Vector2(fx * Screen.width, (1f - fy) * Screen.height);
                        EventSystem es = EventSystem.current;
                        if (es == null) return Json(false, "error", "no EventSystem");
                        var data = new PointerEventData(es) { position = pos };
                        var hits = new System.Collections.Generic.List<RaycastResult>();
                        es.RaycastAll(data, hits);
                        if (hits.Count == 0) return Json(false, "error", $"no UI at ({pos.x:0}, {pos.y:0}) px");
                        return Click(hits[0].gameObject, data);
                    }
                case "screenshot":   // screenshot [path]: the sim's own picture (no other windows), PNG
                    {
                        string path = w.Length > 1 ? string.Join(" ", w.Skip(1))
                                                   : Path.Combine(Application.persistentDataPath, "simcontrol.png");
                        ScreenCapture.CaptureScreenshot(path);   // written at the end of this frame
                        return "{\"ok\":true,\"path\":" + Str(path) + ",\"width\":" + Screen.width + ",\"height\":" + Screen.height + "}";
                    }
                case "tree":   // tree [^]<object name> [depth]: the scene hierarchy under it (^: its parent)
                    {
                        if (w.Length < 2) return Json(false, "error", "usage: tree [^]<object name> [depth]");
                        int depth = w.Length > 2 && int.TryParse(w[w.Length - 1], out int d) ? d : 3;
                        string name = string.Join(" ", w.Skip(1).Take(w.Length > 2 && int.TryParse(w[w.Length - 1], out _) ? w.Length - 2 : w.Length - 1));
                        bool parent = name.StartsWith("^");
                        if (parent) name = name.Substring(1);
                        Transform root = Resources.FindObjectsOfTypeAll<Transform>()
                            .FirstOrDefault(t => t.name == name && t.gameObject.scene.isLoaded);
                        if (root != null && parent) root = root.parent;
                        if (root == null) return Json(false, "error", "no object named " + name);
                        var sb = new StringBuilder();
                        Tree(root, 0, depth, sb);
                        return Json(true, "tree", sb.ToString());
                    }
                default:
                    return Json(false, "error", "commands: status, tracks, track <name>, connect [ip [port]], " +
                                                "disconnect, reconnect, kick, click <button>, screenshot [path], " +
                                                "tree [^]<object> [depth], clickat <x> <y> (layout checks only)");
            }
        }

        string Status()
        {
            SocketConnection conn = FindObjectOfType<SocketConnection>(true);
            SocketIOComponent sio = conn == null ? null : conn.SocketIO.GetComponent<SocketIOComponent>();
            VehicleController vc = FindObjectOfType<VehicleController>(true);
            double age = lastFrameTicks == 0 ? -1 : (Stopwatch.GetTimestamp() - lastFrameTicks) / (double)Stopwatch.Frequency;
            var sb = new StringBuilder("{\"ok\":true");
            sb.Append(",\"want_connected\":").Append(want ? "true" : "false");
            sb.Append(",\"connect_enabled\":").Append(conn != null && conn.SocketIO.activeSelf ? "true" : "false");
            sb.Append(",\"connected\":").Append(sio != null && sio.IsConnected ? "true" : "false");
            sb.Append(",\"bridge\":").Append(Str(sio == null ? null : sio.IPAddress + ":" + sio.PortNumber));
            sb.Append(",\"fps\":").Append(fps.ToString("0", System.Globalization.CultureInfo.InvariantCulture));
            sb.Append(",\"frames_per_s\":").Append(messagesPerS.ToString("0", System.Globalization.CultureInfo.InvariantCulture));
            sb.Append(",\"last_frame_s\":").Append(age < 0 ? "null" : age.ToString("0.00", System.Globalization.CultureInfo.InvariantCulture));
            sb.Append(",\"track\":").Append(Str(TrackSelector.Current));
            sb.Append(",\"mode\":").Append(Str(vc == null ? null : vc.DrivingMode == 1 ? "Autonomous" : "Manual"));
            return sb.Append('}').ToString();
        }

        // ------------------------------------------------------------ connection

        // Keeps the sim connected while `want` is set, working around the sim's Socket.IO client:
        // - With no bridge answering, its socket thread calls Connect() in a loop with no pause, leaking a
        //   socket and logging a full stack trace each time (hundreds of MB of Player.log a minute, and
        //   "Too many open files" within seconds; then nothing in the sim can open a socket, and it never
        //   reconnects). Docker's port forwarding accepts the TCP connection even with the bridge down,
        //   so the client cannot tell. So the Connect objects stay disabled until a probe (one HTTP
        //   request, as the Socket.IO handshake starts) gets an answer from the bridge, and are disabled
        //   again as soon as the client reports its WebSocket closed (or frames stop for 1 s).
        // - The sim sends a frame on connect and then one per reply ("Bridge" in, EmitTelemetry out).
        //   The frame sent on connect is often lost (sent before the bridge has finished the handshake):
        //   the socket stays up and nothing is exchanged. If no reply came 1.5 s after connecting, send
        //   the frame again (every 2 s). Only while no reply has come: afterwards a second frame would
        //   start a second exchange running alongside the first. No reply within 5 s: start over.
        // The Connect button keeps working: pressing it sets `want` to what it now shows.
        void Supervise()
        {
            SocketConnection conn = FindObjectOfType<SocketConnection>(true);
            if (conn == null) return;
            bool enabled = conn.SocketIO.activeSelf;
            if (enabled != expectEnabled)   // the Connect button was pressed
            {
                want = expectEnabled = enabled;
                enabledAtRun = runTime;
                disconnectsAtEnable = Volatile.Read(ref disconnects);
                Debug.Log(Tag + "Connect button: " + (enabled ? "connect" : "disconnect"));
            }
            if (!want) return;
            if (enabled)
            {
                bool replied = lastFrameRun > enabledAtRun;
                float since = runTime - enabledAtRun;
                if (!replied && since > 1.5f && runTime - lastKickRun > 2f)
                {
                    lastKickRun = runTime;
                    Debug.Log(Tag + $"no reply from the bridge {since:0.0} s after connecting: " + Kick());
                }
                if (!replied && since > 5f) Disable("no reply from the bridge 5 s after connecting");
                // a session that had frames dropped: disable before the reconnect loop gets going
                // (a few "disconnect" events while a connection is first set up are left alone; 4+
                // connect/drop cycles before the first reply end it. Failed attempts that never connect
                // raise no event; the 5 s rule covers those)
                else if (!replied && Volatile.Read(ref disconnects) - disconnectsAtEnable > 3) Disable("the connection keeps dropping before the first reply");
                else if (replied && Volatile.Read(ref disconnects) != disconnectsAtEnable) Disable("the bridge closed the connection");
                else if (replied && runTime - lastFrameRun > 1f) Disable("no frames for 1 s (bridge stopped?)");
            }
            else if (probe == 2)
            {
                probe = 0;
                SetConnection(conn, true);
                expectEnabled = true;
                enabledAtRun = runTime;
                disconnectsAtEnable = Volatile.Read(ref disconnects);
                SocketIOComponent sio = conn.SocketIO.GetComponent<SocketIOComponent>();
                Debug.Log(Tag + $"bridge answers at {sio.IPAddress}:{sio.PortNumber}: connecting");
            }
            else if (probe != 1 && runTime - lastProbeRun > 1f)
            {
                lastProbeRun = runTime;
                SocketIOComponent sio = conn.SocketIO.GetComponent<SocketIOComponent>();
                if (sio != null) StartProbe(sio.IPAddress, sio.PortNumber);
            }
        }

        void Disable(string reason)
        {
            SocketConnection conn = FindObjectOfType<SocketConnection>(true);
            if (conn == null || !conn.SocketIO.activeSelf) return;
            SetConnection(conn, false);
            expectEnabled = false;
            // the sim keeps applying the last command it got: stop the car instead of driving on blind
            foreach (VehicleController v in FindObjectsOfType<VehicleController>(true)) { v.CurrentThrottle = 0f; v.CurrentSteeringAngle = 0f; }
            foreach (AutomobileController a in FindObjectsOfType<AutomobileController>(true)) { a.CurrentThrottle = 0f; a.CurrentSteeringAngle = 0f; }
            foreach (TwistController t in FindObjectsOfType<TwistController>(true)) { t.vSetpoint = 0f; t.wSetpoint = 0f; }
            lastProbeRun = runTime;   // next probe in 1 s
            Debug.Log(Tag + "disconnected: " + reason);
        }

        // One Socket.IO (Engine.IO 3) polling handshake request; the bridge answers 200 with a session id.
        void StartProbe(string ip, string port)
        {
            probe = 1;
            ThreadPool.QueueUserWorkItem(_ =>
            {
                bool ok = false;
                try
                {
                    using (var client = new TcpClient())
                    {
                        IAsyncResult ar = client.BeginConnect(ip, int.Parse(port), null, null);
                        if (ar.AsyncWaitHandle.WaitOne(500) && client.Connected)
                        {
                            client.EndConnect(ar);
                            NetworkStream stream = client.GetStream();
                            stream.ReadTimeout = 1000;
                            byte[] req = Encoding.ASCII.GetBytes($"GET /socket.io/?EIO=3&transport=polling HTTP/1.1\r\n" +
                                                                 $"Host: {ip}:{port}\r\nConnection: close\r\n\r\n");
                            stream.Write(req, 0, req.Length);
                            var buf = new byte[4096];
                            int n, got = 0;
                            while (got < buf.Length && (n = stream.Read(buf, got, buf.Length - got)) > 0) got += n;
                            string resp = Encoding.ASCII.GetString(buf, 0, got);
                            ok = resp.StartsWith("HTTP/1.1 200") && resp.Contains("sid");
                        }
                    }
                }
                catch (Exception) { }   // refused, reset by Docker's forwarder, or timed out: no bridge
                probe = ok ? 2 : 3;
            });
        }

        string Connect(string ip, string port)
        {
            SocketConnection conn = FindObjectOfType<SocketConnection>(true);
            if (conn == null) return "no Connection object in this scene";
            SocketIOComponent sio = conn.SocketIO.GetComponent<SocketIOComponent>();
            if (sio != null && ip != null) sio.IPAddress = ip;         // read in OnEnable, when connecting
            if (sio != null && port != null) sio.PortNumber = port;
            want = true;
            if (conn.SocketIO.activeSelf) return "already connected";
            return "connecting when the bridge answers at " + (sio == null ? "?" : sio.IPAddress + ":" + sio.PortNumber);
        }

        string Disconnect()
        {
            want = false;
            SocketConnection conn = FindObjectOfType<SocketConnection>(true);
            if (conn == null) return "no Connection object in this scene";
            if (!conn.SocketIO.activeSelf) return "already disconnected";
            Disable("disconnect command");
            return "disconnected";
        }

        // Same as the Connect button (SocketConnection.ToggleSocketConnection), whose own on/off
        // state is a private field: keep it in step so the button still works afterwards.
        static void SetConnection(SocketConnection conn, bool on)
        {
            FieldInfo enabled = typeof(SocketConnection).GetField("Enabled", BindingFlags.NonPublic | BindingFlags.Instance);
            bool state = enabled != null && (bool)enabled.GetValue(conn);
            if (state != on) conn.ToggleSocketConnection();
            conn.Socket.SetActive(on);
            conn.SocketIO.SetActive(on);
        }

        void Hook()
        {
            SocketConnection conn = FindObjectOfType<SocketConnection>(true);
            SocketIOComponent sio = conn == null ? null : conn.SocketIO.GetComponent<SocketIOComponent>();
            // its handler table is made in Awake, which runs when the object is first enabled
            if (sio == null || !conn.SocketIO.activeInHierarchy) return;
            sio.On("Bridge", e =>
            {
                Interlocked.Exchange(ref lastFrameTicks, Stopwatch.GetTimestamp());
                Interlocked.Increment(ref totalMessages);
            });
            sio.On("disconnect", e => Interlocked.Increment(ref disconnects));
            hooked = sio;
        }

        // Send one telemetry frame (the "kick" command; see Supervise).
        string Kick()
        {
            global::Socket s = FindObjectOfType<global::Socket>(true);
            if (s == null || !s.isActiveAndEnabled) return "not connected";
            MethodInfo emit = typeof(global::Socket).GetMethod("EmitTelemetry", BindingFlags.NonPublic | BindingFlags.Instance);
            emit.Invoke(s, new object[] { null });
            return "sent one telemetry frame";
        }

        // ------------------------------------------------------------ Map button

        string SwitchTrack(string name)
        {
            string chosen = TrackSelector.Switch(name);
            if (chosen != null)
            {
                Debug.Log(Tag + $"switched to '{chosen}'");
                SetLabel(mapButton, "Map: " + chosen);
                if (mapList != null) mapList.SetActive(false);
            }
            return chosen;
        }

        // The Menu panel (top-left button) lists the sim's controls as rows: an icon button with a
        // label to its right, anchored top to bottom (Connection, Driving Mode, Camera, Rendering
        // Quality, Scene Light, Reset, Quit). Map goes right after Camera; the rows are re-spaced
        // evenly over the same height. The track list opens to the right of the panel.
        void BuildMapButton()
        {
            SocketConnection conn = FindObjectOfType<SocketConnection>(true);
            if (conn == null || conn.transform.parent == null) return;   // Update tries again
            var menu = (RectTransform)conn.transform.parent;
            RectTransform[] rows = menu.Cast<Transform>().Where(t => t.GetComponent<Button>() != null)
                                       .Select(t => (RectTransform)t).OrderByDescending(r => r.anchorMax.y).ToList()
                                       .ToArray();
            RectTransform camera = rows.FirstOrDefault(r => r.GetComponent<CameraSwitch>() != null) ?? rows.Last();

            mapButton = CleanClone(camera.gameObject, menu, "Map");
            var map = (RectTransform)mapButton.transform;
            map.SetSiblingIndex(camera.GetSiblingIndex() + 1);
            var order = rows.ToList();
            order.Insert(order.IndexOf(camera) + 1, map);
            float top = rows.First().anchorMax.y, bottom = rows.Last().anchorMin.y, step = (top - bottom) / order.Count;
            for (int i = 0; i < order.Count; i++)
            {
                order[i].anchorMax = new Vector2(order[i].anchorMax.x, top - i * step);
                order[i].anchorMin = new Vector2(order[i].anchorMin.x, top - (i + 1) * step);
            }
            Image icon = mapButton.GetComponent<Image>();
            if (icon != null) icon.sprite = MapIcon();
            SetLabel(mapButton, "Map: " + (TrackSelector.Current ?? "?"));
            mapButton.GetComponent<Button>().onClick.AddListener(ToggleMapList);

            // the list, beside the panel at the Map row (a child of the panel: it hides with it)
            Text labelTemplate = mapButton.GetComponentsInChildren<Text>(true).First();
            Image panelImage = menu.GetComponent<Image>();
            mapList = new GameObject("Map List", typeof(RectTransform));
            var list = (RectTransform)mapList.transform;
            list.SetParent(menu, false);
            list.anchorMin = new Vector2(1f, map.anchorMax.y);
            list.anchorMax = new Vector2(1f, map.anchorMax.y);
            list.pivot = new Vector2(0f, 1f);
            const float itemW = 230f, itemH = 36f;
            list.sizeDelta = new Vector2(itemW, itemH * TrackSelector.Tracks.Count);
            list.anchoredPosition = new Vector2(4f, 0f);
            int k = 0;
            foreach (string track in TrackSelector.Tracks)
            {
                var item = new GameObject("Map " + track, typeof(RectTransform), typeof(CanvasRenderer), typeof(Image), typeof(Button));
                var irt = (RectTransform)item.transform;
                irt.SetParent(list, false);
                irt.anchorMin = irt.anchorMax = new Vector2(0f, 1f);
                irt.pivot = new Vector2(0f, 1f);
                irt.sizeDelta = new Vector2(itemW, itemH - 2f);
                irt.anchoredPosition = new Vector2(0f, -itemH * k++);
                Image bg = item.GetComponent<Image>();
                if (panelImage != null) { bg.sprite = panelImage.sprite; bg.type = panelImage.type; bg.color = panelImage.color; }
                else bg.color = new Color(0f, 0f, 0f, 0.7f);
                Text text = Instantiate(labelTemplate, irt);
                text.name = "Label";
                var trt = text.rectTransform;
                trt.anchorMin = Vector2.zero; trt.anchorMax = Vector2.one;
                trt.pivot = new Vector2(0.5f, 0.5f);
                trt.offsetMin = new Vector2(12f, 0f); trt.offsetMax = new Vector2(-8f, 0f);
                text.alignment = TextAnchor.MiddleLeft;
                text.resizeTextForBestFit = true; text.resizeTextMaxSize = text.fontSize; text.resizeTextMinSize = 10;
                text.text = track;
                string t = track;
                item.GetComponent<Button>().onClick.AddListener(() => SwitchTrack(t));
            }
            mapList.SetActive(false);
            Debug.Log(Tag + $"Map button after '{camera.name}' in '{menu.name}', {k} tracks");
        }

        void LogNoMenu()
        {
            Debug.LogWarning(Tag + "no Connection button (Menu panel) found in 20 s: no Map button");
            mapSearchUntil = 0f;
        }

        void ToggleMapList()
        {
            if (mapList == null) return;
            mapList.SetActive(!mapList.activeSelf);
            // right of the Map row's label (it runs past the panel's edge for long track names)
            Text label = mapButton.GetComponentInChildren<Text>(true);
            var menu = (RectTransform)mapList.transform.parent;
            if (label != null && mapList.activeSelf)
            {
                Rect r = label.rectTransform.rect;
                float right = menu.InverseTransformPoint(label.rectTransform.TransformPoint(new Vector3(r.xMin + label.preferredWidth, 0f))).x;
                ((RectTransform)mapList.transform).anchoredPosition = new Vector2(Mathf.Max(4f, right - menu.rect.xMax + 8f), 0f);
            }
            foreach (Transform item in mapList.transform)
                SetLabel(item.gameObject, (item.name == "Map " + TrackSelector.Current ? "> " : "") + item.name.Substring(4));
        }

        // A copy of one of the sim's buttons with only Unity UI components left (none of the sim's
        // scripts, e.g. Quit's or Reset's click handlers) and no click listeners.
        static GameObject CleanClone(GameObject source, Transform parent, string name)
        {
            GameObject go = Instantiate(source, parent);
            go.name = name;
            foreach (MonoBehaviour mb in go.GetComponentsInChildren<MonoBehaviour>(true))
                if (mb.GetType().Namespace != "UnityEngine.UI" && mb.GetType().Namespace != "UnityEngine.EventSystems")
                    DestroyImmediate(mb);
            foreach (Button b in go.GetComponentsInChildren<Button>(true)) b.onClick = new Button.ButtonClickedEvent();
            go.SetActive(true);
            return go;
        }

        static void SetLabel(GameObject button, string text)
        {
            if (button == null) return;
            Text label = button.GetComponentInChildren<Text>(true);
            if (label != null) label.text = text;
        }

        // A folded paper map in white (tinted by the button's Image colour, like the sim's icons).
        static Sprite mapIcon;
        static Sprite MapIcon()
        {
            if (mapIcon != null) return mapIcon;
            const int n = 128;
            var tex = new Texture2D(n, n, TextureFormat.RGBA32, false);
            var px = new Color32[n * n];
            Vector2[] outline = { new Vector2(12, 28), new Vector2(44, 12), new Vector2(84, 28), new Vector2(116, 12),
                                  new Vector2(116, 100), new Vector2(84, 116), new Vector2(44, 100), new Vector2(12, 116) };
            var segments = new System.Collections.Generic.List<(Vector2, Vector2)>();
            for (int i = 0; i < outline.Length; i++) segments.Add((outline[i], outline[(i + 1) % outline.Length]));
            segments.Add((new Vector2(44, 12), new Vector2(44, 100)));
            segments.Add((new Vector2(84, 28), new Vector2(84, 116)));
            const float half = 4.5f;
            for (int y = 0; y < n; y++)
                for (int x = 0; x < n; x++)
                {
                    var p = new Vector2(x + 0.5f, y + 0.5f);
                    float d = segments.Min(s => DistanceToSegment(p, s.Item1, s.Item2));
                    byte a = (byte)(255f * Mathf.Clamp01(half + 0.5f - d));   // anti-aliased edge
                    px[y * n + x] = new Color32(255, 255, 255, a);
                }
            tex.SetPixels32(px);
            tex.Apply();
            mapIcon = Sprite.Create(tex, new Rect(0, 0, n, n), new Vector2(0.5f, 0.5f), 100f);
            return mapIcon;
        }

        static float DistanceToSegment(Vector2 p, Vector2 a, Vector2 b)
        {
            Vector2 ab = b - a;
            float t = Mathf.Clamp01(Vector2.Dot(p - a, ab) / ab.sqrMagnitude);
            return Vector2.Distance(p, a + t * ab);
        }

        // The pointer events a mouse click sends (enter, down, up, click, exit), to the object or the
        // first parent that handles each, as Unity's input module does. So both Button.onClick and
        // scripts with their own IPointerClickHandler (the sim's Reset) see a real click.
        static string Click(GameObject target, PointerEventData data)
        {
            EventSystem es = EventSystem.current;
            if (es == null) return Json(false, "error", "no EventSystem");
            if (data == null)
            {
                var rt = target.transform as RectTransform;
                Vector2 pos = rt == null ? Vector2.zero : RectTransformUtility.WorldToScreenPoint(null, rt.TransformPoint(rt.rect.center));
                data = new PointerEventData(es) { position = pos };
            }
            data.button = PointerEventData.InputButton.Left;
            data.clickCount = 1;
            data.pointerPressRaycast = data.pointerCurrentRaycast = new RaycastResult { gameObject = target };
            ExecuteEvents.ExecuteHierarchy(target, data, ExecuteEvents.pointerEnterHandler);
            GameObject down = ExecuteEvents.ExecuteHierarchy(target, data, ExecuteEvents.pointerDownHandler);
            data.pointerPress = down ?? ExecuteEvents.GetEventHandler<IPointerClickHandler>(target);
            data.rawPointerPress = target;
            ExecuteEvents.Execute(data.pointerPress, data, ExecuteEvents.pointerUpHandler);
            GameObject clicked = ExecuteEvents.ExecuteHierarchy(target, data, ExecuteEvents.pointerClickHandler);
            ExecuteEvents.ExecuteHierarchy(target, data, ExecuteEvents.pointerExitHandler);
            return "{\"ok\":" + (clicked != null ? "true" : "false") + ",\"hit\":" + Str(target.name) +
                   ",\"clicked\":" + Str(clicked == null ? null : clicked.name) + "}";
        }

        static void Tree(Transform t, int level, int depth, StringBuilder sb)
        {
            sb.Append(new string(' ', 2 * level)).Append(t.name).Append(t.gameObject.activeSelf ? "" : " (inactive)");
            if (t is RectTransform r)
                sb.Append($" pos={r.anchoredPosition} size={r.rect.size} anchors={r.anchorMin}-{r.anchorMax} pivot={r.pivot}");
            sb.Append(" [").Append(string.Join(",", t.GetComponents<Component>().Select(c => c.GetType().Name))).Append(']');
            Text text = t.GetComponent<Text>();
            if (text != null) sb.Append(" '").Append(text.text).Append('\'');
            Button b = t.GetComponent<Button>();
            if (b != null)
                for (int i = 0; i < b.onClick.GetPersistentEventCount(); i++)
                    sb.Append($" onClick={b.onClick.GetPersistentTarget(i)?.GetType().Name}.{b.onClick.GetPersistentMethodName(i)}");
            sb.Append('\n');
            if (level < depth)
                foreach (Transform c in t) Tree(c, level + 1, depth, sb);
        }

        // ------------------------------------------------------------ JSON

        static string Str(string s) =>
            s == null ? "null" : "\"" + s.Replace("\\", "\\\\").Replace("\"", "\\\"").Replace("\n", "\\n") + "\"";

        static string Json(bool ok, string key, string value) =>
            "{\"ok\":" + (ok ? "true" : "false") + ",\"" + key + "\":" + Str(value) + "}";
    }
}
