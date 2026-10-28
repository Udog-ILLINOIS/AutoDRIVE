// The map view's accuracy line (MapView, under the figure): how far the stack's map and its pose
// estimate are from the truth.
//   Map           mean distance from the map's wall cells (occupied, as save_map counts them) to the
//                 nearest real wall surface facing the track (the track's mesh, rasterized at 2 cm)
//   Localization  mean distance from the estimated base_link to where the car's rear axle really was at
//                 the time of that estimate, over the last 5 s
// Both are measured with the map placed where its walls best fit the real ones: starting from the rigid
// transform that best fits the estimates of the last 60 s to the truth (least squares; with too little
// motion for that, the mean heading difference), refined to the least mean wall distance. So the
// localization number is where the car thinks it is on its own map against where it is on the real
// track; the trajectory fit alone mixes map frames from before and after Cartographer's loop closures
// while mapping. The latched map_to_world.yaml is a few cm off, so it is not used.
// Without poses (no localization or mapping running, only the last map), the map is placed by its walls
// alone (RegisterToWalls): every 5 deg around, the walls' centre of mass on the real walls' one, the
// four best refined, the best kept if its walls lie within 8 cm of the real ones on average, or within
// 20 cm and clearly better (1.4 times) than the best placement turned more than 20 deg from it (a map
// that matches the walls less well, but only one way round). So the
// Mapping tab has the Ground Truth tab's orientation then too; only the map error is shown.
// The placement belongs to the map, not to the poses: a reset or teleport clears the pose pairs but
// keeps the last placement, and each refit starts from whichever of the trajectory fit, the last
// placement and the walls-alone placement fits the real walls best. Poses that are wrong for a while
// (MCL keeps TF flowing while it relocalizes after a sim reset, so map_view sends a stale pose for
// ~2-3 s) then only raise the localization error instead of turning the map.
// Each estimate carries its age (map_view), and is compared with the car's pose that long ago,
// interpolated between rendered frames. Both are reset on a track switch and when the car is moved
// (a reset or a teleport).
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using UnityEngine;

namespace TrackSelect
{
    public partial class MapView
    {
        const float TruthCell = 0.02f;   // m; coarser for large tracks
        const int TruthMaxSide = 2048;
        const double FitWindow = 60.0, LocWindow = 5.0;   // s

        struct Sample { public long t; public float x, y, yaw; }

        readonly ConcurrentQueue<Sample> estimates = new ConcurrentQueue<Sample>();   // from the reader thread
        readonly List<Sample> pending = new List<Sample>();   // estimates newer than the last rendered frame
        readonly List<Sample> truthHistory = new List<Sample>();   // the car's base_link, the last 2 s
        readonly List<(Sample est, Sample truth)> pairs = new List<(Sample, Sample)>();
        float[] wallDistance;   // m from each truth cell to the nearest wall surface
        Raster truthRaster;
        Vector2 truthCentre;   // centre of mass of the real wall surface (world)
        bool registered; int registeredSeq = -1;   // a placement from the walls alone, for that map
        float regCos = 1f, regSin, regX, regY;
        bool fitted; float fitCos, fitSin, fitX, fitY;   // map -> world
        bool placed; float placedCos = 1f, placedSin, placedX, placedY;   // the last placement shown, until a track switch
        float nextStartLog;
        float mapError = float.NaN, locError = float.NaN;
        float nextScore;

        static double Seconds(long ticks) => ticks / (double)Stopwatch.Frequency;

        void ClearAccuracy()
        {
            while (estimates.TryDequeue(out _)) { }
            pending.Clear(); truthHistory.Clear(); pairs.Clear();
            fitted = false;
            placed = false;
            alignedSeq = -1;
            mapError = locError = float.NaN;
        }

        string AccuracyText()
        {
            string Cm(float v) => float.IsNaN(v) ? "–" : $"{v * 100f:0.0} cm";
            return $"Map error  {Cm(mapError)}        Localization error  {Cm(locError)}";
        }

        // ------------------------------------------------------------ the truth

        void SetTruth(Raster r)
        {
            truthRaster = r;
            // chamfer distance transform from the surface cells (3x3, exact along rows and diagonals)
            int w = r.w, h = r.h;
            var d = new float[w * h];
            for (int j = 0; j < h; j++)
                for (int i = 0; i < w; i++) d[j * w + i] = r.Surface(i, j) ? 0f : float.MaxValue;
            float a = r.cell, b = r.cell * 1.41421356f;
            for (int j = 0; j < h; j++)
                for (int i = 0; i < w; i++)
                {
                    int k = j * w + i; float v = d[k];
                    if (i > 0) v = Math.Min(v, d[k - 1] + a);
                    if (j > 0)
                    {
                        v = Math.Min(v, d[k - w] + a);
                        if (i > 0) v = Math.Min(v, d[k - w - 1] + b);
                        if (i < w - 1) v = Math.Min(v, d[k - w + 1] + b);
                    }
                    d[k] = v;
                }
            for (int j = h - 1; j >= 0; j--)
                for (int i = w - 1; i >= 0; i--)
                {
                    int k = j * w + i; float v = d[k];
                    if (i < w - 1) v = Math.Min(v, d[k + 1] + a);
                    if (j < h - 1)
                    {
                        v = Math.Min(v, d[k + w] + a);
                        if (i < w - 1) v = Math.Min(v, d[k + w + 1] + b);
                        if (i > 0) v = Math.Min(v, d[k + w - 1] + b);
                    }
                    d[k] = v;
                }
            wallDistance = d;
            Vector2 sum = Vector2.zero; int ns = 0;
            for (int j = 0; j < h; j++)
                for (int i = 0; i < w; i++)
                    if (d[j * w + i] == 0f) { sum += new Vector2(r.x0 + (i + 0.5f) * r.cell, r.y0 + (j + 0.5f) * r.cell); ns++; }
            truthCentre = ns > 0 ? sum / ns : Vector2.zero;
            registeredSeq = -1;   // place the map again against the new truth
            nextScore = 0f;   // score the map against the new truth right away
        }

        // m from world point (x, y) to the nearest real wall surface
        float DistanceToWall(float x, float y)
        {
            Raster r = truthRaster;
            float fi = (x - r.x0) / r.cell - 0.5f, fj = (y - r.y0) / r.cell - 0.5f;
            int i = Mathf.Clamp(Mathf.RoundToInt(fi), 0, r.w - 1), j = Mathf.Clamp(Mathf.RoundToInt(fj), 0, r.h - 1);
            float outside = new Vector2(Mathf.Max(0f, Math.Abs(fi - i) - 0.5f), Mathf.Max(0f, Math.Abs(fj - j) - 0.5f)).magnitude * r.cell;
            return wallDistance[j * r.w + i] + outside;
        }

        // The car's base_link (rear axle) in the world, once per rendered frame.
        void RecordTruth()
        {
            if (!FindCar()) return;
            Vector3 p = car.position, f = car.forward;
            float yaw = Mathf.Atan2(-f.x, f.z);
            var s = new Sample { t = Stopwatch.GetTimestamp(), x = p.z - carRearAxle * Mathf.Cos(yaw),
                                 y = -p.x - carRearAxle * Mathf.Sin(yaw), yaw = yaw };
            if (truthHistory.Count > 0)
            {
                Sample last = truthHistory[truthHistory.Count - 1];
                // faster than the car can drive: reset or teleported; the estimates restart too
                double dt = Math.Max(Seconds(s.t - last.t), 0.02);
                if (new Vector2(s.x - last.x, s.y - last.y).magnitude > 0.5f + 30.0 * dt)
                {
                    pending.Clear(); truthHistory.Clear(); pairs.Clear(); fitted = false;
                    locError = mapError = float.NaN;
                }
            }
            truthHistory.Add(s);
            long cut = s.t - 2 * Stopwatch.Frequency;
            int old = 0;
            while (old < truthHistory.Count - 2 && truthHistory[old].t < cut) old++;
            if (old > 0) truthHistory.RemoveRange(0, old);
        }

        // ------------------------------------------------------------ per frame

        void UpdateAccuracy()
        {
            RecordTruth();
            while (estimates.TryDequeue(out Sample e)) pending.Add(e);
            int n = truthHistory.Count;
            for (int k = pending.Count - 1; k >= 0; k--)
            {
                Sample e = pending[k];
                if (n < 2 || e.t > truthHistory[n - 1].t) continue;   // wait for a frame after it
                pending.RemoveAt(k);
                if (e.t < truthHistory[0].t) continue;                // older than the history
                int b = 1;
                while (truthHistory[b].t < e.t) b++;
                Sample t0 = truthHistory[b - 1], t1 = truthHistory[b];
                float u = t1.t == t0.t ? 1f : (float)((e.t - t0.t) / (double)(t1.t - t0.t));
                pairs.Add((e, new Sample { t = e.t, x = Mathf.Lerp(t0.x, t1.x, u), y = Mathf.Lerp(t0.y, t1.y, u),
                                           yaw = t0.yaw + u * Mathf.DeltaAngle(t0.yaw * Mathf.Rad2Deg, t1.yaw * Mathf.Rad2Deg) * Mathf.Deg2Rad }));
            }
            if (pending.Count > 500) pending.RemoveRange(0, pending.Count - 500);
            long now = Stopwatch.GetTimestamp();
            int stale = 0;
            while (stale < pairs.Count && Seconds(now - pairs[stale].est.t) > FitWindow) stale++;
            if (stale > 0) pairs.RemoveRange(0, stale);

            if (Time.realtimeSinceStartup < nextScore)
            {
                if (alignedSeq == mapSeqShown) return;
                // a new map: with poses, at once in the last fit; without, placed by its walls right now
                if (fitted) { DrawAligned(); return; }
            }
            nextScore = Time.realtimeSinceStartup + 0.5f;
            Fit();
            List<Vector2> walls = MapWalls();
            if (walls.Count >= 20 && wallDistance != null)
            {
                if (!fitted && registeredSeq != mapSeqShown) { RegisterToWalls(walls); registeredSeq = mapSeqShown; }
                PickStart(walls);
                if (fitted) AlignToWalls(walls);
            }
            else if (!fitted && placed) { fitCos = placedCos; fitSin = placedSin; fitX = placedX; fitY = placedY; fitted = true; }
            if (fitted) { placed = true; placedCos = fitCos; placedSin = fitSin; placedX = fitX; placedY = fitY; }
            ScoreMap(walls);
            ScoreLocalization(now);
            DrawAligned();
        }

        // map -> world: least squares over the window's positions (2D Kabsch), or the mean heading
        // difference when the car has hardly moved (a rotation is not defined by points close together)
        void Fit()
        {
            int n = pairs.Count;
            if (n < 10) { fitted = false; return; }
            double ex = 0, ey = 0, gx = 0, gy = 0;
            foreach (var (e, g) in pairs) { ex += e.x; ey += e.y; gx += g.x; gy += g.y; }
            ex /= n; ey /= n; gx /= n; gy /= n;
            double sxx = 0, sxy = 0, syx = 0, syy = 0, spread = 0, hc = 0, hs = 0;
            foreach (var (e, g) in pairs)
            {
                double ax = e.x - ex, ay = e.y - ey, bx = g.x - gx, by = g.y - gy;
                sxx += ax * bx; sxy += ax * by; syx += ay * bx; syy += ay * by;
                spread += ax * ax + ay * ay;
                hc += Math.Cos(g.yaw - e.yaw); hs += Math.Sin(g.yaw - e.yaw);
            }
            double th = spread / n > 0.25 ? Math.Atan2(sxy - syx, sxx + syy) : Math.Atan2(hs, hc);
            fitCos = (float)Math.Cos(th); fitSin = (float)Math.Sin(th);
            fitX = (float)(gx - (fitCos * ex - fitSin * ey));
            fitY = (float)(gy - (fitSin * ex + fitCos * ey));
            fitted = true;
        }

        void ScoreLocalization(long now)
        {
            if (!fitted) { locError = float.NaN; return; }
            double sum = 0; int n = 0;
            for (int k = pairs.Count - 1; k >= 0 && Seconds(now - pairs[k].est.t) <= LocWindow; k--)
            {
                var (e, g) = pairs[k];
                float x = fitCos * e.x - fitSin * e.y + fitX, y = fitSin * e.x + fitCos * e.y + fitY;
                sum += new Vector2(x - g.x, y - g.y).magnitude; n++;
            }
            locError = n == 0 ? float.NaN : (float)(sum / n);
        }

        // the map's wall cells (occupied, as save_map counts them), their centres in the map frame
        List<Vector2> MapWalls()
        {
            var pts = new List<Vector2>();
            byte[] cells; int w, h; float res, ox, oy, yaw;
            lock (sync) { cells = mapCells; w = mapW; h = mapH; res = mapRes; ox = mapOx; oy = mapOy; yaw = mapYaw; }
            if (cells == null) return pts;
            float c = Mathf.Cos(yaw), s = Mathf.Sin(yaw);
            for (int j = 0; j < h; j++)
                for (int i = 0; i < w; i++)
                {
                    if ((sbyte)cells[j * w + i] < OccupiedPct) continue;
                    float gx = (i + 0.5f) * res, gy = (j + 0.5f) * res;
                    pts.Add(new Vector2(ox + c * gx - s * gy, oy + s * gx + c * gy));
                }
            return pts;
        }

        float WallCost(List<Vector2> pts, int step, float th, float tx, float ty, float cap)
        {
            float c = Mathf.Cos(th), s = Mathf.Sin(th);
            double sum = 0; int n = 0;
            for (int k = 0; k < pts.Count; k += step)
            {
                Vector2 p = pts[k];
                sum += Mathf.Min(DistanceToWall(c * p.x - s * p.y + tx, s * p.x + c * p.y + ty), cap);
                n++;
            }
            return (float)(sum / n);
        }

        // Pattern search from the trajectory fit to the least mean distance (capped at 0.3 m, so stray
        // cells do not pull) of the map's walls to the real ones. Along a straight the cost is flat and
        // the fit is kept.
        void AlignToWalls(List<Vector2> pts, float dt = 0.05f, float da = 0.5f * Mathf.Deg2Rad)
        {
            int step = Math.Max(1, pts.Count / 4000);
            const float cap = 0.3f;
            float th = Mathf.Atan2(fitSin, fitCos), tx = fitX, ty = fitY;
            float best = WallCost(pts, step, th, tx, ty, cap);
            // turns are about the walls' centre of mass, so a turn does not also shift them
            Vector2 m = Vector2.zero;
            foreach (Vector2 p in pts) m += p;
            m /= pts.Count;
            for (int it = 0; it < 60 && dt > 0.002f; it++)
            {
                bool moved = false;
                foreach (var (a, x, y) in new[] { (da, 0f, 0f), (-da, 0f, 0f), (0f, dt, 0f), (0f, -dt, 0f), (0f, 0f, dt), (0f, 0f, -dt) })
                {
                    float c0 = Mathf.Cos(th), s0 = Mathf.Sin(th), c1 = Mathf.Cos(th + a), s1 = Mathf.Sin(th + a);
                    float nx = tx + x + (c0 - c1) * m.x - (s0 - s1) * m.y;
                    float ny = ty + y + (s0 - s1) * m.x + (c0 - c1) * m.y;
                    float c = WallCost(pts, step, th + a, nx, ny, cap);
                    if (c < best - 1e-5f) { best = c; th += a; tx = nx; ty = ny; moved = true; }
                }
                if (!moved) { dt /= 2f; da /= 2f; }
            }
            fitCos = Mathf.Cos(th); fitSin = Mathf.Sin(th); fitX = tx; fitY = ty;
        }

        // The start for AlignToWalls: of the trajectory fit (if any), the last placement and the
        // walls-alone placement, the one whose walls lie closest to the real ones (see the header).
        void PickStart(List<Vector2> pts)
        {
            var c = new List<(string what, float cos, float sin, float x, float y)>();
            if (fitted) c.Add(("trajectory fit", fitCos, fitSin, fitX, fitY));
            if (placed) c.Add(("last placement", placedCos, placedSin, placedX, placedY));
            if (registered) c.Add(("walls alone", regCos, regSin, regX, regY));
            if (c.Count == 0) return;
            int step = Math.Max(1, pts.Count / 4000);
            var cost = c.Select(k => WallCost(pts, step, Mathf.Atan2(k.sin, k.cos), k.x, k.y, 0.3f)).ToList();
            int best = cost.IndexOf(cost.Min());
            if (fitted && best != 0 && Time.realtimeSinceStartup >= nextStartLog)
            {
                nextStartLog = Time.realtimeSinceStartup + 5f;
                float turn = Mathf.DeltaAngle(Mathf.Atan2(c[best].sin, c[best].cos) * Mathf.Rad2Deg, Mathf.Atan2(fitSin, fitCos) * Mathf.Rad2Deg);
                UnityEngine.Debug.Log(Tag + $"trajectory fit ignored (turned {turn:0.0} deg, walls {cost[0] * 100f:0.0} cm off): " +
                          $"kept the {c[best].what} ({cost[best] * 100f:0.0} cm)");
            }
            (_, fitCos, fitSin, fitX, fitY) = c[best];
            fitted = true;
        }

        // Place the map by its walls alone (no poses): see the header.
        void RegisterToWalls(List<Vector2> pts)
        {
            const float cap = 0.3f;
            Vector2 m = Vector2.zero;
            foreach (Vector2 p in pts) m += p;
            m /= pts.Count;
            int coarse = Math.Max(1, pts.Count / 600);
            var tried = new List<(float cost, float th)>();
            for (int k = 0; k < 72; k++)
            {
                float th = k * 5f * Mathf.Deg2Rad, c = Mathf.Cos(th), s = Mathf.Sin(th);
                float tx = truthCentre.x - (c * m.x - s * m.y), ty = truthCentre.y - (s * m.x + c * m.y);
                tried.Add((WallCost(pts, coarse, th, tx, ty, cap), th));
            }
            float best = float.MaxValue;
            registered = false;
            var refined = new List<(float cost, float th)>();
            // the best four, and the best turned well away from the first (the alternative to beat)
            var starts = tried.OrderBy(t => t.cost).Take(4).ToList();
            float first = starts[0].th;
            starts.Add(tried.Where(t => Math.Abs(Mathf.DeltaAngle(t.th * Mathf.Rad2Deg, first * Mathf.Rad2Deg)) > 20f)
                            .OrderBy(t => t.cost).First());
            foreach (var (_, th0) in starts)
            {
                fitCos = Mathf.Cos(th0); fitSin = Mathf.Sin(th0);
                fitX = truthCentre.x - (fitCos * m.x - fitSin * m.y);
                fitY = truthCentre.y - (fitSin * m.x + fitCos * m.y);
                AlignToWalls(pts, 0.2f, 3f * Mathf.Deg2Rad);
                float cost = WallCost(pts, Math.Max(1, pts.Count / 4000), Mathf.Atan2(fitSin, fitCos), fitX, fitY, cap);
                refined.Add((cost, Mathf.Atan2(fitSin, fitCos)));
                if (cost < best) { best = cost; regCos = fitCos; regSin = fitSin; regX = fitX; regY = fitY; }
            }
            float bestTh = Mathf.Atan2(regSin, regCos);
            float other = refined.Where(r => Math.Abs(Mathf.DeltaAngle(r.th * Mathf.Rad2Deg, bestTh * Mathf.Rad2Deg)) > 20f)
                                 .Select(r => r.cost).DefaultIfEmpty(float.MaxValue).Min();
            registered = best < 0.08f || (best < 0.2f && best * 1.4f < other);
            UnityEngine.Debug.Log(Tag + $"map placed by its walls alone: {Mathf.Atan2(regSin, regCos) * Mathf.Rad2Deg:0.0} deg, " +
                      $"walls {best * 100f:0.0} cm from the real ones on average (other turns: {other * 100f:0.0} cm)" +
                      (registered ? "" : ": not clear, not used"));
        }

        void ScoreMap(List<Vector2> pts)
        {
            if (!fitted || pts.Count == 0 || wallDistance == null) { mapError = float.NaN; return; }
            double sum = 0;
            foreach (Vector2 p in pts) sum += DistanceToWall(fitCos * p.x - fitSin * p.y + fitX, fitSin * p.x + fitCos * p.y + fitY);
            mapError = (float)(sum / pts.Count);
        }
    }
}
