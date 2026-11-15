// Slow-frame watchdog: once per frame (SimControl.Update) checks how long the last frame took; over
// 0.1 s it logs one line, at most once a second (the rest are counted in the next line):
//   [Perf] frame took 1043 ms (GC gen0 +1 gen1 +0 gen2 +0, heap 412 MB; map view 3.1 ms, map fit 12.0 ms, car link emit 0.8 ms)
// The component times are what our own code spent since the previous frame's check (Stopwatch around
// MapView.LateUpdate, its accuracy fit, and CarLink.Emit), so a long frame with small numbers there and
// a GC count went into a collection or into the stock sim. ctl status: slow_frames, last_slow_frame_ms.
using System;
using System.Diagnostics;
using Debug = UnityEngine.Debug;

namespace TrackSelect
{
    public static class Perf
    {
        public const float SlowFrame = 0.1f;   // s
        public static int SlowFrames { get; private set; }
        public static float LastSlowMs { get; private set; } = float.NaN;

        // Stopwatch ticks spent since the last Frame(), main thread only
        public static long MapViewTicks, MapFitTicks, EmitTicks;

        static int gc0 = -1, gc1, gc2;
        static float nextLog;
        static int unlogged;

        static double Ms(long ticks) => ticks * 1000.0 / Stopwatch.Frequency;

        /// <summary>Once per frame, with Time.unscaledDeltaTime and Time.realtimeSinceStartup.</summary>
        public static void Frame(float dt, float now, bool hitch)
        {
            int g0 = GC.CollectionCount(0), g1 = GC.CollectionCount(1), g2 = GC.CollectionCount(2);
            if (gc0 < 0) { gc0 = g0; gc1 = g1; gc2 = g2; }
            if (dt > SlowFrame)
            {
                SlowFrames++;
                LastSlowMs = dt * 1000f;
                if (now >= nextLog)
                {
                    nextLog = now + 1f;
                    Debug.Log($"[Perf] frame took {dt * 1000f:0} ms (GC gen0 +{g0 - gc0} gen1 +{g1 - gc1} gen2 +{g2 - gc2}, " +
                              $"heap {GC.GetTotalMemory(false) >> 20} MB; map view {Ms(MapViewTicks):0.0} ms, " +
                              $"map fit {Ms(MapFitTicks):0.0} ms, car link emit {Ms(EmitTicks):0.0} ms)" +
                              (hitch ? " [ctl hitch]" : "") + (unlogged > 0 ? $"; {unlogged} more slow frame(s) not logged" : ""));
                    unlogged = 0;
                }
                else unlogged++;
            }
            gc0 = g0; gc1 = g1; gc2 = g2;
            MapViewTicks = MapFitTicks = EmitTicks = 0;
        }
    }
}
