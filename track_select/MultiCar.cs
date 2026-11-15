// More than one car in the AutoDRIVE RoboRacer sim, each driven by its own bridge on its own port.
//
// Launch arguments:
//   -cars <n>             cars on track (default 1: the scene as built, nothing here runs)
//   -contact <0|1>        1 (default): the cars collide with each other. 0: ghosts, each car's lidar
//                         still sees the others but the bodies pass through each other
//   -ghost <0|1>          the Menu's Ghost switch: 1 = -contact 0 -lidar-cars 0 and the ghost look,
//                         0 = -contact 1 -lidar-cars 1 (both of those still set one part alone)
//   -lidar-cars <0|1>     1 (default): the lidars see the other cars; 0: they do not, in either mode
//                         (the cars' colliders go to the Ignore Raycast layer: the sim's LIDAR casts on
//                         Default only; the cars still collide with the track, and with each other in contact mode)
//   -car-ports <p2,p3..>  bridge port of car 2, 3, ... (default: car 1's port + 1, + 2, ...)
//   -grid "<b,l>;<b,l>"   start of car 2, 3, ... in metres behind (b) and left of (l) car 1's start
//                         (default: 1.5 m behind per car, 0.4 m to the right, then left, alternating;
//                         the tracks' own "Spawn 2" is not a grid slot: Porto's is elsewhere on the track)
//
// One car: the scene as built, on the sim's own Socket (-ip/-port, 4567, Connect button, SimControl's
// supervision), nothing here changes it. 2+ cars: car 1 is the scene's own "Roboracer" and cars 2.. are
// copies of it ("Roboracer 2", ...), every one (car 1 too, on the sim's port) driven through a CarLink: the sim's own Socket.IO client on that car's port
// and the same frame exchange as the sim's Socket ("Bridge" in, telemetry out, one per reply) with
// the same "V1 ..." keys, so an unchanged bridge (fast_bridge) drives any car on its own port. Each
// frame also carries "V1 Sim Time" (s, Unity time) and "V1 Frame" (render frame), plus "V1 Car".
//
// hold / start (ctl): held cars get zero throttle and steering whatever their bridges ask, while the
// requests are recorded (ctl cars: throttle_req, wants_to_drive); start releases them all in one frame
// with their lap timers restarted. race_cars.sh starts every driver held, then releases them together.
// Ghosts: Physics.IgnoreCollision between every collider pair of two different cars. Raycasts do
// not use the collision matrix, so the lidars still see the other cars.
// Log lines start with [MultiCar].
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using SocketIO;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

namespace TrackSelect
{
    public class Car
    {
        public int Index;                 // 1-based; car 1 is the scene's own
        public GameObject Root;
        public Rigidbody Body;
        public VehicleController Vc;
        public LapTimer Lap;
        public GPS Gps;
        public IMU Imu;
        public LIDAR Lidar;
        public WheelEncoder LeftEnc, RightEnc;
        public CarLink Link;              // null with one car (the sim's own Socket)
        public bool Parked;               // stopped in place by the sim (finished its race): commands ignored
        public Vector3 StartPos;
        public Quaternion StartRot;
        public WheelProbe Probe;          // 2+ cars: what each wheel stands on (another car = a physics bug)
    }

    /// <summary>Diagnostics: every physics step, what each of the car's WheelColliders has as ground. A
    /// wheel on another car's collider (ghosts overlap) gets that car's surface instead of the track's.
    /// ctl cars: wheel_on_car (steps with any wheel on another car), wheel_steps, wheel_on_car_last.</summary>
    public class WheelProbe : MonoBehaviour
    {
        public Car Car;
        public long Steps, OnCar, OnOther;
        public string Last = "";
        WheelCollider[] wheels;

        void FixedUpdate()
        {
            if (Car == null || Car.Root == null) return;
            if (wheels == null) wheels = Car.Root.GetComponentsInChildren<WheelCollider>(true);
            Steps++;
            bool car = false, other = false;
            foreach (WheelCollider w in wheels)
            {
                if (w == null || !w.GetGroundHit(out WheelHit hit) || hit.collider == null) continue;
                Rigidbody rb = hit.collider.attachedRigidbody;
                if (rb != null && rb != Car.Body && MultiCar.Cars.Any(c => c.Body == rb))
                {
                    car = true; Last = w.name + " on " + hit.collider.name + " (" + rb.name + ")";
                }
                else if (rb != null && rb == Car.Body)
                {
                    other = true; Last = w.name + " on its own " + hit.collider.name;
                }
            }
            if (car) OnCar++;
            if (other) OnOther++;
        }
    }

    public static class MultiCar
    {
        const string Tag = "[MultiCar] ";
        public static int Count = 1;
        public static bool Contact = true;
        public static bool LidarSeesCars = true;   // the lidars see the other cars (either mode)
        const int IgnoreRaycast = 2;             // Unity's built-in layer; LIDAR's layer_mask is 1 (Default)
        static readonly Dictionary<GameObject, int> layers = new Dictionary<GameObject, int>();
        static int[] ports;
        static Vector2[] grid;            // (behind, left) per car index 2.. (index 0 = car 2)
        public static readonly List<Car> Cars = new List<Car>();
        public static Car Primary => Cars.Count > 0 ? Cars[0] : null;
        /// <summary>Car 1 talks to its bridge through its own CarLink (default, any car count), not the sim's
        /// stock Socket: every frame then carries V1 Sim Time, which the bridge stamps messages with (a
        /// frame hitch no longer shows up as a gap in the stamps). -carlink 0: the stock Socket with one car.</summary>
        public static bool UseCarLink = true;
        public static bool Linked => Cars.Count > 0 && Cars[0].Link != null;
        /// <summary>Car 1's link wants a connection (one car): -autoconnect, then ctl connect / disconnect or
        /// the Connect button; kept across scene reloads (track switches).</summary>
        public static bool? WantConnected;

        public static void Init()
        {
            string[] a = Environment.GetCommandLineArgs();
            for (int i = 0; i < a.Length - 1; i++)
            {
                string k = a[i].ToLower(), v = a[i + 1];
                if (k == "-cars") int.TryParse(v, out Count);
                else if (k == "-contact") Contact = v != "0" && v.ToLower() != "false";
                else if (k == "-ghost") { bool g = v != "0" && v.ToLower() != "false"; Contact = !g; LidarSeesCars = !g; }
                else if (k == "-carlink") UseCarLink = v != "0" && v.ToLower() != "false";
                else if (k == "-lidar-cars") LidarSeesCars = v != "0" && v.ToLower() != "false";
                else if (k == "-car-ports") ports = v.Split(',').Select(s => int.Parse(s.Trim())).ToArray();
                else if (k == "-grid")
                    grid = v.Split(';').Select(p => p.Split(','))
                            .Select(p => new Vector2(F(p[0]), p.Length > 1 ? F(p[1]) : 0f)).ToArray();
            }
            Count = Mathf.Clamp(Count, 1, 8);
            SceneManager.sceneLoaded += (scene, mode) => { try { Setup(); } catch (Exception e) { Debug.LogError(Tag + e); } };
        }

        static float F(string s) => float.Parse(s.Trim(), CultureInfo.InvariantCulture);

        /// <summary>The scene's own car (the "Roboracer" root object), or null.</summary>
        public static GameObject PrimaryRoot() =>
            SceneManager.GetActiveScene().GetRootGameObjects().FirstOrDefault(g => g.name == "Roboracer");

        static Car Describe(GameObject root, int index)
        {
            return new Car
            {
                Index = index, Root = root,
                Body = root.GetComponent<Rigidbody>(),
                Vc = root.GetComponent<VehicleController>(),
                Lap = root.GetComponent<LapTimer>(),
                Gps = root.GetComponentInChildren<GPS>(true),
                Imu = root.GetComponentInChildren<IMU>(true),
                Lidar = root.GetComponentInChildren<LIDAR>(true),
                LeftEnc = root.GetComponentsInChildren<WheelEncoder>(true).FirstOrDefault(e => e.name.StartsWith("Left")),
                RightEnc = root.GetComponentsInChildren<WheelEncoder>(true).FirstOrDefault(e => e.name.StartsWith("Right")),
            };
        }

        static void Setup()
        {
            Cars.Clear();
            // a scene (re)load destroys the cars these were keyed by
            layers.Clear(); original.Clear(); decals.Clear(); shadows.Clear();
            faintMat = null; baseMat = null;
            Held = false; LaunchUntil = -1;
            GameObject first = PrimaryRoot();
            if (first == null) { Debug.LogError(Tag + "no Roboracer object"); return; }
            Cars.Add(Describe(first, 1));
            if (Count <= 1 && !UseCarLink) { PlaceGrid(); return; }   // the stock Socket; car 1's start pose
            int basePort = 4567;
            var sio = UnityEngine.Object.FindObjectOfType<SocketIOComponent>(true);
            if (sio != null) int.TryParse(sio.PortNumber, out basePort);
            // car 1 gets a CarLink on the sim's own port (the sim's Socket stays disconnected; SimControl
            // leaves it alone), so every car has the same exchange: hold/start, sim time stamps
            Car one = Cars[0];
            one.Link = new GameObject("CarLink 1").AddComponent<CarLink>();
            one.Link.Bind(one, sio != null ? sio.IPAddress : "127.0.0.1", basePort.ToString());
            // one car: connect only with -autoconnect, as the stock Socket did (--no-connect); 2+ cars: always
            if (WantConnected == null)
                WantConnected = Environment.GetCommandLineArgs().Any(x => x.ToLower() == "-autoconnect");
            one.Link.Want = Count > 1 || WantConnected.Value;
            one.Probe = one.Root.AddComponent<WheelProbe>();
            one.Probe.Car = one;
            if (Count <= 1)
            {
                PlaceGrid();
                Debug.Log(Tag + $"1 car, CarLink on port {one.Link.Port}");
                return;
            }
            for (int i = 2; i <= Count; i++)
            {
                GameObject copy = UnityEngine.Object.Instantiate(first);
                copy.name = "Roboracer " + i;
                Car car = Describe(copy, i);
                Quiet(car);
                car.Vc.DrivingMode = 1;   // Autonomous: its bridge drives it
                int port = ports != null && ports.Length >= i - 1 ? ports[i - 2] : basePort + i - 1;
                var linkGo = new GameObject("CarLink " + i);
                car.Link = linkGo.AddComponent<CarLink>();
                car.Link.Bind(car, "127.0.0.1", port.ToString());
                Cars.Add(car);
            }
            foreach (Car c in Cars.Skip(1))
            {
                c.Probe = c.Root.AddComponent<WheelProbe>();
                c.Probe.Car = c;
            }
            PlaceGrid();
            ApplyContact();
            Debug.Log(Tag + $"{Count} cars, contact {(Contact ? "on" : "off (ghosts)")}, ports " +
                      string.Join(",", Cars.Select(c => c.Link.Port)));
        }

        // A copy must not render, listen or write the HUD: those are car 1's.
        static void Quiet(Car car)
        {
            foreach (Camera c in car.Root.GetComponentsInChildren<Camera>(true)) c.enabled = false;
            foreach (AudioListener l in car.Root.GetComponentsInChildren<AudioListener>(true)) l.enabled = false;
            if (car.Lap != null)
            {
                var hud = new GameObject("HUD texts " + car.Root.name);
                hud.SetActive(false);
                hud.transform.SetParent(car.Root.transform, false);
                Text T() { var g = new GameObject("t"); g.transform.SetParent(hud.transform, false); return g.AddComponent<Text>(); }
                car.Lap.txtLapTime = T(); car.Lap.txtLastLap = T(); car.Lap.txtBestLap = T();
                car.Lap.txtLapCount = T(); car.Lap.txtCollisionCount = T();
            }
        }

        /// <summary>Cars 2.. to their start poses relative to car 1's (after a track switch too).</summary>
        public static void PlaceGrid()
        {
            if (Cars.Count == 0) return;
            Car one = Cars[0];
            // car 1 is at its start (scene load, track switch): that is its reset pose (resetcar 1 with one
            // car too), and the grid is laid out from it
            one.StartPos = one.Root.transform.position; one.StartRot = one.Root.transform.rotation;
            GridStarts(true);
        }

        /// <summary>Cars 2..'s start (reset) poses back to their grid slots behind car 1's start pose (after a
        /// ghost race stacked them on car 1: ctl grid); teleport: also move them there now.</summary>
        public static void GridStarts(bool teleport)
        {
            if (Cars.Count < 2) return;
            Car one = Cars[0];
            Vector3 fwd = one.StartRot * Vector3.forward, right = one.StartRot * Vector3.right;
            for (int k = 1; k < Cars.Count; k++)
            {
                Car car = Cars[k];
                Vector2 o = grid != null && grid.Length >= k ? grid[k - 1] : new Vector2(1.5f * k, k % 2 == 1 ? -0.4f : 0.4f);
                car.StartPos = one.StartPos - fwd * o.x - right * o.y;
                car.StartRot = one.StartRot;
                if (teleport) Teleport(car, car.StartPos, car.StartRot);
            }
        }

        /// <summary>Cars 2.. back to their start poses with lap state restarted (after a track switch: their
        /// lap timers still held the old track's checkpoints).</summary>
        public static void ResetOthers()
        {
            for (int k = 1; k < Cars.Count; k++) ResetCar(Cars[k]);
        }

        public static void Teleport(Car car, Vector3 pos, Quaternion rot)
        {
            car.Root.transform.SetPositionAndRotation(pos, rot);
            if (car.Body != null)
            {
                car.Body.position = pos; car.Body.rotation = rot;
                if (!car.Body.isKinematic) { car.Body.velocity = Vector3.zero; car.Body.angularVelocity = Vector3.zero; }
            }
        }

        /// <summary>Car back to its start pose with its lap timer and encoders at their start values
        /// (what the sim's ResetManager does for car 1).</summary>
        public static void ResetCar(Car car)
        {
            if (car.Parked) Park(car, false);
            Teleport(car, car.StartPos, car.StartRot);
            car.Vc.CurrentThrottle = 0f; car.Vc.CurrentSteeringAngle = 0f;
            if (car.Lap != null)
            {
                car.Lap.LapCount = 0; car.Lap.LapTime = 0f;
                car.Lap.LastLapTime = float.PositiveInfinity; car.Lap.BestLapTime = float.PositiveInfinity;
                car.Lap.CurrentCheckpoint = 0; car.Lap.PreviousCheckpoint = 0;
                car.Lap.CheckpointCount = 0; car.Lap.CollisionCount = 0;
            }
            if (car.LeftEnc != null) car.LeftEnc.TotalRevolutions = 0f;
            if (car.RightEnc != null) car.RightEnc.TotalRevolutions = 0f;
        }

        /// <summary>Contact on: cars collide. Off: ghosts (seen by the lidars, no physical contact).</summary>
        public static void ApplyContact()
        {
            for (int k = 1; k < Cars.Count; k++) Paint(Cars[k], !Contact);
            // cars the lidars do not see: every car's colliders off the lidar's layer (all cars, so
            // no car sees another; a car's own lidar never needed its own body)
            bool hide = !LidarSeesCars;
            foreach (Car car in Cars)
                foreach (Collider c in car.Root.GetComponentsInChildren<Collider>(true))
                {
                    GameObject g = c.gameObject;
                    if (!layers.ContainsKey(g)) layers[g] = g.layer;
                    g.layer = hide ? IgnoreRaycast : layers[g];
                }
            for (int a = 0; a < Cars.Count; a++)
                for (int b = a + 1; b < Cars.Count; b++)
                    foreach (Collider ca in Cars[a].Root.GetComponentsInChildren<Collider>())
                        foreach (Collider cb in Cars[b].Root.GetComponentsInChildren<Collider>())
                            Physics.IgnoreCollision(ca, cb, !Contact);
        }

        public static string SetContact(bool on)
        {
            Contact = on;
            ApplyContact();
            return on ? "contact on" : "contact off (ghosts)";
        }

        // ---------------------------------------------------------------- look of cars 2..
        // Ghosts: no decals (SIGRobotics, roboracer, AutoDRIVE canvases), translucent blue, every mesh on
        // HDRP/Unlit with a multiply blend (the build has no alpha variant): the track shows through and
        // overlapping parts tint deeper, so the car's detail stays visible. Contact: exactly car 1's look.
        public static Color GhostColor = new Color(0.6f, 0.85f, 1f);   // pale blue: the multiply tints what is behind
        public static float GhostAlpha = 0.3f;   // used only if a build has HDRP's alpha variant
        static Material faintMat, baseMat;   // baseMat: the car's own body material, before any repaint
        static readonly Dictionary<Renderer, Material[]> original = new Dictionary<Renderer, Material[]>();
        static readonly Dictionary<GameObject, bool> decals = new Dictionary<GameObject, bool>();
        static readonly Dictionary<Renderer, UnityEngine.Rendering.ShadowCastingMode> shadows = new Dictionary<Renderer, UnityEngine.Rendering.ShadowCastingMode>();

        static void SetColor(Material m, Color c)
        {
            foreach (string p in new[] { "_BaseColor", "_UnlitColor", "_Color" })
                if (m.HasProperty(p)) m.SetColor(p, c);
            foreach (string p in new[] { "_BaseColorMap", "_UnlitColorMap", "_MainTex", "_NormalMap", "_MaskMap", "_EmissiveColorMap" })
                if (m.HasProperty(p)) m.SetTexture(p, null);
            if (m.HasProperty("_EmissiveColor")) m.SetColor("_EmissiveColor", Color.black);
        }

        static string ghostShader = "unlit";
        // blend of the faint look (UnityEngine.Rendering.BlendMode numbers). The build has no alpha variant
        // of HDRP/Unlit (alpha is ignored), so the default is a multiply (DstColor, Zero): the track shows
        // through the car tinted by its colour, a faint glass car, with no shader alpha needed.
        public static float GhostSrc = 2f, GhostDst = 0f;

        // An opaque HDRP material made transparent (alpha blend, no depth write), as HDRP's material
        // editor does. "unlit": HDRP/Unlit instead of the car's own shader (flat colour, no lighting).
        static Material MakeTransparent(Material m, string kind)
        {
            if (kind == "unlit")
            {
                Shader u = Shader.Find("HDRP/Unlit");
                if (u != null) m.shader = u; else Debug.Log(Tag + "no HDRP/Unlit shader: using the car's own");
            }
            m.SetFloat("_SurfaceType", 1f);
            m.SetFloat("_BlendMode", 0f);
            m.SetFloat("_AlphaCutoffEnable", 0f);
            m.SetFloat("_SrcBlend", GhostSrc); m.SetFloat("_DstBlend", GhostDst);
            m.SetFloat("_AlphaSrcBlend", GhostSrc); m.SetFloat("_AlphaDstBlend", GhostDst);
            m.SetFloat("_ZWrite", 0f); m.SetFloat("_TransparentZWrite", 0f);
            m.SetFloat("_ZTestDepthEqualForOpaque", 4f);                         // LessEqual
            m.SetFloat("_CullMode", 2f); m.SetFloat("_CullModeForward", 2f);
            m.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
            m.EnableKeyword("_BLENDMODE_ALPHA");
            m.DisableKeyword("_ALPHATEST_ON");
            m.SetOverrideTag("RenderType", "Transparent");
            m.renderQueue = 3000;
            m.SetShaderPassEnabled("TransparentDepthPrepass", false);
            m.SetShaderPassEnabled("TransparentDepthPostpass", false);
            m.SetShaderPassEnabled("TransparentBackface", false);
            m.SetShaderPassEnabled("DepthOnly", false);
            m.SetShaderPassEnabled("MotionVectors", false);
            return m;
        }

        /// <summary>ghostlook [alpha] [lit|unlit] [r g b] [src dst]: change the ghost look now (ctl).</summary>
        public static string GhostLook(string[] w)
        {
            var ci = CultureInfo.InvariantCulture;
            if (w.Length > 1) GhostAlpha = float.Parse(w[1], ci);
            if (w.Length > 2) ghostShader = w[2].ToLower();
            if (w.Length > 5) GhostColor = new Color(float.Parse(w[3], ci), float.Parse(w[4], ci), float.Parse(w[5], ci));
            if (w.Length > 7) { GhostSrc = float.Parse(w[6], ci); GhostDst = float.Parse(w[7], ci); }
            faintMat = null;
            for (int k = 1; k < Cars.Count; k++) Paint(Cars[k], !Contact);
            return $"ghost alpha {GhostAlpha}, {ghostShader}, colour {GhostColor}, blend {GhostSrc} {GhostDst}";
        }

        static void Paint(Car car, bool faint)
        {
            // decals (SIGRobotics, roboracer, AutoDRIVE): off on ghosts, back as they were otherwise
            foreach (Canvas c in car.Root.GetComponentsInChildren<Canvas>(true))
            {
                GameObject g = c.gameObject;
                if (!decals.ContainsKey(g)) decals[g] = g.activeSelf;
                g.SetActive(!faint && decals[g]);
            }
            Renderer[] rs = car.Root.GetComponentsInChildren<MeshRenderer>(true);
            if (rs.Length == 0) return;
            foreach (Renderer r in rs)
            {
                if (!original.ContainsKey(r)) original[r] = r.sharedMaterials;
                if (!shadows.ContainsKey(r)) shadows[r] = r.shadowCastingMode;
            }
            if (baseMat == null) baseMat = original[rs[0]][0];
            if (faint && faintMat == null)
            {
                faintMat = MakeTransparent(new Material(baseMat), ghostShader);
                faintMat.name = "Car ghost";
                SetColor(faintMat, new Color(GhostColor.r, GhostColor.g, GhostColor.b, GhostAlpha));
            }
            foreach (Renderer r in rs)
            {
                if (faint && faintMat != null)
                {
                    var mats = new Material[r.sharedMaterials.Length];
                    for (int i = 0; i < mats.Length; i++) mats[i] = faintMat;
                    r.sharedMaterials = mats;
                }
                else r.sharedMaterials = original[r];
                r.shadowCastingMode = faint ? UnityEngine.Rendering.ShadowCastingMode.Off : shadows[r];
            }
        }

        /// <summary>Held: every car gets zero throttle and steering whatever its bridge asks (CarLink); the
        /// request is still recorded, so a script can see when every stack is ready to drive.</summary>
        public static bool Held;

        // Launch after a start: a stack whose speed command wound up while its car was held would get it as
        // one throttle step (wheel spin; the wheel odometry counts it as travel and the localization jumps).
        // For LaunchSeconds after start, each car's throttle is capped at what about LaunchLead m/s above its
        // own speed takes (LaunchThrottlePerMps: the sim car's ~25 m/s per throttle unit). Only at the start.
        public static double LaunchUntil = -1;
        public const double LaunchSeconds = 3.0;
        public const float LaunchLead = 1.0f, LaunchThrottlePerMps = 0.04f;

        /// <summary>Release all cars in the same frame, their lap timers restarted (lap time, count, last
        /// and best; not the collision counts: the bridges read a change there as a sim reset).</summary>
        public static string Start()
        {
            foreach (Car c in Cars) if (c.Parked) Park(c, false);
            foreach (Car c in Cars)
                if (c.Lap != null)
                {
                    c.Lap.LapTime = 0f; c.Lap.LapCount = 0;
                    c.Lap.LastLapTime = float.PositiveInfinity; c.Lap.BestLapTime = float.PositiveInfinity;
                }
            Held = false;
            LaunchUntil = Time.timeAsDouble + LaunchSeconds;
            Debug.Log(Tag + $"start: {Cars.Count} cars released at sim time {Time.timeAsDouble:0.000}");
            return $"released {Cars.Count} cars at sim time {Time.timeAsDouble:0.000}";
        }

        /// <summary>park &lt;car&gt;: stop it where it is and keep it there (race_cars.sh, once a car has done
        /// its laps: its stack is then shut down, and a car left at speed would coast into the next wall).
        /// Undone by start, resetcar and unpark.</summary>
        public static string Park(Car car, bool on)
        {
            car.Parked = on;
            car.Vc.CurrentThrottle = 0f; car.Vc.CurrentSteeringAngle = 0f;
            if (car.Body != null)
            {
                if (on) { car.Body.velocity = Vector3.zero; car.Body.angularVelocity = Vector3.zero; }
                car.Body.isKinematic = on;
            }
            return $"car {car.Index} " + (on ? "parked" : "unparked");
        }

        /// <summary>Ghost mode as one switch (the Menu's Ghost row, ctl ghost): on = no contact, translucent,
        /// unseen by the lidars; off = contact, the cars' own look, seen.</summary>
        public static bool Ghost => !Contact;
        public static string SetGhost(bool on)
        {
            Contact = !on;
            LidarSeesCars = !on;
            ApplyContact();
            return on ? "ghost on: no contact, translucent, not seen by the lidars"
                      : "ghost off: contact, normal look, seen by the lidars";
        }

        public static string SetLidarSeesCars(bool on)
        {
            LidarSeesCars = on;
            ApplyContact();
            return "lidar sees cars: " + (on ? "on" : "off");
        }

        static string N(float v) => v.ToString("0.000", CultureInfo.InvariantCulture);
        static string LapN(float v) => float.IsInfinity(v) || float.IsNaN(v) ? "null" : N(v);

        /// <summary>JSON list of the cars (ctl cars): pose in ROS world (x = Unity z, y = -Unity x).</summary>
        public static string Json()
        {
            var sb = new StringBuilder("{\"ok\":true,\"held\":").Append(Held ? "true" : "false").Append(",\"contact\":").Append(Contact ? "true" : "false")
                .Append(",\"ghost\":").Append(Ghost ? "true" : "false").Append(",\"lidar_sees_cars\":").Append(LidarSeesCars ? "true" : "false").Append(",\"cars\":[");
            for (int i = 0; i < Cars.Count; i++)
            {
                Car c = Cars[i];
                Transform t = c.Root.transform;
                float yaw = -t.eulerAngles.y * Mathf.Deg2Rad;
                // the point the bridge reports as the car's position (its IPS: GPS.VehicleTransform,
                // 0.158 m from the body's origin), now rather than at the IPS's last update
                Vector3 ip = TruthPoint(c);
                float px = ip.z, py = -ip.x;
                Vector3 v = c.Body == null ? Vector3.zero : c.Body.velocity;
                if (i > 0) sb.Append(',');
                sb.Append("{\"car\":").Append(c.Index)
                  .Append(",\"name\":\"").Append(c.Root.name).Append('"')
                  .Append(",\"x\":").Append(N(px)).Append(",\"y\":").Append(N(py))
                  .Append(",\"yaw\":").Append(N(Mathf.Atan2(Mathf.Sin(yaw), Mathf.Cos(yaw))))
                  .Append(",\"speed\":").Append(N(new Vector2(v.x, v.z).magnitude))
                  .Append(",\"mode\":\"").Append(c.Vc.DrivingMode == 1 ? "Autonomous" : "Manual").Append('"')
                  .Append(",\"laps\":").Append(c.Lap == null ? 0 : c.Lap.LapCount)
                  // lap count and lap times in one snapshot (the ROS topics are separate messages, so a reader
                  // could pair one lap's count with another lap's time); sim time, null before the first lap
                  .Append(",\"lap_time\":").Append(c.Lap == null ? "null" : N(c.Lap.LapTime))
                  .Append(",\"last_lap\":").Append(LapN(c.Lap == null ? float.PositiveInfinity : c.Lap.LastLapTime))
                  .Append(",\"best_lap\":").Append(LapN(c.Lap == null ? float.PositiveInfinity : c.Lap.BestLapTime))
                  .Append(",\"collisions\":").Append(c.Lap == null ? 0 : c.Lap.CollisionCount)
                  .Append(",\"parked\":").Append(c.Parked ? "true" : "false");
                if (c.Probe != null)
                    sb.Append(",\"wheel_steps\":").Append(c.Probe.Steps)
                      .Append(",\"wheel_on_car\":").Append(c.Probe.OnCar)
                      .Append(",\"wheel_on_self\":").Append(c.Probe.OnOther)
                      .Append(",\"wheel_on_car_last\":\"").Append(c.Probe.Last.Replace("\"", "'")).Append('"');
                if (c.Link != null) c.Link.AppendStatus(sb);
                else sb.Append(",\"port\":\"stock\"");   // one car: the sim's own Socket
                sb.Append('}');
            }
            return sb.Append("]}").ToString();
        }

        /// <summary>World point of the car's IPS (Unity frame): the position the bridge reports.</summary>
        public static Vector3 TruthPoint(Car c) =>
            c.Gps != null && c.Gps.VehicleTransform != null ? c.Gps.VehicleTransform.position : c.Root.transform.position;

        /// <summary>place &lt;car&gt; &lt;x&gt; &lt;y&gt; [yaw]: ROS world pose; becomes its start pose too.</summary>
        public static string Place(int index, float x, float y, float? yaw)
        {
            Car car = Cars.FirstOrDefault(c => c.Index == index);
            if (car == null) return null;
            Transform root = car.Root.transform;
            Quaternion rot = yaw.HasValue ? Quaternion.Euler(0f, -yaw.Value * Mathf.Rad2Deg, 0f) : root.rotation;
            // x, y is where the IPS point goes (what ctl cars and the bridge report)
            Vector3 local = root.InverseTransformPoint(TruthPoint(car));
            Vector3 pos = new Vector3(-y, root.position.y, x) - rot * new Vector3(local.x, 0f, local.z);
            car.StartPos = pos; car.StartRot = rot;
            Teleport(car, pos, rot);
            return $"car {index} at {x:0.00}, {y:0.00}";
        }
    }

    /// <summary>One car's bridge connection: the sim's Socket.IO client on its own port, the sim's own
    /// frame exchange for that one car, and SimControl's supervision (connect only once a bridge answers
    /// the handshake; drop and stop the car when frames stop).</summary>
    public class CarLink : MonoBehaviour
    {
        const string Tag = "[MultiCar] ";
        Car car;
        GameObject sioGo;
        SocketIOComponent sio;
        public string Ip, Port;
        public float RequestedThrottle;
        public bool Want = true;          // keep connected (ctl connect / disconnect with 2+ cars)
        public const double MaxRate = 100.0;   // frames/s to the bridge
        double lastEmit;   // what its bridge last asked (applied unless MultiCar.Held)
        float lastDriveRun = -100f;       // runTime of the last request to drive (|throttle| > 0.005)
        float lastIdleRun;                // runTime of the last request not to drive (or of connecting)
        public const float ReadyAfter = 2f;   // s of requests to drive without a break: the stack is ready
        bool handlersAdded, sendPending;
        int frames, disconnects, disconnectsAtEnable;
        long lastFrameTicks;
        float runTime, enabledAtRun, lastProbeRun, lastKickRun, lastFrameRun;
        int lastFrames;
        volatile int probe;               // 0 idle, 1 running, 2 bridge answered, 3 no bridge
        float rateStart; int rateFrames0; float rate;

        public void Bind(Car c, string ip, string port)
        {
            car = c; Ip = ip; Port = port;
            sioGo = new GameObject("SocketIO " + c.Index);
            sioGo.SetActive(false);
            sioGo.transform.SetParent(transform, false);
            // the client reads IP/Port input fields in OnEnable (the HUD's for car 1): empty ones here
            InputField Field(string n) { var g = new GameObject(n); g.SetActive(false); g.transform.SetParent(sioGo.transform, false); return g.AddComponent<InputField>(); }
            InputField ipField = Field("IP"), portField = Field("Port");
            sio = sioGo.AddComponent<SocketIOComponent>();
            sio.IP = ipField; sio.Port = portField;
            sio.IPAddress = ip; sio.PortNumber = port;
            sio.autoConnect = true;
        }

        void Update()
        {
            if (car == null || car.Root == null) { Destroy(gameObject); return; }
            runTime += Mathf.Min(Time.unscaledDeltaTime, 0.1f);
            int f = Volatile.Read(ref frames);
            if (f != lastFrames) { lastFrames = f; lastFrameRun = runTime; }
            float now = Time.realtimeSinceStartup;
            if (now - rateStart >= 1f) { rate = (f - rateFrames0) / (now - rateStart); rateStart = now; rateFrames0 = f; }
            try { Supervise(); } catch (Exception e) { Debug.LogError(Tag + $"car {car.Index}: " + e); }
            // at most MaxRate frames/s to the bridge, as the sim's own Socket gives one car (~100/s): faster,
            // every bridge and stack works harder (2 stacks at 150-180/s each: pose stalls > 0.2 s, crashes)
            if (sendPending && sio != null && sio.IsConnected && Time.realtimeSinceStartupAsDouble - lastEmit >= 1.0 / MaxRate)
            {
                sendPending = false;
                lastEmit = Time.realtimeSinceStartupAsDouble;
                long t0 = System.Diagnostics.Stopwatch.GetTimestamp();
                Emit();
                Perf.EmitTicks += System.Diagnostics.Stopwatch.GetTimestamp() - t0;
            }
        }

        /// <summary>Drop the connection now; it comes back by itself while Want (ctl reconnect).</summary>
        public void Reconnect(string reason) { if (sioGo.activeSelf) Drop(reason); }
        public void Kick() { sendPending = true; }

        void Supervise()
        {
            if (!Want) { if (sioGo.activeSelf) Drop("disconnect command"); return; }
            if (sioGo.activeSelf)
            {
                bool replied = lastFrameRun > enabledAtRun;
                float since = runTime - enabledAtRun;
                if (!replied && since > 1.5f && runTime - lastKickRun > 2f) { lastKickRun = runTime; sendPending = true; }
                if (!replied && since > 5f) Drop("no reply from the bridge 5 s after connecting");
                else if (!replied && Volatile.Read(ref disconnects) - disconnectsAtEnable > 3) Drop("the connection keeps dropping");
                else if (replied && Volatile.Read(ref disconnects) != disconnectsAtEnable) Drop("the bridge closed the connection");
                else if (replied && runTime - lastFrameRun > 1f) Drop("no frames for 1 s");
            }
            else if (probe == 2)
            {
                probe = 0;
                sioGo.SetActive(true);   // Awake (handler table) on the first time, then OnEnable connects
                if (!handlersAdded)
                {
                    sio.On("connect", e => sendPending = true);
                    sio.On("disconnect", e => Interlocked.Increment(ref disconnects));
                    sio.On("Bridge", OnBridge);
                    handlersAdded = true;
                }
                enabledAtRun = runTime;
                lastIdleRun = runTime;
                disconnectsAtEnable = Volatile.Read(ref disconnects);
                Debug.Log(Tag + $"car {car.Index}: bridge answers at {Ip}:{Port}: connecting");
            }
            else if (probe != 1 && runTime - lastProbeRun > 1f)
            {
                lastProbeRun = runTime;
                StartProbe();
            }
        }

        void Drop(string reason)
        {
            sioGo.SetActive(false);
            car.Vc.CurrentThrottle = 0f; car.Vc.CurrentSteeringAngle = 0f;
            lastProbeRun = runTime;
            Debug.Log(Tag + $"car {car.Index}: disconnected: {reason}");
        }

        void StartProbe()
        {
            probe = 1;
            string ip = Ip, port = Port;
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
                            var stream = client.GetStream();
                            stream.ReadTimeout = 1000;
                            byte[] req = Encoding.ASCII.GetBytes($"GET /socket.io/?EIO=3&transport=polling HTTP/1.1\r\nHost: {ip}:{port}\r\nConnection: close\r\n\r\n");
                            stream.Write(req, 0, req.Length);
                            var buf = new byte[4096];
                            int n, got = 0;
                            while (got < buf.Length && (n = stream.Read(buf, got, buf.Length - got)) > 0) got += n;
                            string resp = Encoding.ASCII.GetString(buf, 0, got);
                            ok = resp.StartsWith("HTTP/1.1 200") && resp.Contains("sid");
                        }
                    }
                }
                catch (Exception) { }
                probe = ok ? 2 : 3;
            });
        }

        // the sim's Socket.OnBridge for one car (runs on the main thread: SocketIOComponent.FixedUpdate)
        void OnBridge(SocketIOEvent e)
        {
            Interlocked.Increment(ref frames);
            Interlocked.Exchange(ref lastFrameTicks, System.Diagnostics.Stopwatch.GetTimestamp());
            try
            {
                JSONObject d = e.data;
                if (car.Vc.DrivingMode == 1 && d != null)
                {
                    JSONObject reset = d.GetField("V1 Reset");
                    if (reset != null && bool.TryParse(reset.str, out bool r) && r) MultiCar.ResetCar(car);
                    JSONObject th = d.GetField("V1 Throttle"), st = d.GetField("V1 Steering");
                    float t = th != null ? float.Parse(th.str, CultureInfo.InvariantCulture) : 0f;
                    float a = st != null ? float.Parse(st.str, CultureInfo.InvariantCulture) : 0f;
                    RequestedThrottle = t;
                    if (Mathf.Abs(t) > 0.005f) lastDriveRun = runTime; else lastIdleRun = runTime;
                    if (!MultiCar.Held && Time.timeAsDouble < MultiCar.LaunchUntil && car.Body != null)
                        t = Mathf.Min(t, MultiCar.LaunchThrottlePerMps * (car.Body.velocity.magnitude + MultiCar.LaunchLead));
                    bool off = MultiCar.Held || car.Parked;
                    car.Vc.CurrentThrottle = off ? 0f : t;
                    car.Vc.CurrentSteeringAngle = off ? 0f : a;
                }
            }
            catch (Exception ex) { Debug.LogError(Tag + $"car {car.Index}: bad frame: {ex.Message}"); }
            sendPending = true;
        }

        static string V3(float[] a) => a == null ? "0 0 0" : string.Join(" ", a.Select(x => x.ToString("F4", CultureInfo.InvariantCulture)));

        // the sim's Socket.EmitTelemetry for one car, keys "V1 ...", plus its time stamp. Numbers always with a
        // "." (InvariantCulture): the bridge parses them so, whatever the Mac's locale
        void Emit()
        {
            var d = new Dictionary<string, string>();
            d["V1 Throttle"] = car.Vc.CurrentThrottle.ToString("F4", CultureInfo.InvariantCulture);
            d["V1 Steering"] = car.Vc.CurrentSteeringAngle.ToString("F4", CultureInfo.InvariantCulture);
            if (car.LeftEnc != null && car.RightEnc != null)
            {
                d["V1 Encoder Ticks"] = car.LeftEnc.Ticks.ToString(CultureInfo.InvariantCulture) + " " + car.RightEnc.Ticks.ToString(CultureInfo.InvariantCulture);
                d["V1 Encoder Angles"] = car.LeftEnc.Angle.ToString("F4", CultureInfo.InvariantCulture) + " " + car.RightEnc.Angle.ToString("F4", CultureInfo.InvariantCulture);
            }
            if (car.Gps != null) d["V1 Position"] = V3(car.Gps.CurrentPosition);
            if (car.Imu != null)
            {
                d["V1 Orientation Quaternion"] = V3(car.Imu.CurrentOrientationQuaternion);
                d["V1 Orientation Euler Angles"] = V3(car.Imu.CurrentOrientationEulerAngles);
                d["V1 Linear Velocity"] = V3(car.Imu.CurrentLinearVelocity);
                d["V1 Angular Velocity"] = V3(car.Imu.CurrentAngularVelocity);
                d["V1 Linear Acceleration"] = V3(car.Imu.CurrentLinearAcceleration);
            }
            if (car.Lidar != null)
            {
                d["V1 LIDAR Scan Rate"] = car.Lidar.CurrentScanRate.ToString("F4", CultureInfo.InvariantCulture);
                string[] ranges = car.Lidar.CurrentRangeArray;
                if (ranges != null && ranges.Length > 0 && ranges[ranges.Length - 1] != null)
                {
                    d["V1 LIDAR Range Array"] = DataCompressor.CompressArray(ranges);
                    d["V1 LIDAR Intensity Array"] = DataCompressor.CompressArray(car.Lidar.CurrentIntensityArray);
                }
            }
            if (car.Lap != null)
            {
                d["V1 Lap Count"] = car.Lap.LapCount.ToString(CultureInfo.InvariantCulture);
                d["V1 Lap Time"] = car.Lap.LapTime.ToString("F4", CultureInfo.InvariantCulture);
                d["V1 Last Lap Time"] = car.Lap.LastLapTime.ToString("F4", CultureInfo.InvariantCulture);
                d["V1 Best Lap Time"] = car.Lap.BestLapTime.ToString("F4", CultureInfo.InvariantCulture);
                d["V1 Collisions"] = car.Lap.CollisionCount.ToString(CultureInfo.InvariantCulture);
            }
            d["V1 Sim Time"] = Time.timeAsDouble.ToString("F4", CultureInfo.InvariantCulture);
            d["V1 Frame"] = Time.frameCount.ToString(CultureInfo.InvariantCulture);
            d["V1 Car"] = car.Index.ToString(CultureInfo.InvariantCulture);
            sio.Emit("Bridge", new JSONObject(d));
        }

        public bool Connected => sio != null && sio.IsConnected;

        public float FramesPerS => rate;
        /// <summary>Seconds since the last frame from the bridge, -1 before the first.</summary>
        public double LastFrameAge => lastFrameTicks == 0 ? -1 : (System.Diagnostics.Stopwatch.GetTimestamp() - Interlocked.Read(ref lastFrameTicks)) / (double)System.Diagnostics.Stopwatch.Frequency;

        public void AppendStatus(StringBuilder sb)
        {
            double age = LastFrameAge;
            sb.Append(",\"port\":\"").Append(Port).Append('"')
              .Append(",\"connected\":").Append(sio != null && sio.IsConnected ? "true" : "false")
              .Append(",\"throttle_req\":").Append(RequestedThrottle.ToString("0.000", CultureInfo.InvariantCulture))
              // asked to drive in every frame for ReadyAfter s (a stack still settling flips between drive and stop)
              .Append(",\"wants_to_drive\":").Append(runTime - lastDriveRun < 0.5f && runTime - lastIdleRun >= ReadyAfter ? "true" : "false")
              .Append(",\"frames_per_s\":").Append(rate.ToString("0", CultureInfo.InvariantCulture))
              .Append(",\"last_frame_s\":").Append(age < 0 ? "null" : age.ToString("0.00", CultureInfo.InvariantCulture));
        }
    }
}
