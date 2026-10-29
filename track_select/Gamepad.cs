// Gamepad driving in the sim's Manual mode, with driving-game handling.
//
// Unity's input on macOS sees no XInput pad (Logitech F710/F310 in X mode, Xbox 360 pads), so this reads
// them over USB with libusb (brew install libusb; without it, or with no pad, nothing changes). The pad is
// found and opened in the background, also after it is plugged in later or replugged.
//
// Only in Manual mode and only while a pad is open: VehicleController is disabled and its manual step
// (Steer, Drive, UpdateWheelPoses) runs from here with this file's throttle and steering. Autonomous mode
// (the bridge) and Manual mode without a pad are the stock sim (keyboard: W/S throttle, A/D steering).
// The keyboard still works with a pad open, through the same handling.
//
//   RT throttle        LT brake, then reverse once stopped       left stick steering
//   B  handbrake       Start  car back to the start pose (this track)
//   LB/RB  power level down/up     Y  steering assist on/off     X  traction control on/off
//
// Handling: stick deadzone and response curve; steering eased in and out; steering assist scales full
// stick down with speed to about the grip limit; the throttle is proportional to how far RT is pulled
// (times the power level); traction control (off by default) ramps the throttle and caps it at low
// speed (the sim throttle is wheel torque, a full step spins the wheels); off the pedals the car coasts
// with light engine braking instead of the stock zero-throttle full brake, and holds once stopped.
// Log lines start with [Gamepad].
using System;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Threading;
using UnityEngine;

namespace TrackSelect
{
    public class Gamepad : MonoBehaviour
    {
        const string Tag = "[Gamepad] ";
        static Gamepad instance;

        // ---- handling (stick and trigger units are 0-1, throttle is the sim's -1..1 torque command)
        const float StickDeadzone = 0.10f;
        const float TriggerDeadzone = 0.04f;
        const float SteerExpo = 1.4f;            // |x|^expo after the deadzone: finer control near center
        const float SteerInRate = 4.0f;          // full lock per s turning in (0 -> 1 in 0.25 s)
        const float SteerOutRate = 7.0f;         // per s back toward center / the other way
        const float AssistLatAccel = 12.0f;      // m/s^2: steering assist's grip estimate
        const float AssistMargin = 1.3f;         // full stick = this times the grip-limit angle
        const float AssistMinLock = 0.12f;       // assist never scales full stick below this
        const float Wheelbase = 0.33f;           // m (control/config/vehicle.yaml)
        const float ThrottleExpo = 1.0f;         // linear: throttle = RT pulled x power level
        const float ThrottleRiseRate = 3.0f;     // per s, with traction control (release is immediate)
        const float TcBase = 0.20f;              // traction control cap: TcBase + TcPerMps * speed
        const float TcPerMps = 0.05f;            // (steady state is ~0.04 throttle per m/s)
        const float BrakeGain = 1.0f;            // LT at full while rolling forward
        const float ReverseLimit = 0.25f;        // LT at full once stopped (~6 m/s)
        const float EngineBrake = 0.03f;         // off the pedals while rolling
        const float StoppedSpeed = 0.3f;         // m/s: below this LT reverses and no input holds
        static readonly float[] PowerLevels = { 0.25f, 0.4f, 0.55f, 0.7f, 0.85f, 1.0f };
        const int DefaultPower = 2;

        // ---- XInput over libusb: (vendor, product) of pads that show up as Xbox 360 controllers
        static readonly (ushort vid, ushort pid, string name)[] Pads = {
            (0x046d, 0xc21f, "Logitech F710"),   // X mode
            (0x046d, 0xc21d, "Logitech F310"),   // X mode
            (0x046d, 0xc21e, "Logitech F510"),   // X mode
            (0x045e, 0x028e, "Xbox 360 pad"),
            (0x045e, 0x0719, "Xbox 360 wireless receiver"),
        };
        const byte EpIn = 0x81, EpOut = 0x02;

        struct PadState
        {
            public float lt, rt, lx;
            public ushort buttons;
        }
        // XInput button bits (report bytes 2-3)
        const ushort BtnStart = 0x0010, BtnLB = 0x0100, BtnRB = 0x0200, BtnB = 0x2000, BtnX = 0x4000, BtnY = 0x8000;

        readonly object stateLock = new object();
        PadState padState;
        volatile string padName;            // null while no pad is open
        volatile bool stopping;
        Thread reader;

        // ---- driving state (main thread)
        VehicleController vc;
        Action steer, drive, updateWheelPoses;
        FieldInfo throttleInput, steeringInput, driveTorque, currentT, currentS;
        bool driving;                       // VehicleController disabled, we run its manual step
        float steerOut, throttleOut;
        int power = DefaultPower;
        bool assist = true, tc = false;
        ushort lastButtons;
        string toast; float toastUntil;
        GUIStyle toastStyle;

        public static void Init()
        {
            var go = new GameObject("Gamepad");
            DontDestroyOnLoad(go);
            instance = go.AddComponent<Gamepad>();
        }

        public static string Status() => instance == null ? null : instance.padName;

        void Start()
        {
            if (!Usb.Load()) { Debug.Log(Tag + "no libusb (brew install libusb): gamepad off, keyboard only"); return; }
            reader = new Thread(ReadLoop) { IsBackground = true, Name = "Gamepad" };
            reader.Start();
        }

        void OnApplicationQuit() { stopping = true; reader?.Join(500); }

        // ------------------------------------------------------------ USB reader thread

        void ReadLoop()
        {
            IntPtr ctx = IntPtr.Zero;
            if (Usb.init(ref ctx) != 0) { Debug.LogError(Tag + "libusb_init failed"); return; }
            var buf = new byte[32];
            while (!stopping)
            {
                IntPtr h = IntPtr.Zero; string name = null;
                foreach (var pad in Pads)
                {
                    h = Usb.open_device_with_vid_pid(ctx, pad.vid, pad.pid);
                    if (h != IntPtr.Zero && Usb.claim_interface(h, 0) != 0) { Usb.close(h); h = IntPtr.Zero; }
                    if (h != IntPtr.Zero) { name = pad.name; break; }
                }
                if (h == IntPtr.Zero) { Thread.Sleep(1000); continue; }
                lock (stateLock) padState = default;
                Write(h, new byte[] { 0x01, 0x03, 0x06 });                       // LED: player 1
                Rumble(h, 0x60, 0x60); Thread.Sleep(150); Rumble(h, 0, 0);       // "connected" buzz
                padName = name;
                Debug.Log(Tag + name + " connected");
                while (!stopping)
                {
                    int r = Usb.interrupt_transfer(h, EpIn, buf, buf.Length, out int n, 100);
                    if (r == Usb.ErrorTimeout) continue;
                    if (r != 0) break;                                              // unplugged
                    if (n >= 14 && buf[0] == 0x00) lock (stateLock) padState = Parse(buf);
                }
                padName = null;
                lock (stateLock) padState = default;
                if (!stopping) Debug.Log(Tag + name + " disconnected");
                Usb.release_interface(h, 0);
                Usb.close(h);
            }
            Usb.exit(ctx);
        }

        static PadState Parse(byte[] b) => new PadState
        {
            buttons = (ushort)(b[2] | b[3] << 8),
            lt = b[4] / 255f,
            rt = b[5] / 255f,
            lx = Math.Max(-1f, BitConverter.ToInt16(b, 6) / 32767f),
        };

        static void Write(IntPtr h, byte[] data) => Usb.interrupt_transfer(h, EpOut, data, data.Length, out _, 100);
        static void Rumble(IntPtr h, byte big, byte small) => Write(h, new byte[] { 0x00, 0x08, 0x00, big, small, 0x00, 0x00, 0x00 });

        // ------------------------------------------------------------ driving (main thread)

        void FixedUpdate()
        {
            bool want = padName != null && Bind() && vc.DrivingMode == 0 && vc.driveType != VehicleController.DriveType.SkidSteer;
            if (want != driving) SetDriving(want);
            if (!driving) return;
            try { Step(Time.fixedDeltaTime); }
            catch (Exception e) { Debug.LogError(Tag + e); SetDriving(false); padName = null; }
        }

        // finds the car (again after a scene load) and its private manual-step members
        bool Bind()
        {
            if (vc != null) return true;
            vc = FindObjectOfType<VehicleController>();
            if (vc == null) return false;
            const BindingFlags F = BindingFlags.NonPublic | BindingFlags.Instance;
            Type t = typeof(VehicleController);
            throttleInput = t.GetField("ThrottleInput", F); steeringInput = t.GetField("SteeringInput", F);
            driveTorque = t.GetField("DriveTorque", F); currentT = t.GetField("currentT", F); currentS = t.GetField("currentS", F);
            steer = (Action)Delegate.CreateDelegate(typeof(Action), vc, t.GetMethod("Steer", F));
            drive = (Action)Delegate.CreateDelegate(typeof(Action), vc, t.GetMethod("Drive", F));
            updateWheelPoses = (Action)Delegate.CreateDelegate(typeof(Action), vc, t.GetMethod("UpdateWheelPoses", F));
            driving = false;   // a new car is enabled as built
            Debug.Log(Tag + $"car: drive {vc.driveType}, brake {vc.brakeType}, throttle limit {vc.ThrottleLimit}, motor torque {vc.MotorTorque}");
            return true;
        }

        void SetDriving(bool on)
        {
            driving = on;
            if (vc != null) vc.enabled = !on;
            steerOut = throttleOut = 0f;
            if (on) Toast($"{padName}: driving   power {PowerLevels[power]:P0}");
        }

        void Step(float dt)
        {
            PadState p;
            lock (stateLock) p = padState;
            Buttons(p.buttons);

            // the keyboard counts too, as if it were the pad (whichever is pushed further)
            float kv = Input.GetAxis("Vertical"), kh = Input.GetAxis("Horizontal");
            float steerIn = Curve(p.lx, StickDeadzone, SteerExpo);
            if (Mathf.Abs(kh) > Mathf.Abs(steerIn)) steerIn = kh;
            float rt = Mathf.Max(Curve(p.rt, TriggerDeadzone, ThrottleExpo), Mathf.Max(0f, kv));
            float lt = Mathf.Max(Curve(p.lt, TriggerDeadzone, 1f), Mathf.Max(0f, -kv));

            Rigidbody rb = vc.VehicleRigidBody;
            float v = Vector3.Dot(rb.velocity, rb.transform.forward);   // m/s, + forward
            float speed = Mathf.Abs(v);

            // steering: assist scales full stick to the grip limit at this speed, then ease toward it
            float target = steerIn;
            if (assist && speed > 0.5f)
            {
                float grip = Mathf.Atan(AssistLatAccel * Wheelbase / (speed * speed)) * AssistMargin;
                target *= Mathf.Clamp(grip / (vc.SteeringLimit * Mathf.Deg2Rad), AssistMinLock, 1f);
            }
            bool turningIn = Mathf.Abs(target) > Mathf.Abs(steerOut) && Mathf.Sign(target) == Mathf.Sign(steerOut) || steerOut == 0f;
            steerOut = Mathf.MoveTowards(steerOut, target, (turningIn ? SteerInRate : SteerOutRate) * dt);

            // throttle: RT drives; LT brakes while rolling forward and reverses once stopped
            float scale = PowerLevels[power];
            float cmd;
            bool reversing = false;
            if ((p.buttons & BtnB) != 0) cmd = 0f;                                   // stock zero = all-wheel brake
            else if (lt > rt && v > StoppedSpeed) cmd = -lt * BrakeGain;
            else if (lt > rt) { cmd = -lt * ReverseLimit; reversing = true; }
            else if (rt > 0f) cmd = v < -StoppedSpeed ? rt * BrakeGain : rt * scale; // RT while rolling back brakes
            else cmd = speed > StoppedSpeed ? -Mathf.Sign(v) * EngineBrake : 0f;     // coast, hold once stopped
            if (tc && cmd > 0f && v >= -StoppedSpeed)
            {
                cmd = Mathf.Min(cmd, TcBase + TcPerMps * speed);
                if (cmd > throttleOut) cmd = Mathf.Min(cmd, Mathf.Max(throttleOut, 0f) + ThrottleRiseRate * dt);
            }
            throttleOut = cmd;

            // VehicleController's own manual step (FixedUpdate with DrivingMode 0), with our inputs
            rb.isKinematic = false;
            throttleInput.SetValue(vc, throttleOut);
            steeringInput.SetValue(vc, -steerOut);   // the sim's SteeringInput is -Horizontal
            currentT.SetValue(vc, (float)driveTorque.GetValue(vc) / vc.MotorTorque);
            currentS.SetValue(vc, steerOut * vc.SteeringLimit * Mathf.Deg2Rad);   // -SteeringAngle in rad
            steer();
            // With brake type IAWB / IRWB, Drive turns a negative throttle into brake torque only, so the
            // car could never back up: reverse with CAWB, where it is negative motor torque.
            VehicleController.BrakeType brake = vc.brakeType;
            if (reversing) vc.brakeType = VehicleController.BrakeType.CAWB;
            try { drive(); }
            finally { vc.brakeType = brake; }
            updateWheelPoses();
        }

        // deadzone, rescale to 0-1 past it, then |x|^expo; keeps the sign
        static float Curve(float x, float deadzone, float expo)
        {
            float a = Mathf.Abs(x);
            if (a <= deadzone) return 0f;
            return Mathf.Sign(x) * Mathf.Pow(Mathf.Min(1f, (a - deadzone) / (1f - deadzone)), expo);
        }

        void Buttons(ushort b)
        {
            ushort pressed = (ushort)(b & ~lastButtons);
            lastButtons = b;
            if ((pressed & BtnRB) != 0 && power < PowerLevels.Length - 1) power++;
            if ((pressed & BtnLB) != 0 && power > 0) power--;
            if ((pressed & (BtnRB | BtnLB)) != 0) Toast($"power {PowerLevels[power]:P0}");
            if ((pressed & BtnY) != 0) { assist = !assist; Toast("steering assist " + (assist ? "on" : "off")); }
            if ((pressed & BtnX) != 0) { tc = !tc; Toast("traction control " + (tc ? "on" : "off")); }
            if ((pressed & BtnStart) != 0 && TrackSelector.Current != null)
            {
                TrackSelector.Switch(TrackSelector.Current);
                steerOut = throttleOut = 0f;
                Toast("reset to start");
            }
        }

        void Toast(string s)
        {
            toast = s; toastUntil = Time.realtimeSinceStartup + 2.5f;
            Debug.Log(Tag + s);
        }

        void OnGUI()
        {
            if (toast == null || Time.realtimeSinceStartup > toastUntil) return;
            if (toastStyle == null)
                toastStyle = new GUIStyle(GUI.skin.box) { fontSize = 18, alignment = TextAnchor.MiddleCenter };
            GUI.Box(new Rect(Screen.width / 2f - 220, Screen.height - 70, 440, 36), toast, toastStyle);
        }

        // ------------------------------------------------------------ libusb, loaded by path (Homebrew)

        static class Usb
        {
            public const int ErrorTimeout = -7;
            [DllImport("libSystem.dylib")] static extern IntPtr dlopen(string path, int mode);
            [DllImport("libSystem.dylib")] static extern IntPtr dlsym(IntPtr handle, string symbol);

            [UnmanagedFunctionPointer(CallingConvention.Cdecl)] public delegate int InitFn(ref IntPtr ctx);
            [UnmanagedFunctionPointer(CallingConvention.Cdecl)] public delegate void ExitFn(IntPtr ctx);
            [UnmanagedFunctionPointer(CallingConvention.Cdecl)] public delegate IntPtr OpenFn(IntPtr ctx, ushort vid, ushort pid);
            [UnmanagedFunctionPointer(CallingConvention.Cdecl)] public delegate void CloseFn(IntPtr h);
            [UnmanagedFunctionPointer(CallingConvention.Cdecl)] public delegate int InterfaceFn(IntPtr h, int iface);
            [UnmanagedFunctionPointer(CallingConvention.Cdecl)] public delegate int TransferFn(IntPtr h, byte ep, byte[] data, int length, out int transferred, uint timeoutMs);

            public static InitFn init; public static ExitFn exit; public static OpenFn open_device_with_vid_pid;
            public static CloseFn close; public static InterfaceFn claim_interface, release_interface;
            public static TransferFn interrupt_transfer;

            public static bool Load()
            {
                IntPtr lib = IntPtr.Zero;
                foreach (string path in new[] { "/opt/homebrew/lib/libusb-1.0.0.dylib", "/usr/local/lib/libusb-1.0.0.dylib" })
                    if ((lib = dlopen(path, 2)) != IntPtr.Zero) break;   // 2 = RTLD_NOW
                if (lib == IntPtr.Zero) return false;
                T F<T>(string name) where T : Delegate => (T)Marshal.GetDelegateForFunctionPointer(dlsym(lib, "libusb_" + name), typeof(T));
                init = F<InitFn>("init"); exit = F<ExitFn>("exit");
                open_device_with_vid_pid = F<OpenFn>("open_device_with_vid_pid"); close = F<CloseFn>("close");
                claim_interface = F<InterfaceFn>("claim_interface"); release_interface = F<InterfaceFn>("release_interface");
                interrupt_transfer = F<TransferFn>("interrupt_transfer");
                return true;
            }
        }
    }
}
