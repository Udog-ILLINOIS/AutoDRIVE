// Track selection for the AutoDRIVE RoboRacer sim (2026-iros build), by launch argument:
//   AutoDRIVE Simulator -track <name>      e.g. porto, berlin, srl2024_iros, iros2026
// Built-in tracks are the "<X> Track" children of Infrastructure, each with "<X> Checkpoints",
// "<X> Spawn Points" and "<X> Camera Target". Custom tracks are loaded from
// StreamingAssets/tracks/<name>/{track.json, mesh.bin} (written by build_track.py) by cloning
// Porto's objects, so materials, MeshCollider, trigger tags and layers carry over.
// The sim's scripts are pointed at the chosen track: LapTimer.RacetrackName (collision name) and
// .Checkpoints (lap logic and respawn), the car's start pose (ResetManager records it in Start,
// which runs after this), and camera followers aimed at a "Camera Target".
// No -track argument leaves the scene untouched (Porto). Re-applied when the scene reloads
// (the in-sim Reset button reloads it). Log lines start with [TrackSelect] in Player.log.
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace TrackSelect
{
    [Serializable] public class Marker { public string name; public float[] pos; public float yaw; public float width; }
    [Serializable] public class TrackFile
    {
        public string name;            // display name, e.g. "IROS 2026"; objects become "<name> Track" etc.
        public Marker[] checkpoints;   // keyed by Porto checkpoint child names ("0".."20", finish lines)
        public Marker[] spawns;        // "Spawn 1", "Spawn 2"
        public float[] camera;         // camera target position
    }

    public static class TrackSelector
    {
        const string Tag = "[TrackSelect] ";
        static string requested;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        public static void Init()
        {
            string[] args = Environment.GetCommandLineArgs();
            for (int i = 0; i < args.Length - 1; i++)
                if (args[i].ToLower() == "-track") requested = args[i + 1];
            if (requested == null) return;
            SceneManager.sceneLoaded += (scene, mode) => Apply();
        }

        static string Key(string s) => new string(s.ToLower().Where(char.IsLetterOrDigit).ToArray());

        static Transform Child(Transform parent, string name)
        {
            foreach (Transform t in parent) if (t.name == name) return t;
            return null;
        }

        static void Apply()
        {
            try { ApplyUnsafe(); }
            catch (Exception e) { Debug.LogError(Tag + e); }
        }

        static void ApplyUnsafe()
        {
            GameObject infraGo = SceneManager.GetActiveScene().GetRootGameObjects().FirstOrDefault(g => g.name == "Infrastructure");
            if (infraGo == null) { Debug.LogError(Tag + "no Infrastructure object"); return; }
            Transform infra = infraGo.transform;

            LoadCustomTracks(infra);
            // prefix ("Porto", "SRL 2024 IROS", "IROS 2026", ...) of every "<prefix> Track"
            var prefixes = new List<string>();
            foreach (Transform t in infra)
                if (t.name.EndsWith(" Track")) prefixes.Add(t.name.Substring(0, t.name.Length - " Track".Length));
            string chosen = prefixes.FirstOrDefault(p => Key(p) == Key(requested));
            if (chosen == null)
            {
                Debug.LogError(Tag + $"unknown track '{requested}'. Available: " +
                               string.Join(", ", prefixes.Select(p => Key(p))));
                return;
            }
            foreach (string p in prefixes)
                foreach (string suffix in new[] { " Track", " Checkpoints", " Spawn Points", " Camera Target" })
                {
                    Transform t = Child(infra, p + suffix);
                    if (t != null) t.gameObject.SetActive(p == chosen);
                }

            Transform cps = Child(infra, chosen + " Checkpoints");
            Transform[] numbered = cps.Cast<Transform>().Where(t => int.TryParse(t.name, out _))
                                      .OrderBy(t => int.Parse(t.name)).ToArray();
            Transform spawn = Child(Child(infra, chosen + " Spawn Points"), "Spawn 1");
            foreach (LapTimer lt in UnityEngine.Object.FindObjectsOfType<LapTimer>(true))
            {
                lt.RacetrackName = chosen + " Track";
                lt.Checkpoints = numbered;
                lt.transform.SetPositionAndRotation(spawn.position, spawn.rotation);
                Rigidbody rb = lt.GetComponent<Rigidbody>();
                if (rb != null) { rb.position = spawn.position; rb.rotation = spawn.rotation; }
            }
            Transform camTarget = Child(infra, chosen + " Camera Target");
            if (camTarget != null)   // e.g. Trackcam's TrackTarget
                foreach (var f in UnityEngine.Object.FindObjectsOfType<AbstractTargetFollower>(true))
                    if (f.Target != null && f.Target.name.EndsWith(" Camera Target")) f.SetTarget(camTarget);
            Debug.Log(Tag + $"track '{chosen}': {numbered.Length} checkpoints, spawn {spawn.position}");
        }

        // ------------------------------------------------------------ custom tracks

        static void LoadCustomTracks(Transform infra)
        {
            string root = Path.Combine(Application.streamingAssetsPath, "tracks");
            if (!Directory.Exists(root)) return;
            Transform portoTrack = Child(infra, "Porto Track");
            Transform portoCps = Child(infra, "Porto Checkpoints");
            Transform portoSpawns = Child(infra, "Porto Spawn Points");
            Transform portoCam = Child(infra, "Porto Camera Target");
            foreach (string dir in Directory.GetDirectories(root))
            {
                string jsonPath = Path.Combine(dir, "track.json"), meshPath = Path.Combine(dir, "mesh.bin");
                if (!File.Exists(jsonPath) || !File.Exists(meshPath)) continue;
                TrackFile tf = JsonUtility.FromJson<TrackFile>(File.ReadAllText(jsonPath));
                if (Child(infra, tf.name + " Track") != null) continue;   // already built in this scene

                GameObject track = UnityEngine.Object.Instantiate(portoTrack.gameObject, infra);
                track.name = tf.name + " Track";
                track.transform.localPosition = portoTrack.localPosition;
                track.transform.localRotation = portoTrack.localRotation;
                Mesh mesh = ReadMesh(meshPath, tf.name);
                track.GetComponent<MeshFilter>().sharedMesh = mesh;
                MeshCollider mc = track.GetComponent<MeshCollider>();
                if (mc != null) { mc.sharedMesh = null; mc.sharedMesh = mesh; }
                track.SetActive(false);

                GameObject cps = UnityEngine.Object.Instantiate(portoCps.gameObject, infra);
                cps.name = tf.name + " Checkpoints";
                Place(cps.transform, tf.checkpoints, true);
                cps.SetActive(false);
                GameObject spawns = UnityEngine.Object.Instantiate(portoSpawns.gameObject, infra);
                spawns.name = tf.name + " Spawn Points";
                Place(spawns.transform, tf.spawns, false);
                spawns.SetActive(false);
                GameObject cam = UnityEngine.Object.Instantiate(portoCam.gameObject, infra);
                cam.name = tf.name + " Camera Target";
                cam.transform.localPosition = new Vector3(tf.camera[0], tf.camera[1], tf.camera[2]);
                cam.SetActive(false);
                Debug.Log(Tag + $"loaded custom track '{tf.name}' from {dir}: {mesh.vertexCount} vertices");
            }
        }

        static void Place(Transform parent, Marker[] markers, bool scaleWidth)
        {
            foreach (Marker m in markers)
            {
                Transform t = Child(parent, m.name);
                if (t == null) { Debug.LogWarning(Tag + $"no marker '{m.name}' under {parent.name}"); continue; }
                t.localPosition = new Vector3(m.pos[0], m.pos[1], m.pos[2]);
                t.localRotation = Quaternion.Euler(0f, m.yaw, 0f);
                if (scaleWidth) t.localScale = new Vector3(m.width, t.localScale.y, t.localScale.z);
            }
        }

        // mesh.bin: int32 nv, int32 ni; float32 pos[nv*3], nrm[nv*3], tan[nv*4], uv[nv*2];
        // int32 sub0[ni], sub1[ni] (outward faces, inward faces; Porto's two material slots)
        static Mesh ReadMesh(string path, string name)
        {
            using (var r = new BinaryReader(File.OpenRead(path)))
            {
                int nv = r.ReadInt32(), ni = r.ReadInt32();
                Vector3[] V3() { var a = new Vector3[nv]; for (int i = 0; i < nv; i++) a[i] = new Vector3(r.ReadSingle(), r.ReadSingle(), r.ReadSingle()); return a; }
                Vector3[] pos = V3(), nrm = V3();
                var tan = new Vector4[nv];
                for (int i = 0; i < nv; i++) tan[i] = new Vector4(r.ReadSingle(), r.ReadSingle(), r.ReadSingle(), r.ReadSingle());
                var uv = new Vector2[nv];
                for (int i = 0; i < nv; i++) uv[i] = new Vector2(r.ReadSingle(), r.ReadSingle());
                int[] I() { var a = new int[ni]; for (int i = 0; i < ni; i++) a[i] = r.ReadInt32(); return a; }
                int[] sub0 = I(), sub1 = I();
                var mesh = new Mesh { name = "Mesh " + name + " Track", indexFormat = UnityEngine.Rendering.IndexFormat.UInt32 };
                mesh.vertices = pos; mesh.normals = nrm; mesh.tangents = tan; mesh.uv = uv;
                mesh.subMeshCount = 2;
                mesh.SetTriangles(sub0, 0);
                mesh.SetTriangles(sub1, 1);
                mesh.RecalculateBounds();
                return mesh;
            }
        }
    }
}
