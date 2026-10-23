// Decals added to the car (e.g. the club logo on the rear fin), from StreamingAssets/livery/livery.json:
//   {"decals": [{"name": "SIGRobotics Fin", "image": "sigrobotics_logo.png",
//                "pos": [x, y, z], "rot": [x, y, z], "width": 0.08}]}
// pos (m) and rot (Euler, deg) are in the car's own frame (the "Roboracer" object, Unity axes: x right,
// y up, z forward); width is in m, the height follows the image. Each decal is a clone of the car's own
// "Decals" world-space canvas (the AutoDRIVE / RoboRacer logos), so it renders and moves like them.
// The UI shader draws both faces, so a decal on a thin fin shows (mirrored) from the other side too.
// Applied on every scene load; the control port's `livery` command re-reads the file (for placing one).
// Log lines start with [Livery].
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

namespace TrackSelect
{
    [Serializable] public class Decal { public string name; public string image; public float[] pos; public float[] rot; public float width; }
    [Serializable] public class LiveryFile { public Decal[] decals; }

    public static class Livery
    {
        const string Tag = "[Livery] ";
        const string Prefix = "Livery ";

        public static void Init() => SceneManager.sceneLoaded += (scene, mode) => Apply();

        // re-reads livery.json and replaces the decals; returns a summary (or the error)
        public static string Apply()
        {
            try { return ApplyUnsafe(); }
            catch (Exception e) { Debug.LogError(Tag + e); return "error: " + e.Message; }
        }

        static string ApplyUnsafe()
        {
            string dir = Path.Combine(Application.streamingAssetsPath, "livery");
            string file = Path.Combine(dir, "livery.json");
            if (!File.Exists(file)) return "no " + file;
            GameObject carGo = SceneManager.GetActiveScene().GetRootGameObjects().FirstOrDefault(g => g.name == "Roboracer");
            if (carGo == null) return "no Roboracer object";
            Transform car = carGo.transform;
            Transform template = car.Find("Decals");
            if (template == null) return "no Roboracer/Decals canvas";

            foreach (Transform t in car.Cast<Transform>().Where(t => t.name.StartsWith(Prefix)).ToList())
                UnityEngine.Object.Destroy(t.gameObject);

            var placed = new List<string>();
            foreach (Decal d in JsonUtility.FromJson<LiveryFile>(File.ReadAllText(file)).decals ?? new Decal[0])
            {
                string path = Path.Combine(dir, d.image);
                if (!File.Exists(path)) { Debug.LogError(Tag + "no image " + path); continue; }
                var tex = new Texture2D(2, 2, TextureFormat.RGBA32, true);
                if (!tex.LoadImage(File.ReadAllBytes(path))) { Debug.LogError(Tag + "can't read " + path); continue; }
                tex.wrapMode = TextureWrapMode.Clamp;
                tex.anisoLevel = 4;

                GameObject canvas = UnityEngine.Object.Instantiate(template.gameObject, car);
                canvas.name = Prefix + d.name;
                canvas.SetActive(true);
                foreach (Transform c in canvas.transform) UnityEngine.Object.Destroy(c.gameObject);
                var rt = (RectTransform)canvas.transform;
                rt.localPosition = V(d.pos);
                rt.localRotation = Quaternion.Euler(V(d.rot));
                rt.localScale = Vector3.one;
                rt.pivot = new Vector2(0.5f, 0.5f);
                rt.sizeDelta = new Vector2(d.width, d.width * tex.height / tex.width);

                var img = new GameObject("Image", typeof(RectTransform));
                img.layer = canvas.layer;
                var irt = (RectTransform)img.transform;
                irt.SetParent(rt, false);
                irt.anchorMin = Vector2.zero; irt.anchorMax = Vector2.one;
                irt.offsetMin = irt.offsetMax = Vector2.zero;
                img.AddComponent<CanvasRenderer>();
                img.AddComponent<Image>().sprite =
                    Sprite.Create(tex, new Rect(0, 0, tex.width, tex.height), new Vector2(0.5f, 0.5f));
                placed.Add(d.name);
            }
            string summary = "decals: " + string.Join(", ", placed);
            Debug.Log(Tag + summary);
            return summary;
        }

        static Vector3 V(float[] a) => a == null || a.Length < 3 ? Vector3.zero : new Vector3(a[0], a[1], a[2]);
    }
}
