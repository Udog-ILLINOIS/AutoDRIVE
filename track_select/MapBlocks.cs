// The map view's block bar: a thin grey strip between the tabs and the figure with a square in it. Drag the square onto the
// track and let go: a block (BlockSize x 0.4 x BlockSize m, SimControl.CreateObstacle: the lidar sees it
// and the car collides with it) stands there on the real track. While dragged over the figure the
// square is drawn to scale. Each block has a handle on the figure (a dark grey square): drag it to move
// the block, or back into the bar to remove it. Only while the figure is in the world frame (the Ground
// Truth tab, and the Mapping tab once it is aligned); blocks placed with the control port's `obstacle`
// get handles too. Only on the Ground Truth tab: on the Mapping tab the bar and the handles are hidden
// (its strip stays, empty, so the map is the same size and in the same place as on Ground Truth).
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

namespace TrackSelect
{
    // Forwards a UI element's drag events to MapView. block: the block a handle stands for, or null for
    // the bar's square (a new block).
    public class MapDrag : MonoBehaviour, IBeginDragHandler, IDragHandler, IEndDragHandler
    {
        public MapView owner;
        public GameObject block;
        public void OnBeginDrag(PointerEventData e) => owner.BeginBlockDrag(this, e);
        public void OnDrag(PointerEventData e) => owner.BlockDrag(e);
        public void OnEndDrag(PointerEventData e) => owner.EndBlockDrag(this, e);
    }

    public partial class MapView
    {
        const float BlockSize = 0.3f;   // m, as SimControl's obstacle command
        static readonly Color BarColor = new Color(0.36f, 0.37f, 0.40f, 1f);
        static readonly Color BlockColor = new Color(0.13f, 0.13f, 0.14f, 1f);

        RectTransform bar, barSquare, ghost;
        readonly List<(GameObject block, RectTransform handle, Vector2 at, float size)> blocks =
            new List<(GameObject, RectTransform, Vector2, float)>();
        bool figureIsWorld;   // the figure shows gtRect (world frame): blocks can be placed on it
        float figurePxPerM;
        bool blocksDirty = true;

        void BuildBar()
        {
            bar = NewRect("Block Bar", view, new Vector2(0f, 1f), new Vector2(1f, 1f));
            bar.pivot = new Vector2(0.5f, 1f);
            bar.gameObject.AddComponent<Image>().color = BarColor;
            barSquare = NewRect("Block", bar, new Vector2(0f, 0.5f), new Vector2(0f, 0.5f));
            barSquare.pivot = new Vector2(0f, 0.5f);
            Image sq = barSquare.gameObject.AddComponent<Image>();
                        sq.color = BlockColor;
            var d = barSquare.gameObject.AddComponent<MapDrag>();
            d.owner = this;
            blocks.Clear();
            blocksDirty = true;
        }

        // The strip across the top of the view, thin (grows with the window); returns its height.
        float LayoutBar(Rect r)
        {
            float h = Mathf.Clamp(r.height * 0.075f, 8f, 48f);
            bar.sizeDelta = new Vector2(0f, h);
            bar.anchoredPosition = Vector2.zero;
            float s = h * 0.72f;
            barSquare.sizeDelta = new Vector2(s, s);
            barSquare.anchoredPosition = new Vector2((h - s) / 2f, 0f);
            bar.gameObject.SetActive(!mappingTab);   // Ground Truth only; the strip's room is kept either way
            return h;
        }

        // The blocks in the scene, read again after anything changed them (MapView.Invalidate).
        void RefreshBlocks()
        {
            foreach (var b in blocks) if (b.handle != null) Destroy(b.handle.gameObject);
            blocks.Clear();
            foreach (GameObject o in Resources.FindObjectsOfTypeAll<GameObject>()
                         .Where(g => g.name == SimControl.ObstacleName && g.scene.isLoaded && g.activeInHierarchy))
            {
                Renderer rd = o.GetComponent<Renderer>();
                Bounds ob = rd != null ? rd.bounds : new Bounds(o.transform.position, o.transform.localScale);
                RectTransform h = NewRect("Block Handle", image, new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f));
                Image img = h.gameObject.AddComponent<Image>();
                                img.color = BlockColor;
                h.gameObject.AddComponent<MapDrag>().owner = this;
                h.GetComponent<MapDrag>().block = o;
                blocks.Add((o, h, new Vector2(ob.center.z, -ob.center.x), Mathf.Max(ob.size.x, ob.size.z)));
            }
            blocksDirty = false;
        }

        // Handles on the figure, to scale (at least 8 px); hidden while the figure is not in the world frame.
        void UpdateHandles()
        {
            if (blocksDirty) RefreshBlocks();
            foreach (var b in blocks)
            {
                if (b.handle == null) continue;
                b.handle.gameObject.SetActive(!mappingTab && figureIsWorld && b.block != null && b.block.activeInHierarchy);
                if (!b.handle.gameObject.activeSelf) continue;
                Vector2 uv = new Vector2((b.at.x - gtRect.xMin) / gtRect.width, (b.at.y - gtRect.yMin) / gtRect.height);
                b.handle.anchorMin = b.handle.anchorMax = uv;
                b.handle.anchoredPosition = Vector2.zero;
                float s = Mathf.Max(8f, b.size * figurePxPerM);
                b.handle.sizeDelta = new Vector2(s, s);
                b.handle.SetAsLastSibling();
            }
            marker.SetAsLastSibling();   // the car on top
        }

        // ------------------------------------------------------------ dragging

        public void BeginBlockDrag(MapDrag from, PointerEventData e)
        {
            Canvas canvas = view.GetComponentInParent<Canvas>().rootCanvas;
            ghost = NewRect("Block Ghost", canvas.transform, new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f));
            Image img = ghost.gameObject.AddComponent<Image>();
                        img.color = BlockColor;
            img.raycastTarget = false;
            ghost.SetAsLastSibling();
            if (from.block != null) from.GetComponent<Image>().enabled = false;   // the handle travels with the pointer
            BlockDrag(e);
        }

        public void BlockDrag(PointerEventData e)
        {
            if (ghost == null) return;
            Camera cam = e.pressEventCamera;
            if (RectTransformUtility.ScreenPointToWorldPointInRectangle((RectTransform)ghost.parent, e.position, cam, out Vector3 p))
                ghost.position = p;
            // to scale over the figure, the bar's size elsewhere (canvas units; the ghost is on the root canvas)
            float scale = ghost.parent.lossyScale.x / view.lossyScale.x;
            float s = ToWorld(e, out _) ? Mathf.Max(8f, BlockSize * figurePxPerM) : barSquare.rect.width;
            ghost.sizeDelta = new Vector2(s, s) / scale;
        }

        public void EndBlockDrag(MapDrag from, PointerEventData e)
        {
            if (ghost != null) Destroy(ghost.gameObject);
            ghost = null;
            Camera cam = e.pressEventCamera;
            bool overBar = RectTransformUtility.RectangleContainsScreenPoint(bar, e.position, cam);
            if (overBar)
            {
                if (from.block != null) SimControl.RemoveObstacle(from.block);   // back into the bar: gone
                return;
            }
            if (ToWorld(e, out Vector2 at))
            {
                float size = BlockSize;
                if (from.block != null)
                {
                    Renderer rd = from.block.GetComponent<Renderer>();
                    if (rd != null) size = Mathf.Max(rd.bounds.size.x, rd.bounds.size.z);
                    SimControl.RemoveObstacle(from.block);   // moved: placed again at the new spot
                }
                SimControl.CreateObstacle(at.x, at.y, size);
                return;
            }
            if (from.block != null) from.GetComponent<Image>().enabled = true;   // dropped elsewhere: stays
        }

        // The ROS world point under the pointer, if it is on the figure and the figure is in the world frame.
        bool ToWorld(PointerEventData e, out Vector2 at)
        {
            at = Vector2.zero;
            if (mappingTab || !figureIsWorld || !imageRaw.enabled) return false;
            if (!RectTransformUtility.ScreenPointToLocalPointInRectangle(image, e.position, e.pressEventCamera, out Vector2 lp))
                return false;
            Rect r = image.rect;
            float u = (lp.x - r.xMin) / r.width, v = (lp.y - r.yMin) / r.height;
            if (u < 0f || v < 0f || u > 1f || v > 1f) return false;
            // only what the view shows (the figure is clipped to it)
            if (!RectTransformUtility.RectangleContainsScreenPoint(view, e.position, e.pressEventCamera)) return false;
            at = new Vector2(gtRect.xMin + u * gtRect.width, gtRect.yMin + v * gtRect.height);
            return true;
        }
    }
}
