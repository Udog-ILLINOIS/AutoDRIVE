#!/usr/bin/env python3
"""Custom racetracks for the AutoDRIVE RoboRacer simulator, switchable by launch argument.

  venv/bin/python build_track.py walls <drawn.png> <drawn.yaml> <map_dir> <map_name> [--start x,y,yaw]
  venv/bin/python build_track.py export <map_dir>/<map_name>_walls.json <global_waypoints.json> <name> "<Display Name>"
  venv/bin/python build_track.py app <stock.app> <dst.app>
  "<dst.app>/Contents/MacOS/AutoDRIVE Simulator" -track <name>     (or ./autodrive.sh sim --track <name>)

How the sim's tracks are built (2026-iros build, read with UnityPy and ILSpy): the scene holds six
tracks under `Infrastructure` (Porto active; Berlin, SRL 2024 CDC/IROS, SRL 2025 ICRA/CDC-TF
disabled), each "<X> Track" (one mesh of 10" or 12" duct tubes with a MeshCollider; two submeshes
on the same triangles, outward and inward faces), "<X> Checkpoints" (21 trigger boxes "0".."20"
+ finish lines), "<X> Spawn Points" and "<X> Camera Target". LapTimer counts a crash when the car
hits a collider named LapTimer.RacetrackName, and laps from LapTimer.Checkpoints. The build has
no way to pick a track, so `app` adds TrackSelect.dll (track_select/), which reads `-track` at
startup, enables that track and points LapTimer, the car's start pose and the cameras at it.

walls: traces the drawn wall lines (thin black lines; grid dots and specks are dropped) into
  polylines in the map frame, and writes <map_name>_walls.json plus the ForzaETH map
  <map_name>.png/.yaml rasterized from those same ducts, so the ROS map matches the sim walls.
export: writes tracks/<name>/mesh.bin (Porto-style duct mesh) and track.json (checkpoints spaced
  along the centerline of global_waypoints.json, spawn at the map's initial_pose). The plugin
  builds the track at startup by cloning Porto's objects. The map frame is the sim's world frame
  (ROS x = Unity z, ROS y = -Unity x), so ForzaETH needs no localization offset.
app: copies the stock app, adds TrackSelect.dll (built with ~/.dotnet) and every tracks/<name>/,
  and signs the copy ad hoc. The stock app's own data files are not modified.
"""
import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

import cv2
import numpy as np
import yaml
from skimage.morphology import skeletonize

DUCT_R = 0.127          # Porto's 10" ducts; the Porto Track transform lifts the mesh by this much
RING_SIDES = 16
RING_STEP = 0.10        # m between duct rings
INFRA = np.array([-10.0, 0.0, -14.0])   # Infrastructure position (Unity world)


# ---------------------------------------------------------------- walls from a drawing

def trace_skeleton(sk):
    """8-connected skeleton -> list of pixel polylines (row, col). Junction clusters are merged."""
    h, w = sk.shape
    pts = set(zip(*np.nonzero(sk)))
    nb = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

    def neigh(p):
        return [(p[0] + a, p[1] + b) for a, b in nb if (p[0] + a, p[1] + b) in pts]

    node = {p for p in pts if len(neigh(p)) != 2}
    used = set()
    lines = []

    def walk(a, b):
        path = [a, b]
        used.add((a, b)); used.add((b, a))
        while path[-1] not in node:
            nxt = [q for q in neigh(path[-1]) if (path[-1], q) not in used and q != path[-2]]
            if not nxt:
                break
            q = nxt[0]
            used.add((path[-1], q)); used.add((q, path[-1]))
            path.append(q)
            if q == path[0]:
                break
        return path

    for p in node:
        for q in neigh(p):
            if (p, q) not in used:
                lines.append(walk(p, q))
    for p in pts:   # pure loops with no node
        for q in neigh(p):
            if (p, q) not in used:
                lines.append(walk(p, q))
    return lines


def smooth_resample(xy, step, closed):
    """Moving-average smoothing, then even spacing along the line."""
    k = 5
    if len(xy) > 2 * k:
        if closed:
            pad = np.vstack([xy[-k:], xy, xy[:k]])
            ker = np.ones(2 * k + 1) / (2 * k + 1)
            xy = np.column_stack([np.convolve(pad[:, i], ker, 'valid') for i in range(2)])
        else:
            out = xy.copy()
            for i in range(k, len(xy) - k):
                out[i] = xy[i - k:i + k + 1].mean(0)
            xy = out
    if closed:
        xy = np.vstack([xy, xy[:1]])
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    s = np.concatenate([[0], np.cumsum(seg)])
    n = max(int(s[-1] / step), 1)
    t = np.linspace(0, s[-1], n + 1)
    res = np.column_stack([np.interp(t, s, xy[:, 0]), np.interp(t, s, xy[:, 1])])
    return res[:-1] if closed else res


def cmd_walls(a):
    meta = yaml.safe_load(open(a.yaml))
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    img = cv2.imread(a.png, cv2.IMREAD_GRAYSCALE)
    H = img.shape[0]
    wall = (img < 128).astype(np.uint8)
    n, lab, st, _ = cv2.connectedComponentsWithStats(wall, connectivity=8)
    for i in range(1, n):
        if st[i, cv2.CC_STAT_AREA] < a.min_wall_px:
            wall[lab == i] = 0
    wall = cv2.morphologyEx(wall, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))   # bridge 1 px gaps
    sk = skeletonize(wall.astype(bool), method='lee')
    lines = []
    for pl in trace_skeleton(sk):
        if len(pl) < a.min_line_px:
            continue
        rc = np.array(pl, float)
        xy = np.column_stack([ox + rc[:, 1] * res, oy + (H - 1 - rc[:, 0]) * res])   # map frame
        closed = pl[0] == pl[-1]
        lines.append({'closed': bool(closed), 'xy': smooth_resample(xy, RING_STEP, closed).round(4).tolist()})
    total = sum(np.linalg.norm(np.diff(np.array(l['xy']), axis=0), axis=1).sum() for l in lines)
    print(f'{len(lines)} wall polylines, {total:.1f} m of duct')

    # ForzaETH map rasterized from the ducts (same resolution/origin as the drawing)
    occ = np.zeros_like(img, np.uint8)
    rad_px = max(int(round(DUCT_R / res)), 1)
    for l in lines:
        xy = np.array(l['xy'])
        pix = np.column_stack([(xy[:, 0] - ox) / res, H - 1 - (xy[:, 1] - oy) / res]).round().astype(np.int32)
        cv2.polylines(occ, [pix], l['closed'], 1, thickness=2 * rad_px + 1)
    free = (1 - occ).astype(np.uint8)
    n, lab, st, _ = cv2.connectedComponentsWithStats(free, connectivity=4)
    best = None
    for i in range(1, n):
        x, y, w, h, area = st[i]
        if x == 0 or y == 0 or x + w == img.shape[1] or y + h == img.shape[0]:
            continue
        if best is None or area > st[best, 4]:
            best = i
    if best is None:
        sys.exit('The ducts do not enclose a track.')
    os.makedirs(a.map_dir, exist_ok=True)
    cv2.imwrite(os.path.join(a.map_dir, f'{a.map_name}.png'), np.where(lab == best, 255, 0).astype(np.uint8))
    out_yaml = {'image': f'{a.map_name}.png', 'resolution': res, 'origin': [ox, oy, 0.0], 'negate': 0,
                'occupied_thresh': 0.65, 'free_thresh': 0.196,
                'initial_pose': [float(v) for v in a.start.split(',')]}
    with open(os.path.join(a.map_dir, f'{a.map_name}.yaml'), 'w') as f:
        yaml.safe_dump(out_yaml, f)
    with open(os.path.join(a.map_dir, f'{a.map_name}_walls.json'), 'w') as f:
        json.dump({'source': os.path.basename(a.png), 'duct_radius_m': DUCT_R, 'frame': 'map (= sim world, ROS)',
                   'lines': lines}, f)
    print(f'Wrote {a.map_name}.png/.yaml and {a.map_name}_walls.json in {a.map_dir}')


# ---------------------------------------------------------------- duct mesh

def ros_to_local(x, y):
    """Map/ROS (x, y) -> Porto Track mesh local (x, z) in Unity (Infrastructure frame)."""
    return -y - INFRA[0], x - INFRA[2]


def tube(xz, closed):
    """Rings along a polyline (Unity local x,z). Returns positions, outward normals, tangents, uv, quads."""
    n = len(xz)
    if closed:
        d = np.roll(xz, -1, 0) - np.roll(xz, 1, 0)
    else:
        d = np.gradient(xz, axis=0)
    d /= np.maximum(np.linalg.norm(d, axis=1, keepdims=True), 1e-9)
    side = np.column_stack([d[:, 1], -d[:, 0]])          # horizontal perpendicular
    seg = np.linalg.norm(np.diff(xz, axis=0), axis=1)
    s = np.concatenate([[0], np.cumsum(seg)])
    ang = np.linspace(0, 2 * np.pi, RING_SIDES + 1)      # last column duplicates the first for the uv seam
    ca, sa = np.cos(ang), np.sin(ang)
    nrm = np.zeros((n, RING_SIDES + 1, 3))
    nrm[..., 0] = side[:, None, 0] * ca
    nrm[..., 2] = side[:, None, 1] * ca
    nrm[..., 1] = sa[None, :]
    pos = nrm * DUCT_R
    pos[..., 0] += xz[:, None, 0]
    pos[..., 2] += xz[:, None, 1]
    tan = np.zeros((n, RING_SIDES + 1, 4))
    tan[..., 0] = d[:, None, 0]
    tan[..., 2] = d[:, None, 1]
    tan[..., 3] = 1.0
    uv = np.stack(np.broadcast_arrays(s[:, None] / (2 * np.pi * DUCT_R), ang[None, :] / (2 * np.pi)), -1)
    m = RING_SIDES + 1
    quads = []
    rings = list(range(n)) + ([0] if closed else [])
    for i in range(len(rings) - 1):
        r0, r1 = rings[i], rings[i + 1]
        for j in range(RING_SIDES):
            quads.append((r0 * m + j, r1 * m + j, r1 * m + j + 1, r0 * m + j + 1))
    return pos.reshape(-1, 3), nrm.reshape(-1, 3), tan.reshape(-1, 4), uv.reshape(-1, 2), np.array(quads)


def build_mesh(walls):
    P, N, T, U, Q = [], [], [], [], []
    off = 0
    for l in walls['lines']:
        xy = np.array(l['xy'])
        lx, lz = ros_to_local(xy[:, 0], xy[:, 1])
        p, nn, t, u, q = tube(np.column_stack([lx, lz]), l['closed'])
        P.append(p); N.append(nn); T.append(t); U.append(u); Q.append(q + off)
        off += len(p)
    P, N, T, U, Q = map(np.concatenate, (P, N, T, U, Q))
    tri = np.concatenate([Q[:, [0, 1, 2]], Q[:, [0, 2, 3]]])
    # Unity front faces: cross(b-a, c-a) along the normal (checked against Porto's mesh)
    a, b, c = P[tri[:, 0]], P[tri[:, 1]], P[tri[:, 2]]
    flip = (np.cross(b - a, c - a) * (N[tri[:, 0]] + N[tri[:, 1]] + N[tri[:, 2]])).sum(1) < 0
    tri[flip] = tri[flip][:, [0, 2, 1]]
    nv = len(P)
    outer = np.hstack([P, N, T, U]).astype(np.float32)
    inner = np.hstack([P, -N, T * [1, 1, 1, -1], U]).astype(np.float32)
    verts = np.vstack([outer, inner])
    idx = np.concatenate([tri.ravel(), (tri[:, [0, 2, 1]] + nv).ravel()]).astype(np.uint32)
    return verts, idx, nv, len(tri) * 3


# ---------------------------------------------------------------- custom track files

def marker(name, x, y, heading, width=0.0, height=0.01):
    """Map pose -> marker in Infrastructure-local Unity coordinates (yaw in degrees, = -heading)."""
    lx, lz = ros_to_local(x, y)
    return {'name': name, 'pos': [float(lx), height, float(lz)], 'yaw': float(-math.degrees(heading)),
            'width': float(width)}


def cmd_export(a):
    """Write tracks/<name>/{mesh.bin, track.json} for the TrackSelect plugin."""
    walls = json.load(open(a.walls))
    wp = json.load(open(a.waypoints))['centerline_waypoints']['wpnts']
    cx = np.array([w['x_m'] for w in wp]); cy = np.array([w['y_m'] for w in wp])
    cs = np.array([w['s_m'] for w in wp]); cpsi = np.array([w['psi_rad'] for w in wp])
    cw = np.array([w['d_left'] + w['d_right'] for w in wp])
    lap = cs[-1]
    # s is measured from the map's initial_pose (the start line), not the centerline's own s = 0
    start = yaml.safe_load(open(a.walls.replace('_walls.json', '.yaml')))['initial_pose']
    s_start = cs[int(np.argmin(np.hypot(cx - start[0], cy - start[1])))]

    def at(s):
        i = int(np.argmin(np.abs(cs - ((s_start + s) % lap))))
        return cx[i], cy[i], cpsi[i], cw[i]

    # Porto's markers: checkpoints "0".."20" evenly over the lap (0 = the start line), finish line
    # A on the start line and B half a lap on (second agent), spawns 0.6 m past each finish line.
    n_cp, s0, half = 21, 0.6, lap / 2
    cps = []
    for k in range(n_cp):
        x, y, psi, w = at(k * lap / n_cp)
        cps.append(marker(str(k), x, y, psi, w + 2 * DUCT_R))
    for name, s in (('Agent-A Finish Line', 0.0), ('Agent-B Finish Line', half)):
        x, y, psi, w = at(s)
        cps.append(marker(name, x, y, psi, w + 2 * DUCT_R))
    spawns = [marker(name, *at(s)[:3]) for name, s in (('Spawn 1', s0), ('Spawn 2', half + s0))]
    lx, lz = ros_to_local((cx.min() + cx.max()) / 2, (cy.min() + cy.max()) / 2)
    track = {'name': a.display_name, 'checkpoints': cps, 'spawns': spawns,
             'camera': [float(lx), float(max(np.ptp(cx), np.ptp(cy)) * 0.55), float(lz)]}

    verts, idx, nv, ni = build_mesh(walls)
    out = os.path.join(a.out_dir, a.name)
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, 'mesh.bin'), 'wb') as f:
        f.write(np.array([len(verts), ni], np.int32).tobytes())
        for cols in ((0, 3), (3, 6), (6, 10), (10, 12)):          # pos, normal, tangent, uv
            f.write(np.ascontiguousarray(verts[:, cols[0]:cols[1]]).tobytes())
        f.write(idx.astype(np.int32).tobytes())                     # sub0 then sub1, ni each
    with open(os.path.join(out, 'track.json'), 'w') as f:
        json.dump(track, f, indent=1)
    x, y, psi, _ = at(s0)
    print(f"Exported '{a.display_name}' to {out}: {len(verts)} vertices, {2 * ni // 3} triangles, "
          f'{n_cp} checkpoints over {lap:.1f} m, spawn ({x:.2f}, {y:.2f}) heading {psi:.2f} rad')


# ---------------------------------------------------------------- the multi-track app

def cmd_app(a):
    """Stock app + TrackSelect.dll + every exported track -> dst (ad-hoc signed)."""
    here = os.path.dirname(os.path.abspath(__file__))
    managed_src = os.path.join(os.path.abspath(a.src), 'Contents/Resources/Data/Managed')
    # Build and sign outside the (iCloud-synced) workspace: iCloud adds Finder xattrs that
    # codesign rejects ("resource fork, Finder information, or similar detritus").
    tmp = tempfile.mkdtemp(prefix='autodrive_track_')
    dll_out = os.path.join(tmp, 'dll')
    env = dict(os.environ, DOTNET_ROOT=os.path.expanduser('~/.dotnet'), DOTNET_CLI_TELEMETRY_OPTOUT='1')
    dotnet = shutil.which('dotnet') or os.path.expanduser('~/.dotnet/dotnet')
    subprocess.run([dotnet, 'build', os.path.join(here, 'track_select'), '-c', 'Release', '-nologo', '-v', 'q',
                    f'-p:ManagedDir={managed_src}', '-o', dll_out], check=True, env=env)

    app = os.path.join(tmp, os.path.basename(a.dst))
    subprocess.run(['ditto', '--noextattr', a.src, app], check=True)
    data = os.path.join(app, 'Contents/Resources/Data')
    shutil.copy(os.path.join(dll_out, 'TrackSelect.dll'), os.path.join(data, 'Managed'))
    sa_path = os.path.join(data, 'ScriptingAssemblies.json')
    sa = json.load(open(sa_path))
    if 'TrackSelect.dll' not in sa['names']:
        sa['names'].append('TrackSelect.dll'); sa['types'].append(16)   # 16 = user assembly
    json.dump(sa, open(sa_path, 'w'))
    ri_path = os.path.join(data, 'RuntimeInitializeOnLoads.json')
    ri = json.load(open(ri_path))
    ri['root'] = [r for r in ri['root'] if r['assemblyName'] != 'TrackSelect']
    ri['root'].append({'assemblyName': 'TrackSelect', 'nameSpace': 'TrackSelect', 'className': 'TrackSelector',
                       'methodName': 'Init', 'loadTypes': 1, 'isUnityClass': False})   # 1 = BeforeSceneLoad
    json.dump(ri, open(ri_path, 'w'))
    tracks = sorted(d for d in os.listdir(a.tracks) if os.path.isfile(os.path.join(a.tracks, d, 'track.json')))
    for t in tracks:
        shutil.copytree(os.path.join(a.tracks, t), os.path.join(data, 'StreamingAssets', 'tracks', t))
    subprocess.run(['xattr', '-cr', app], check=False)
    subprocess.run(['codesign', '--force', '--deep', '-s', '-', app], check=True, stderr=subprocess.DEVNULL)
    if os.path.exists(a.dst):
        shutil.rmtree(a.dst)
    os.makedirs(os.path.dirname(os.path.abspath(a.dst)), exist_ok=True)
    subprocess.run(['ditto', app, a.dst], check=True)
    shutil.rmtree(tmp)
    print(f'Wrote {a.dst} (ad-hoc signed) with TrackSelect and custom tracks: {", ".join(tracks) or "none"}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('walls')
    p.add_argument('png'); p.add_argument('yaml'); p.add_argument('map_dir'); p.add_argument('map_name')
    p.add_argument('--start', default='0,0,0', help='ForzaETH initial_pose x,y,yaw (start line, driving direction)')
    p.add_argument('--min-wall-px', type=int, default=60, help='black blobs smaller than this are erased')
    p.add_argument('--min-line-px', type=int, default=8, help='traced wall pieces shorter than this are dropped')
    p.set_defaults(f=cmd_walls)
    p = sub.add_parser('export')
    p.add_argument('walls'); p.add_argument('waypoints'); p.add_argument('name'); p.add_argument('display_name')
    p.add_argument('--out-dir', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tracks'))
    p.set_defaults(f=cmd_export)
    p = sub.add_parser('app')
    p.add_argument('src'); p.add_argument('dst')
    p.add_argument('--tracks', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tracks'))
    p.set_defaults(f=cmd_app)
    a = ap.parse_args()
    a.f(a)


if __name__ == '__main__':
    main()
