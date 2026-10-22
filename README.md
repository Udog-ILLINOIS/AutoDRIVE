# AutoDRIVE RoboRacer Multi-Track

![AutoDRIVE](https://img.shields.io/badge/AutoDRIVE-RoboRacer%202026--iros-blue)
![Platform](https://img.shields.io/badge/platform-macOS-lightgrey?logo=apple)
![.NET](https://img.shields.io/badge/.NET-8%20SDK-512BD4?logo=dotnet&logoColor=white)
![License](https://img.shields.io/badge/license-BSD--2--Clause-green)

Multi-track and remote-control extension for the
[AutoDRIVE RoboRacer simulator](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing),
maintained by [SIGRobotics](https://github.com/SIGRobotics-UIUC) at the University of Illinois
Urbana-Champaign for the [roboracer](https://github.com/SIGRobotics-UIUC/roboracer) race stack.

The stock `2026-iros` practice build ships six tracks in its scene, but it can only load Porto, and it
can only be driven from its own UI. This branch adds a small runtime plugin, `TrackSelect.dll`, that
does three things:
- picks the track at launch or while the sim runs
- adds custom tracks built from a drawn map
- lets scripts control the sim from outside, so a test can run with no clicking

The stock app's own data files are not modified: the plugin is loaded next to them.

> [!NOTE]
> - Built and tested on the **macOS practice build** of release
>   [`2026-iros`](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/tag/2026-iros).
>   The plugin targets that build's assemblies. Other platforms and builds have not been tried.
> - Prebuilt app: release [`f1tenth_multitrack_v2`](https://github.com/Udog-ILLINOIS/AutoDRIVE/releases/tag/f1tenth_multitrack_v2)
>   (all features, built from `094f9d5`). The older `f1tenth_multitrack_v1` has track selection only.

## Features

| Feature | What it does |
|---|---|
| **Track selection** | `-track <name>` launch argument. It enables that track and points the lap timer, checkpoints, collision detection, spawn pose and cameras at it. |
| **Track switching while running** | A **Map** row in the sim's Menu panel and the `track` command switch tracks without restarting the sim. The car goes to the new track's start pose, and the sim's own reset runs at once (encoders, lap and collision counters). |
| **Custom tracks** | `build_track.py` turns a drawn track map into sim walls, checkpoints and a spawn pose, plus a matching ROS map. Included: `iros2026`. |
| **Autoconnect** | `-autoconnect` keeps the sim connected to the bridge and reconnects after bridge restarts, without clicking **Connect**. It works around the stock Socket.IO client, which retries without pause when no bridge answers (leaking a socket each time and flooding `Player.log`). |
| **Safe disconnect** | When the connection drops, the car's throttle and steering are zeroed, so a dead bridge does not leave the car driving on its last command. |
| **Control port** | A TCP control port on 127.0.0.1:4580 with JSON replies: status, track switching, connect/disconnect, pressing UI buttons by name, screenshots of the sim window. |

## Tracks

| Name | Track | Source |
|---|---|---|
| `porto` (default) | Porto | built in |
| `berlin` | Berlin | built in |
| `srl2024cdc`, `srl2024iros` | SRL 2024 CDC, SRL 2024 IROS | built in |
| `srl2025icra`, `srl2025cdctf` | SRL 2025 ICRA, SRL 2025 CDC-TF | built in |
| `iros2026` | IROS 2026 | custom, `tracks/iros2026/` |

An unknown name logs the available list in `Player.log` (lines starting with `[TrackSelect]`).

## Installation

Download `autodrive_simulator_multitrack_macos.zip` from release
[`f1tenth_multitrack_v2`](https://github.com/Udog-ILLINOIS/AutoDRIVE/releases/tag/f1tenth_multitrack_v2)
and unzip it. If macOS refuses to open it:

```bash
xattr -cr "AutoDRIVE Simulator.app"
```

With the [f1tenth_workspace](https://github.com/Udog-ILLINOIS/f1tenth_workspace) checkout, `./autodrive.sh fetch` does this.

## Usage

```bash
"AutoDRIVE Simulator.app/Contents/MacOS/AutoDRIVE Simulator" -track iros2026 -autoconnect
```

| Argument | Default | |
|---|---|---|
| `-track <name>` | Porto | track to load (see [Tracks](#tracks)); re-applied when the scene reloads |
| `-autoconnect` | off | connect to the bridge and stay connected |
| `-ip <address>`, `-port <n>` | `127.0.0.1`, `4567` | the bridge (the sim's own arguments) |
| `-control-port <n>` | `4580` | control port, on 127.0.0.1 only |

From the workspace: `./autodrive.sh sim [--track NAME] [--full] [--no-connect]`. It opens a 1280×720
window at "Very Low" quality, because the sim sends one sensor frame per rendered frame and a small
window gives a faster control loop.

### Control port

One command per TCP connection; one line in, one JSON line out:

```bash
printf 'status\n' | nc 127.0.0.1 4580       # or: ./autodrive.sh ctl status
```

| Command | |
|---|---|
| `status` | connected?, bridge address, render fps, sensor frames/s, last frame age, track, driving mode |
| `tracks` | track names for `track` |
| `track <name>` | switch track, car at its start pose |
| `connect [ip [port]]`, `disconnect`, `reconnect` | bridge connection |
| `kick` | send one telemetry frame, which restarts the exchange when the bridge has not replied since connecting (autoconnect does this by itself) |
| `click <button>` | press a visible UI button by name, e.g. `Menu`, `"Driving Mode"`, `Map`, `"Map Porto"`, `"Scene Reset"`. It sends a mouse click's pointer events and does not move the real mouse. |
| `screenshot [path]` | PNG of the sim's own picture, no other windows |
| `tree [^]<object> [depth]` | scene hierarchy under an object (`^`: its parent); lists button names |
| `clickat <x> <y>` | click by position (0-1 of the window). Positions change with window size and scaling, so use it only to check the layout. |

To put the car back at its start pose:
- **In Autonomous with a bridge:** publish the bridge's `/autodrive/reset_command`.
- **In any mode:** `track <current track>`.
- **Scene Reset** reloads the whole scene.

Commands run on Unity's main thread, so they wait while the sim is paused. Log lines start with `[SimControl]`.

## Building a custom track

`build_track.py` runs the pipeline from a drawn map to a track in the sim:

```bash
python3 -m venv venv && venv/bin/pip install -r requirements.txt

# 1. Drawn map (thin black wall lines, PNG + YAML) -> wall polylines and a ROS map rasterized from the same walls
venv/bin/python build_track.py walls drawn.png drawn.yaml <map_dir> <map_name> --start x,y,yaw

# 2. Walls + a centerline (global_waypoints.json) -> tracks/<name>/mesh.bin and track.json
venv/bin/python build_track.py export <map_dir>/<map_name>_walls.json global_waypoints.json <name> "<Display Name>"

# 3. Stock app -> multi-track app with the plugin and every tracks/<name>/
venv/bin/python build_track.py app "<stock>/AutoDRIVE Simulator.app" "<out>/AutoDRIVE Simulator.app"
```

Custom tracks clone Porto's objects (duct-tube walls, MeshCollider, trigger checkpoints), so collisions,
laps and respawns work as on the built-in tracks. The map frame is the sim's world frame
(ROS x = Unity z, ROS y = −Unity x), so a ROS stack needs no offset between the two. The docstring at the
top of `build_track.py` describes how the sim's tracks are built.

## Rebuilding the app

Needs:
- the [.NET 8 SDK](https://dotnet.microsoft.com/download/dotnet/8.0) in `~/.dotnet`
- the stock app, [`autodrive_simulator_practice_macos.zip`](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/download/2026-iros/autodrive_simulator_practice_macos.zip)

```bash
venv/bin/python build_track.py app "<stock>/AutoDRIVE Simulator.app" "<out>/AutoDRIVE Simulator.app"
```

`app` builds `TrackSelect.dll` against the stock app's own `Managed/` assemblies and registers it with
the player. It copies in every `tracks/<name>/`, then signs the copy ad hoc. The source can also be an
earlier multi-track build: its plugin and tracks are replaced.

## Repository layout

This is the `f1tenth_multitrack` branch of our fork of [Tinker-Twins/AutoDRIVE](https://github.com/Tinker-Twins/AutoDRIVE).

```
track_select/TrackSelector.cs    -track, built-in and custom tracks, switching while running
track_select/SimControl.cs       -autoconnect, control port, Map row in the Menu panel
track_select/TrackSelect.csproj  plugin project, built against the stock app's assemblies
tracks/<name>/                   custom tracks: mesh.bin (wall mesh) + track.json (checkpoints, spawns, camera)
build_track.py                   walls | export | app
requirements.txt                 Python dependencies of build_track.py
```

## Known limitations

- macOS practice build only; the plugin depends on that build's internal class names.
- In the Map list, a long track label leaves a wider gap than intended (cosmetic).
- The sim does not announce resets or respawns itself. A ROS bridge has to detect them from the frames
  (roboracer's `fast_bridge` does).

## Acknowledgement

Built on the [AutoDRIVE Ecosystem](https://github.com/Tinker-Twins/AutoDRIVE) and its
[RoboRacer Sim-Racing](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing) simulator
(BSD-2-Clause). The plugin does not include or modify any of the simulator's own files.

## Citation

If you use this, please cite the AutoDRIVE simulator:

#### [AutoDRIVE: A Comprehensive, Flexible and Integrated Digital Twin Ecosystem for Enhancing Autonomous Driving Research and Education](https://arxiv.org/abs/2212.05241)
```bibtex
@article{AutoDRIVE-Ecosystem-2023,
author = {Samak, Tanmay and Samak, Chinmay and Kandhasamy, Sivanathan and Krovi, Venkat and Xie, Ming},
title = {AutoDRIVE: A Comprehensive, Flexible and Integrated Digital Twin Ecosystem for Autonomous Driving Research & Education},
journal = {Robotics},
volume = {12},
year = {2023},
number = {3},
article-number = {77},
url = {https://www.mdpi.com/2218-6581/12/3/77},
issn = {2218-6581},
doi = {10.3390/robotics12030077}
}
```
