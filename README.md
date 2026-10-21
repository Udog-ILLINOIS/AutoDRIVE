# AutoDRIVE RoboRacer multi-track (F1Tenth / SIGRobotics)

Adds a `-track <name>` launch argument to the stock AutoDRIVE RoboRacer simulator
(release [`2026-iros`](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/tag/2026-iros),
macOS practice build) through a small runtime plugin, `TrackSelect.dll`, plus custom tracks.
The stock scene data is not modified.

- `track_select/`: plugin source, built against the stock app's own `Managed/` assemblies:
  `TrackSelector.cs` (track by `-track`, and switching while the sim runs) and `SimControl.cs`
  (`-autoconnect`, a control port on 127.0.0.1:4580, and a Map row in the sim's Menu panel)
- `tracks/<name>/`: custom tracks (`mesh.bin` + `track.json`), loaded from the app's `StreamingAssets/tracks/`
- `build_track.py`: `walls` (drawn map to wall polylines + ROS map), `export` (track mesh + checkpoints), `app` (stock app to multi-track app)

Track names: built in `porto` (default), `berlin`, `srl2024cdc`, `srl2024iros`, `srl2025icra`, `srl2025cdctf`; custom `iros2026`.

`SimControl.cs`: with `-autoconnect` the sim connects to the bridge (`-ip`/`-port`, default
127.0.0.1:4567) and reconnects after bridge restarts, keeping its Socket.IO client off while no bridge
answers (left on, it retries without pause, leaking a socket and logging a stack trace each time).
When it drops the connection it also zeroes the car's throttle and steering, so a dead bridge does not
leave the car driving on its last command.
The sim's lap and collision counters (`lap_count`, `collision_count`) restart at 0 on every reset
(Scene Reset, `/autodrive/reset_command`, a track switch): count increments, not differences from a
value read earlier. A track switch runs the sim's own reset (ResetManager) at once, so a bridge message
cannot cancel it.
Control port commands (one line per TCP connection, JSON reply): `status`, `tracks`, `track <name>`,
`connect [ip [port]]`, `disconnect`, `reconnect`, `kick`,
`click <button>` (a mouse click's pointer events to the visible button with that name; the way to press
the sim's buttons, e.g. `Menu`, `"Driving Mode"`, `Map`), `screenshot [path]` (the sim's picture only),
`tree [^]<object> [depth]`, and `clickat <x> <y>` (by position, 0-1 of the window; layout checks only,
positions depend on the window). Car back to its start pose: `/autodrive/reset_command` in Autonomous
with a bridge, otherwise `track <current track>`; Scene Reset reloads the scene.
From the F1Tenth repo: `./autodrive.sh ctl <command>`.

## Prebuilt app

The built macOS app is attached to this fork's [Releases](https://github.com/Udog-ILLINOIS/AutoDRIVE/releases)
(not committed to git). From the F1Tenth repo: `./autodrive.sh fetch`, then `./autodrive.sh sim --track iros2026`.

## Rebuilding

Needs the .NET 8 SDK in `~/.dotnet` and the stock app
([`autodrive_simulator_practice_macos.zip`](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/download/2026-iros/autodrive_simulator_practice_macos.zip)).

```bash
python3 -m venv venv && venv/bin/pip install -r requirements.txt
venv/bin/python build_track.py app "<stock>/AutoDRIVE Simulator.app" "<out>/AutoDRIVE Simulator.app"
```

The source app can also be an earlier multi-track build (its plugin and tracks are replaced).

See the docstring at the top of `build_track.py` for the full pipeline.
