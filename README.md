# AutoDRIVE RoboRacer multi-track (F1Tenth / SIGRobotics)

Adds a `-track <name>` launch argument to the stock AutoDRIVE RoboRacer simulator
(release [`2026-iros`](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/tag/2026-iros),
macOS practice build) through a small runtime plugin, `TrackSelect.dll`, plus custom tracks.
The stock scene data is not modified.

- `track_select/`: plugin source (`TrackSelector.cs`), built against the stock app's own `Managed/` assemblies
- `tracks/<name>/`: custom tracks (`mesh.bin` + `track.json`), loaded from the app's `StreamingAssets/tracks/`
- `build_track.py`: `walls` (drawn map to wall polylines + ROS map), `export` (track mesh + checkpoints), `app` (stock app to multi-track app)

Track names: built in `porto` (default), `berlin`, `srl2024cdc`, `srl2024iros`, `srl2025icra`, `srl2025cdctf`; custom `iros2026`.

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

See the docstring at the top of `build_track.py` for the full pipeline.
