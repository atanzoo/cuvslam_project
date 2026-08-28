# cuVSLAM D435i mapping project

This repository is the independent cuVSLAM development line for a RealSense
D435i stereo camera and IMU on a Jetson running Isaac ROS. It covers input and
frame contracts, deployment, repeatable experiments, the macOS monitoring GUI,
and the evidence needed before localization or Nav2 integration.

The cuVSLAM line remains separate from the existing LiDAR/2D-map/Nav2 system.
An experiment result is not an integration result until it passes the defined
comparison and integration review.

## Current state

- R1 stereo/IMU transport and R1.5 IMU qualification are complete.
- R2 stereo+IMU mapping and live odometry monitoring are operational.
- Official 640x360x30 input is the default baseline.
- Full path retention, process logs, quality metrics, and R2 trace summaries
  are implemented.
- Good-image conditions pass the current engineering gate; a special-scene
  turn can still produce false translation and remains an explicit limitation.

Read [`docs/CUVSLAM_CURRENT_STATUS_20260825.md`](docs/CUVSLAM_CURRENT_STATUS_20260825.md)
and [`docs/REAL_D435I_MAPPING_HANDOFF_20260817.md`](docs/REAL_D435I_MAPPING_HANDOFF_20260817.md)
before operating the hardware.

## Main entry points

- `Real D435i R1.5 Axis Test.command` — starts the local monitoring GUI.
- `tools/real_d435i_axis_web_gui.py` — GUI and R2 measurement workflow.
- `tools/run_real_d435i_r2.sh` — Jetson-side real R2 launcher.
- `tools/open_real_d435i_rviz_view.sh` — Jetson RViz/VNC viewer helper.
- `deployment/isaac_ros/isaac_ros_visual_slam_d435i_real.launch.py` — real
  camera Isaac ROS launch configuration.

## Directory map

| Directory | Purpose |
|---|---|
| `tools/` | Repeatable GUI, launch, inspection, and analysis tools |
| `deployment/` | Jetson/Isaac ROS boundary and simulation adapters |
| `config/` | Shareable RViz/Xorg configuration |
| `calibration/` | Camera, IMU, TF, timestamp, and hardware records |
| `docs/` | Handoffs, contracts, design decisions, and test plans |
| `reports/` | Curated experiment and acceptance reports |
| `logs/` | Runtime evidence; only selected records belong in Git |
| `models/` | Robot description sources |

## Local setup

Copy `config/local.env.example` to `config/local.env` and set the Jetson
connection values. The local file is ignored by Git. The Jetson also needs its
own ignored `/home/tseng/isaac_ros_ws/.local.env` for the RViz VNC password.
Do not put credentials in scripts, reports, screenshots, or commit history.

For the repository upload policy and pre-push checklist, see
[`docs/GITHUB_REPOSITORY_GUIDE.md`](docs/GITHUB_REPOSITORY_GUIDE.md).

## Development flow

```text
R1/R1.5 contract
  -> R2 stereo+IMU odometry
  -> straight/turn/closed-loop evaluation
  -> real mapping
  -> localization
  -> Nav2 integration and A/B comparison
```

Simulation and D1 navigation research are separate workstreams; they do not
replace the real-camera mapping gate.
