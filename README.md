# Quadruped navigation and cuVSLAM odometry research

This project investigates navigation for a quadruped without wheel encoders.
The navigation work motivates a reliable local odometry source: a RealSense
D435i stereo camera and IMU, processed by Isaac ROS cuVSLAM on a Jetson.
The repository includes sensor and frame contracts, deployment helpers,
repeatable experiments, monitoring interfaces, and navigation research.

The real-camera cuVSLAM line remains separate from the existing
LiDAR/2D-map/Nav2 solution track and the navigation proxy simulation.
An experiment result is not an integration result until it passes the defined
comparison and integration review.

## Current state

The following is a research snapshot of the recorded work, not a claim of
production-ready or real-quadruped autonomous navigation.

| Workstream | Recorded progress | Remaining boundary |
|---|---|---|
| Real D435i / cuVSLAM | Stereo/IMU transport and IMU qualification; live odometry, image-quality checks, trajectory recording and monitoring | Good-image bench tests pass; special-scene turns can still produce false translation. Robot-mounted/navigation acceptance is pending. |
| Experimental LiDAR mapping/localization | A2M12, `slam_toolbox`, candidate maps and isolated AMCL diagnostics | Candidate maps and diagnostic localization are not production navigation acceptance; extrinsic and integration gates remain open. |
| Navigation proxy simulation | ROS 2 Humble Nav2, MuJoCo proxy, map/path checks and static-route controller evaluation | Proxy motion is not evidence of gait performance or real-robot accuracy. |
| Dynamic interaction research | IMM/PPO experiments and a versioned MPPI interaction-input contract | G1 input contract has recorded acceptance; G2 critic build/load, IMM integration and policy-runtime gates remain incomplete. |

The real-camera experiments use a 640x360 stereo stream at 30 Hz. Recorded
good-image closed-loop bench trials of approximately 10 m ended with
0.011–0.036 m XY closure residual; this is not an absolute localization
accuracy measurement. The current-space emitter A/B evaluation selected
emitter OFF for cuVSLAM-only odometry, while EKF shadow fusion remains optional
and deferred. See the operating-envelope and emitter evaluation sections of
[`PROJECT_HANDOFF.md`](PROJECT_HANDOFF.md) for the environment, procedure,
evidence paths and limitations.

The handoff also records two static-route Nav2 MPPI proxy-simulation passes
on 2026-10-02 with no collisions, map violations or timeouts in those two
runs. These results do not establish dynamic-obstacle or hardware readiness.
Earlier PPO pilots did not establish successful navigation; their failure
and timeout trade-offs are retained in the research records.

For AI-assisted work, [`AGENTS.md`](AGENTS.md) is the repository-level operating
contract. It points to the mandatory engineering policy and the current handoff;
it does not replace either document.

Read [`PROJECT_HANDOFF.md`](PROJECT_HANDOFF.md) first, then use
[`docs/README.md`](docs/README.md) to enter the relevant workflow before
operating the hardware.

## Workstreams

The repository is intentionally split into independent workstreams:

- **Real cuVSLAM / D435i:** physical-camera input, IMU fusion, odometry,
  mapping, and localization readiness.
- **cuVSLAM simulation/replay:** ROS contracts, frame/timestamp checks, and
  controlled estimator experiments.
- **Path planning / navigation simulation:** D1 Edu, PPO/IMM/MPPI, obstacle
  simulation, and trajectory evaluation.
- **Research records:** dated experiment notes and navigation research handoffs.
- **Shared infrastructure:** Jetson connection, RViz/Foxglove monitoring,
  datasets, and project-level handoff documents.

See [`docs/PROJECT_CONTENT_MAP.md`](docs/PROJECT_CONTENT_MAP.md) for the full
classification and the boundary between the real-camera and navigation lines.

## Main entry points

- `Real D435i R1.5 Axis Test.command` — starts the local monitoring GUI.
- `shared/tools/jetson_connection_web.py` — shared Jetson connection GUI.
- `simulation/cuvslam/Foxglove Simulation.command` — simulation viewer launcher.
- `real_robot/cuvslam/tools/real_d435i_axis_web_gui.py` — GUI and R2
  measurement workflow.
- `real_robot/cuvslam/tools/run_real_d435i_r2.sh` — Jetson-side real R2
  launcher.
- `real_robot/cuvslam/tools/open_real_d435i_rviz_view.sh` — Jetson RViz/VNC
  viewer helper.
- `real_robot/cuvslam/deployment/isaac_ros/isaac_ros_visual_slam_d435i_real.launch.py` — real
  camera Isaac ROS launch configuration.

## Directory map

| Directory | Purpose |
|---|---|
| `real_robot/cuvslam/` | Physical D435i/cuVSLAM source, config, calibration, tools, and evidence |
| `simulation/cuvslam/` | cuVSLAM Gazebo/replay source, tools, and simulation evidence |
| `simulation/path_planning/` | D1 Edu models, maps, planning tools, and navigation experiments |
| `shared/` | Cross-workstream config, deployment helpers, and connection tools |
| `research/` | Dated research records; path-planning notes are under `research/path_planning/` |
| `docs/` | Handoffs, contracts, design decisions, and test plans |
| `datasets/`, `output/`, `tmp/` | Generated or source-attributed data; keep ownership explicit |

## Local setup

Copy `shared/config/local.env.example` to `shared/config/local.env` and set the
Jetson connection values. The local file is ignored by Git. The Jetson also
needs its own ignored `<Jetson Isaac ROS workspace>/.local.env` for the RViz
VNC password; use the deployment documentation for the current workspace path.
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

## Verification boundaries

Offline tests cover frame geometry, odometry validation, launch/configuration
contracts, navigation map/path utilities and interaction serialization. They
do not start sensors or prove a Jetson deployment works. Runtime launch,
topic/QoS/TF checks, replay and hardware acceptance require the corresponding
environment and evidence prescribed in `ENGINEERING_GUIDELINES.md`.

The 2026-10-03 publication-preparation checks reproduced a failing legacy
dynamic-crossing decision-layer test. Do not treat the complete research tree
as a fully passing navigation baseline. See
[`docs/GITHUB_UPDATE_PREPARATION_20261003.md`](docs/GITHUB_UPDATE_PREPARATION_20261003.md)
for the checks run and the pending publication step.

Raw recordings, high-volume traces, model archives, local credentials and
graduate-application artifacts are intentionally excluded from Git. Keep the
original evidence locally and publish the relevant summaries.
