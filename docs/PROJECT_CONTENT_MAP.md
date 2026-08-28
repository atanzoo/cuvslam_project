# Project content map

Updated: 2026-08-28 (Asia/Taipei)

This file classifies the repository by workstream. The current working paths
are intentionally kept stable: the macOS GUI, Jetson launchers, deployment
files, and historical reports refer to these paths directly. Classification is
therefore done by an explicit map and reading order instead of a mass move that
could break the running experiment workflow.

## A. Real cuVSLAM / D435i mainline

This is the primary workstream for the current project goal: run cuVSLAM with
the physical Intel RealSense D435i on Jetson and obtain reliable odometry,
mapping, and later localization.

- `calibration/` — camera, IMU, TF, timestamp, and hardware records.
- `config/real_d435i_mapping.rviz` — shareable RViz layout.
- `docs/REAL_D435I_MAPPING_HANDOFF_20260817.md` — real-camera procedure and
  operating boundaries.
- `docs/CUVSLAM_CURRENT_STATUS_20260825.md` — current real-camera status and
  next-test entry point.
- `docs/sensor_and_frame_contract.md` — input, frame, TF, and CameraInfo
  contract.
- `deployment/isaac_ros/isaac_ros_visual_slam_d435i_real.launch.py` — real
  camera Isaac ROS launch boundary.
- `tools/real_d435i_*` — Jetson launch, inspection, recording, GUI, and
  analysis tools.
- `tools/collect_real_d435i_*`, `tools/inspect_real_d435i_*`,
  `tools/measure_real_d435i_*`, `tools/run_real_d435i_*` — repeatable real
  hardware checks.
- `logs/real_d435i_quality/` and `reports/real_d435i_*` — real-camera
  evidence, correction records, and acceptance reports.

Current baseline: official `640x360x30`, IMU fusion enabled, denoise enabled,
auto exposure enabled, emitter enabled, and mapping enabled. Good-image
conditions have passed the current engineering gate. A special-scene turn can
still produce false translation, so that limitation remains part of the real
camera handoff.

## B. cuVSLAM simulation and replay support

This workstream is used to validate ROS contracts, frame conventions,
timestamping, observability, and estimator behavior before or alongside real
hardware work. It is not a substitute for real D435i accuracy.

- `deployment/isaac_ros/` — Isaac ROS container and launch support, including
  simulation/replay adapters.
- `deployment/slam_gazebo/` — Gazebo simulation boundary.
- `tools/simulation_*`, `tools/validate_*`, `tools/analyze_*`, and
  `tools/evaluate_*` except the `d1` tools — simulation/replay diagnostics.
- `docs/SIMULATION_HANDOFF.md`, `docs/CUVSLAM_*`, `docs/FRAME_AWARE_*`,
  `docs/STATIC_TF_*` — simulation, frame, and estimator design records.
- `reports/cuvslam_*`, `reports/nvidia_*`, `reports/stage*`,
  `reports/frame_*`, `reports/tf_*`, and `reports/static_tf_*` — simulation
  and NVIDIA reference evidence.

Simulation truth, wheel odometry, and world overlays are valid for controlled
comparisons only. They must not be quoted as physical-camera measurements.

## C. Path planning and navigation simulation

This is the independent D1 Edu / navigation research workstream requested for
path planning and navigation simulation. It is not yet the real D435i mapping
or Nav2 integration path.

- `models/d1_edu/` — D1 Edu robot description and meshes.
- `tools/d1_edu_*`, `tools/run_d1_*`, `tools/generate_d1_*`,
  `tools/evaluate_d1_*`, `tools/train_d1_*`, `tools/trace_d1_*` — D1
  simulation, trajectory generation, training, and evaluation.
- `tools/mac_d1_monitor.py` — macOS-side D1 monitoring helper.
- `tools/build_mppi_annotation.py` and
  `tools/generate_sb3_ppo_trajectory_data.py` — path-planning/training data
  preparation.
- `docs/RESEARCH_PROGRESS_2026-08-06.md`,
  `docs/PPO_BALANCED_SAFE_300K_SUMMARY.md`, and
  `docs/IMM_TRAINING_HANDOFF_2026-08-07.md` — PPO, IMM, MPPI, obstacle, and
  navigation research notes.
- `external/`, `output/`, `research/papers/`, `tmp/`, and the whole-body
  research artifacts — supporting or generated material; most are ignored by
  Git because they are large or environment-specific.

The future integration boundary is:

```text
cuVSLAM visual localization / odometry
        -> approved pose and TF interface
        -> Nav2 localization and path planning
        -> simulation gate
        -> real navigation gate
```

Do not treat D1 training success as proof that the real cuVSLAM localization
interface is ready.

## D. Shared infrastructure and evidence

- `PROJECT_HANDOFF.md` — complete project handoff with current status first
  and historical simulation record retained below it.
- `ENGINEERING_GUIDELINES.md` — repository operating rules.
- `docs/README.md` — document reading index.
- `docs/GITHUB_REPOSITORY_GUIDE.md` — GitHub upload and credential policy.
- `logs/`, `reports/`, `datasets/`, and `evaluation/` — evidence and analysis
  shared by more than one workstream.
- `tools/jetson_connection_*`, `Jetson Connection.command`, and
  `tools/Foxglove Simulation.command` — connection and visualization helpers.
- `deployment/foxglove/` — Foxglove visualization support.

Runtime recordings and generated outputs stay local by default. The GitHub
repository keeps source, configuration examples, curated reports, and the
handoff documents; credentials, raw bags, large models, and high-volume traces
are excluded by `.gitignore`.

## Recommended reading order for the next conversation

1. `PROJECT_HANDOFF.md`
2. `docs/CUVSLAM_CURRENT_STATUS_20260825.md`
3. `docs/PROJECT_CONTENT_MAP.md` (this file)
4. `docs/REAL_D435I_MAPPING_HANDOFF_20260817.md`
5. `docs/GITHUB_REPOSITORY_GUIDE.md`

If the next task is path planning or navigation simulation, read the D1 files
under section C after the cuVSLAM status is understood.
