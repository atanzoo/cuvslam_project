# cuVSLAM Straight Mapping Experiment Design

Date: 2026-07-29
Status: Experiment completed; mapping-quality gate failed
Change class: C3 (mapping experiment)

## Objective

Measure whether the corrected simulated stereo pipeline produces a stable,
approximately scaled local visual map during straight-only motion.

Non-goal: this experiment does not validate turning, loop closure, complete
SLAM, navigation, map persistence, IMU fusion, or real D435i behavior.

Environment: Jetson simulation, ROS 2 Humble domain 43, cuVSLAM 11.4
stereo-only, corrected right CameraInfo baseline.

## Baseline

The corrected short straight run measured:

- truth 3D displacement: `0.2270 m`;
- cuVSLAM 3D displacement: `0.2792 m`;
- cuVSLAM 2D displacement: `0.2721 m`;
- Z change: `-0.0626 m`;
- false rotation: `6.75 deg`;
- all status samples: `vo_state=1`.

The turn-quality gate remains failed and is outside this experiment.

## Option A: Enable Sparse Mapping Visualization (Selected)

Keep `enable_localization_n_mapping=true`, enable SLAM visualization and
landmark output, and record the sparse landmark cloud with the trajectory.
Observations and other high-volume debug views remain disabled.

Advantages:

- verifies that mapping output is produced, not only odometry;
- allows Foxglove comparison with the known-world outline;
- keeps the added GPU and network load smaller than full debug visualization.

Disadvantages:

- changes runtime load relative to the odometry baseline;
- sparse landmarks are not a dense surface map;
- map geometry cannot be accepted while pose accuracy is poor.

Failure modes:

- visualization load causes frame drops;
- landmark messages remain empty;
- a visually plausible map hides trajectory scale or Z errors.

Rollback: restore the two visualization parameters to `false`, redeploy the
launch file, and cleanly restart the GUI simulation.

## Option B: Record Trajectory Only (Rejected)

Leave all map visualization disabled and infer mapping health from
`slam_path`.

This has lower runtime cost but cannot demonstrate that a visual map is
actually populated, so it does not satisfy the requested straight-mapping
trial.

## Motion And Safety

Runs use only `/scan` and `/visual_slam/status` for online control. Gazebo
ground truth is recorded and evaluated only after motion stops.

Each distance starts from a clean Gazebo and cuVSLAM initialization:

| Stage | Command | Expected truth range |
|---|---|---|
| Short | `0.10 m/s` for `3 s` | about `0.23 m` |
| Medium | `0.10 m/s` for `6 s` | about `0.5 m` |
| Long | `0.10 m/s` for `11 s` | about `1.0 m` |
| Extended | `0.10 m/s` for `21 s` | about `2.0 m`, only after earlier gates |

The controller publishes zero velocity on completion, tracking loss, missing
scan, or obstacle stop. A separate zero command is sent after every run.

## Recorded Evidence

- `/cmd_vel`;
- `/scan`;
- `/ground_truth/odom`;
- cuVSLAM status, odometry, VO pose, and SLAM path;
- `/tf` and `/tf_static`;
- raw and normalized right CameraInfo;
- `/visual_slam/vis/landmarks_cloud`;
- pose graph nodes and edges when available.

## Acceptance Signals

This exploratory stage reports, rather than assumes:

1. tracking-state counts and controller completion;
2. truth versus cuVSLAM XY and 3D displacement;
3. Z and rotation error versus distance;
4. whether error is fixed-offset or proportional;
5. landmark message count, finite point count, and XYZ bounds;
6. dropped-frame or runtime warning evidence;
7. visual alignment against the ground-truth world outline.

Straight mapping is not accepted as complete SLAM even if all straight stages
pass.

## Approval

The user approved this bounded experiment on 2026-07-29 with
`處理轉向問題前，我想先試試直行建圖`.

## Result

The short and medium stages completed with `vo_state=1` and populated sparse
landmark clouds. The 1 m and 2 m stages were intentionally skipped after the
medium quality gate failed.

| Stage | Truth 3D | cuVSLAM 3D | Z | Rotation | Final landmarks |
|---|---:|---:|---:|---:|---:|
| Short | `0.2475 m` | `0.3828 m` | `-0.0018 m` | `5.66 deg` | `583` |
| Medium | `0.4375 m` | `0.2524 m` | `-0.1225 m` | `13.80 deg` | `460` |

The short run overestimated distance by about 55 percent. The medium run
underestimated it by about 42 percent. This sign change rules out a stable,
single proportional scale correction.

Both final landmark clouds extended beyond the 6 m by 6 m arena:

- short XYZ bounds: X `[0.7626, 10.8520]`, Y `[0.4102, 8.2194]`,
  Z `[-1.2576, 1.4645]`;
- medium XYZ bounds: X `[0.7394, 10.7774]`, Y `[0.4102, 8.7814]`,
  Z `[-1.5921, 1.4645]`.

No tracking-loss, frame-drop, or image-jitter warning was recorded. The
mapping pipeline is functional but the generated geometry is not accepted.
Sparse visualization was rolled back to its lower-load disabled state after
the experiment.
