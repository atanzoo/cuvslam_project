# Baseline Square Mapping Test

Date: 2026-07-28 (Asia/Taipei)

## Scope

- Environment: Gazebo Baseline simulation only.
- Jetson: AGX Orin, ROS domain 43.
- Estimator: cuVSLAM 11.4, stereo-only, mapping enabled.
- Formal route owner: `cuvslam_sim_estimator_profile`.
- No real-robot, Nav2, motor, or production workspace was used.

## Formal Route

The accepted architecture uses a time-based low-speed command profile. The
controller subscribes only to:

- `/visual_slam/status` for tracking-loss stop;
- `/scan` for stale-data and obstacle-clearance stop.

It publishes only `/cmd_vel`. Runtime ROS graph inspection confirmed that it
had no ground-truth or odometry subscription.

Formal command limits were:

- linear velocity: `0.18 m/s`;
- angular velocity: `-0.25 rad/s`;
- four `6.0 s` straight phases;
- four `6.283 s` clockwise turn phases;
- stop distance: `0.35 m`;
- immediate zero velocity on tracking loss, stale scan, insufficient
  clearance, interruption, or normal completion.

Ground truth was recorded but not read until the controller and recorder had
stopped.

## Rejected Diagnostic Route

The earlier diagnostic run used four truth-odometry-controlled waypoints
forming a local
`1.7 x 1.7 m` square. Command limits were:

- linear velocity: `0.20 m/s`;
- angular velocity: `0.30 rad/s`;
- safety boundary: `abs(x), abs(y) <= 3.35 m`;
- immediate zero velocity on tracking loss, timeout, interruption, or boundary
  violation.

This did not match the intended real architecture. Its result is retained only
as historical diagnostic evidence, and the controller was removed.

## Experiments

### Open-loop rejection

Path:

```text
/home/tseng/isaac_ros_ws/data/experiments/baseline_square_20260728_1559
```

A time-based `2.0 m` square completed without declared tracking loss, but the
truth closure error was `1.024 m`. This control method was rejected.

### Waypoint-controlled run

Path:

```text
/home/tseng/isaac_ros_ws/data/experiments/baseline_square_closed_20260728_1603
```

Evidence:

- duration: `65.230 s`;
- bag size: `572.2 MiB`;
- messages: `32,033`;
- truth closure: `0.0351 m`;
- cuVSLAM tracking odometry closure: `3.2724 m`;
- cuVSLAM VO pose closure: `3.2724 m`;
- cuVSLAM SLAM path closure: `2.9939 m`;
- status samples: `vo_state=1` for all `1,556` messages;
- final command: zero linear and angular velocity;
- final `/cmd_vel` publisher count: zero.

### Formal estimator-only run

Path:

```text
/home/tseng/isaac_ros_ws/data/experiments/baseline_estimator_only_20260728_1646
```

Evidence:

- clean full simulation restart before recording;
- duration: `56.632 s`;
- bag size: `451.3 MiB`;
- messages: `27,219`;
- controller subscribers: `/scan`, `/visual_slam/status`;
- controller publisher: `/cmd_vel`;
- minimum reported scan clearance: approximately `0.74 m`;
- truth endpoint displacement: `0.6123 m`;
- cuVSLAM tracking odometry endpoint displacement: `8.8072 m`;
- cuVSLAM VO pose endpoint displacement: `8.8077 m`;
- cuVSLAM SLAM path endpoint displacement: `8.8077 m`;
- status samples: `vo_state=1` for all `1,342` messages;
- `/visual_slam/tracking/odometry` publisher count: one;
- odometry owner: `/visual_slam_node`;
- final command: zero linear and angular velocity;
- final `/cmd_vel` publisher count: zero.

## Result

The estimator-only architecture, simulation control, recording, and
process-health portions passed. The cuVSLAM trajectory-quality gate failed.
Continuous `vo_state=1` did not imply an accurate trajectory.

Do not advance to a more complex world, IMU fusion, Nav2, or occupancy mapping
from this result. The next investigation should isolate:

1. stereo camera intrinsics and baseline;
2. left-to-right optical-frame transform direction and units;
3. image timestamp synchronization;
4. simulated image texture, motion blur, and feature coverage;
5. cuVSLAM frame conventions and trajectory output semantics.

## Files

- `tools/simulation_estimator_profile.py`
- `tools/evaluate_simulation_bag.py`
- `docs/CUVSLAM_ESTIMATOR_ONLY_LEVEL0_DESIGN.md`
- `reports/baseline_square_mapping_20260728.md`

Stopping either tool removes its `/cmd_vel` publisher. The tools are
simulation-only and are not part of any systemd service or real-robot launch.
