# cuVSLAM Observable-World Repeatability Gate

Date: 2026-07-31 (Asia/Taipei)  
Environment: Jetson AGX Orin, ROS 2 Humble container, ROS domain 43  
Estimator: Isaac ROS Visual SLAM 2.1.0 / cuVSLAM 11.4  
IMU fusion: disabled

## Result

The observable simulation world was expanded from the initial straight-route
diagnostic to the complete square mapping route. Every route leg now has
asymmetric, high-contrast, near-field visual features on both sides, and four
visual-only floor markers provide additional depth variation.

Two independent cold-start 1 m trials passed the repeatability gate. The
simulation mapping gate is therefore open.

## World contract

World:

```text
deployment/slam_gazebo/worlds/cuvslam_mapping_observable.sdf
```

The observable world contains 30 added visual elements. They do not add
collision geometry and do not change camera geometry, TF, CameraInfo, timing,
robot dynamics, truth topics, IMU settings, or cuVSLAM parameters.

```text
baseline SHA-256:
de9fe0515ae0142ab9d3cb54f9b0af16b96e0776703c26a333431cfc1d39791f

route-wide observable SHA-256:
9a6e937c027779c199618ea3a7d39883439c53d636cc2fc58c9643acb794d66f

Jetson ign sdf -k:
Valid.
```

## Repeatability results

| Trial | Native truth | cuVSLAM translation | Rotation | 3D residual | Tracking |
|---|---:|---:|---:|---:|---:|
| Repeat 1 | 1.0176 m | 1.0189 m | 0.78 deg | 0.0504 m | 478 / 478 state 1 |
| Repeat 2 | 1.0152 m | 0.9708 m | 1.32 deg | 0.0749 m | 449 / 449 state 1 |

Repeat 1 inferred base translation:

```text
(+1.0176, +0.0503, +0.0026) m
```

Repeat 2 inferred base translation:

```text
(+0.9690, +0.0543, +0.0229) m
```

Jetson bundles:

```text
/home/tseng/isaac_ros_ws/data/acceptance_1m_observable_repeat1_20260731_24013
/home/tseng/isaac_ros_ws/data/acceptance_1m_observable_repeat2_20260731_25530
```

## Image and timing checks

Both trials recorded 258 ROS left frames and 258 ROS right frames during
motion. Every frame was unique and all 258 stereo timestamps matched exactly.
The median frame interval was 35 ms.

NVIDIA debug images were also unique:

- repeat 1: 257 left and 257 right;
- repeat 2: 253 left and 253 right.

There is no evidence of in-motion stale pixels.

## Acceptance decision

Both trials satisfy:

- native truth translation between 0.95 and 1.05 m;
- cuVSLAM translation between 0.95 and 1.05 m;
- orientation error at most 15 degrees;
- exact stereo timestamp pairing;
- no repeated images during native motion;
- `vo_state=1` throughout;
- zero velocity and complete workload cleanup after each trial.

The straight-motion repeatability gate passes. Mapping should now proceed in
stages:

1. straight segment with trajectory and landmark-quality checks;
2. one controlled 90-degree turn;
3. only after the turn passes, the complete square route.

This result does not yet prove turn accuracy, loop closure, map persistence,
relocalization, or real-camera performance.

