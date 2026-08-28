# cuVSLAM Simulation Camera Frame Fix Design

Date: 2026-07-29
Status: Option B implemented; calibration passed, trajectory gate partial
Change class: C2 (frame and launch contract)

## Objective

Make planar simulated D435i motion remain planar in cuVSLAM by supplying the
camera extrinsics in the physical ROS camera-frame convention expected by the
installed Isaac ROS Visual SLAM 11.4 implementation.

Non-goal: this change does not enable IMU fusion, tune cuVSLAM, alter Gazebo
motion, use ground truth as estimator input, or modify a real-robot workspace.

Affected environment: Jetson simulation, ROS 2 Humble, ROS domain 43,
`isaac_ros_dev-aarch64-container`.

## Evidence And Root Cause

The clean isolation run produced:

- static: zero truth motion and zero cuVSLAM motion;
- straight: truth `0.2355 m`, cuVSLAM `0.3779 m`, false `+0.1112 m` Z and
  `5.59 deg` rotation;
- turn: truth `37.70 deg` with zero translation, cuVSLAM `12.47 deg` with
  false `1.2963 m` translation and `+0.1578 m` Z.

Stereo timestamps were exactly synchronized and the first image pair had a
horizontal shift of `+7 px` with zero vertical shift. The current right
`CameraInfo.P[3]` is zero, while the baseline is supplied by TF.

The installed node source applies its ROS-to-cuVSLAM canonical basis conversion
to the TF between `input_left_camera_frame` and
`input_right_camera_frame`. Its bundled RealSense example supplies
`camera_*_infra1_frame` and `camera_*_infra2_frame`, not optical frames. The
project launch supplied optical frames directly, causing the stereo extrinsic
to be interpreted in the wrong basis.

## Option A: Explicit Physical And Optical Frames (Tested And Rejected)

Publish a simulation-only static tree:

```text
base_link
  -> camera_infra1_frame
    -> camera_infra1_optical_frame
    -> camera_infra2_frame
      -> camera_infra2_optical_frame
```

Configure cuVSLAM with `base_link`, `camera_infra1_frame`, and
`camera_infra2_frame`. Keep image and CameraInfo frame IDs optical.

Advantages:

- matches the installed cuVSLAM source and bundled RealSense example;
- preserves the sensor message contract;
- exposes the physical-to-optical conversion explicitly;
- gives the output TF the project-standard `base_link` child.

Disadvantages:

- adds three simulation-only static TF publishers;
- changes the cuVSLAM output child frame from the left optical frame to
  `base_link`;
- requires Foxglove and world-outline regression checks.

Failure modes:

- wrong optical quaternion rotates all displayed sensor data;
- wrong stereo translation sign produces invalid depth scale;
- duplicate TF ownership would make the tree nondeterministic.

Rollback: restore the previous launch file and restart the simulation. Recorded
bags remain unchanged.

## Option B: Encode Baseline In CameraInfo (Selected)

Leave the input camera-frame parameters empty and set the right projection
matrix translation to `P[3] = -fx * baseline`.

Advantages:

- uses the node's CameraInfo fallback;
- requires fewer TF parameters.

Disadvantages:

- changes the simulated sensor normalization layer;
- duplicates geometry already present in the SDF;
- does not establish the physical-to-optical robot frame tree;
- creates a greater risk of TF and CameraInfo extrinsics diverging.

Rollback and deployment are similar to Option A, but verification must also
prove every CameraInfo sample has the corrected projection matrix.

Implementation uses a dedicated simulation-only
`cuvslam_sim_sensor_adapter` package in the Isaac ROS workspace:

- owner: simulated sensor-normalization layer;
- input: `/d435i/infra2/camera_info`, Reliable/Volatile;
- output: `/cuvslam/input/infra2/camera_info`, Reliable/Volatile;
- transformation: preserve every field except
  `P[3] = -P[0] * stereo_baseline_m`;
- default `stereo_baseline_m`: `0.05`;
- dependencies: `rclpy`, `sensor_msgs`;
- deployment target: Jetson Isaac ROS simulation workspace only;
- failure behavior: no corrected output when focal length or baseline is
  invalid, so cuVSLAM readiness fails instead of using an unvalidated model.

The cuVSLAM launch leaves both input camera-frame parameters empty and removes
the explicit stereo-baseline TF publisher. cuVSLAM therefore uses its
CameraInfo fallback as the sole stereo-extrinsic authority. Image topics,
timestamps, encodings, IMU, LiDAR, truth, and Gazebo motion are unchanged.

This package is separate from `cuvslam_sim_observability` because calibration
normalization and ground-truth visualization have independent ownership,
failure modes, and future replacement paths.

## Compatibility And Ownership

- Gazebo remains the owner of simulated image generation and motion.
- cuVSLAM remains the only owner of `map -> odom` and `odom -> base_link`.
- Static launch publishers own the four camera rig edges.
- Ground truth remains recording and visualization only.
- No topic name, QoS, image timestamp, image encoding, or real-robot contract
  changes.

## Verification Plan

1. Compile-check the launch file.
2. Cleanly stop and start the simulation.
3. Verify each camera TF edge and confirm one publisher per edge.
4. Confirm D435i 5/5 topics, cuVSLAM `vo_state=1`, world markers, and Foxglove.
5. Record the same static, straight, and in-place-turn profiles.
6. Compare truth and cuVSLAM 2D/3D displacement, delta Z, and rotation.
7. Stop motion immediately after every profile.

Acceptance requires no meaningful static drift and a substantial reduction in
false Z/roll/pitch and turn-induced translation. A tracking-only pass is not an
accuracy pass.

## Approval

The user approved beginning this frame-alignment correction in the task on
2026-07-29 with `OK` followed by `開始`.

After reviewing the Option A failure and the zero right projection baseline,
the user explicitly selected Option B on 2026-07-29 with
`先修正右相機就好，並向我回報結果`.

## Option B Test Result

The adapter built successfully and its five calibration unit tests passed.
Runtime evidence confirmed:

- raw right CameraInfo remained `P[3]=0`;
- normalized right CameraInfo published `P[3]=-10.416588`;
- the normalized topic had one publisher and one cuVSLAM subscriber;
- cuVSLAM logged `left_pose_right reading from CameraInfo` and
  `Baseline is : 0.050000`;
- D435i 5/5 topics, world outline, Foxglove, and `vo_state=1` remained ready.

Trajectory results were mixed:

| Profile | Truth | cuVSLAM |
|---|---|---|
| Static | zero motion | zero motion |
| Straight | `0.2270 m`, Z 0, rotation 0 | `0.2792 m`, Z `-0.0626 m`, rotation `6.75 deg` |
| Turn | `36.73 deg`, zero translation | `9.04 deg`, `1.1374 m` translation, Z `-0.1974 m` |

Compared with the original TF-baseline run, straight-distance overestimation
dropped from about 60 percent to about 23 percent and absolute straight Z
error dropped from `0.1112 m` to `0.0626 m`. The turn gate still failed:
translation improved only from `1.2963 m` to `1.1374 m`, while absolute Z
error increased from `0.1578 m` to `0.1974 m` and rotation was further
underestimated.

Option B is retained as the correct simulation CameraInfo contract, but it is
not evidence that cuVSLAM trajectory quality is acceptable.

## Option A Test Result

The implementation compiled, started cleanly, reached `vo_state=1`, preserved
all D435i topics and world markers, and produced the intended TF tree. It failed
the first motion-quality gate:

- before, straight truth `0.2355 m` versus cuVSLAM `0.3779 m`, false Z
  `+0.1112 m`, false rotation `5.59 deg`;
- after Option A, straight truth `0.2265 m` versus cuVSLAM `0.5792 m`, false Z
  `-0.1567 m`, false rotation `11.79 deg`.

The turn test was intentionally not run after the failed straight gate. The
controller sent zero velocity and the launch was rolled back. Option A is not
approved for reuse without new evidence.
