# NVIDIA Reference and Gazebo 1 m Debug Test

Date: 2026-07-30
Environment: Jetson AGX Orin, ROS 2 Humble container, ROS domain 43
Estimator: Isaac ROS Visual SLAM 2.1.0, cuVSLAM 11.4
Final runtime state: Simulation stopped

## Scope

This diagnostic did not change TF, CameraInfo, cuVSLAM frame ownership, or
estimator parameters. It:

1. replayed NVIDIA release-2.1 `small_pol_test` through the installed
   VisualSlamNode; and
2. ran the level-chassis Gazebo world to a native-truth distance of at least
   one metre with the NVIDIA debug dump enabled.

Debug mode is an opt-in launch argument and remains disabled by default.

## NVIDIA Control

The Git LFS objects were verified against NVIDIA's release-2.1 pointers:

```text
metadata.yaml:
35064a4b0c76994035c4f93bdc35886a24798d99b0c0919219f05a6605c8159e

small_pol_test_0.db3:
7dfc81183a61f6e13148d4440e06ba20f104c110a38847ed6fa11b090599bcf0
```

The 12.9-second bag contained 725 left images and 726 right images. The
installed node produced:

| Metric | Result |
|---|---:|
| Odometry samples | 725 |
| Status samples | 725 |
| Output frame | `odom -> base_link` |
| `vo_state=1` | 725 / 725 |
| Endpoint displacement | 0.2060 m |
| Path length | 0.3786 m |
| Endpoint rotation | 93.34 deg |

The official bag has no ground truth, so these motion values are not an
accuracy result. The control passes NVIDIA's release-2.1 package test criterion
of producing odometry and proves that the installed binary, GPU path, and ROS
wrapper can process NVIDIA's own stereo data.

## Gazebo 1 m Result

World: `cuvslam_mapping_simple.sdf`

Motion controller:

- target truth distance: 1.0 m;
- speed: 0.12 m/s;
- LiDAR stop distance: 0.35 m;
- abort on any `vo_state != 1`;
- ground truth controlled only the test endpoint and was not an estimator
  input.

| Metric | Native 6DoF truth | cuVSLAM |
|---|---:|---:|
| Translation | 1.0176 m | 0.1914 m |
| Rotation | 0.00 deg | 5.05 deg |
| Translation error | - | 81.2% low |
| Tracking | - | `vo_state=1` throughout |

The one-metre result confirms severe visual-odometry underestimation. It is not
explained by the previously corrected chassis pitch or by a tracking-state
failure.

## NVIDIA Debug Dump

The dump was complete:

```text
left images:  4186
right images: 4186
metadata:     4186 lines
size:         67 MB
```

Every sampled pair had identical left/right timestamps. Native model motion
started at `111.875 s` and ended at `137.720 s`.

Representative left-image hashes:

| Frame | Sim time | SHA-256 |
|---:|---:|---|
| 2450 | 110.950 s | `382ec528...ccc9e` |
| 2500 | 112.700 s | `85e283a3...586e` |
| 2750 | 121.045 s | `115e3c2e...45b1` |
| 3000 | 129.295 s | `115e3c2e...45b1` |
| 3250 | 137.545 s | `115e3c2e...45b1` |
| 3300 | 139.195 s | `115e3c2e...45b1` |

The image changes shortly after motion starts, then becomes byte-identical from
at least `121.045 s` through the end of physical motion at `137.720 s`.
Timestamps continue advancing. The representative frames also show a
low-texture grey scene with only a few hard-edged landmarks.

This was checked across 501 consecutive pairs, frames 2750 through 3250:

```text
unique left-image SHA-256 values:  1
unique right-image SHA-256 values: 1
```

This establishes that cuVSLAM's observed motion stops because its dumped image
content stops changing. It does not yet identify whether the freeze originates
in the Gazebo camera sensor, the Gazebo-to-ROS bridge, or NVIDIA's debug-dump
path.

## Decision

Accepted:

- installed cuVSLAM processes NVIDIA's official stereo control;
- debug mode remains opt-in;
- the one-metre underestimation is reproduced quantitatively;
- image-content freeze is now a proven upstream symptom.

Not accepted:

- tuning or scaling cuVSLAM odometry;
- changing TF or CameraInfo;
- treating `vo_state=1` as an accuracy gate;
- claiming the Gazebo camera link is detached based on relative link poses.

The next minimum experiment must record the same moving interval at three
boundaries:

1. Gazebo Transport camera payload;
2. bridged ROS `/d435i/infra1/image_rect_raw` and right image;
3. NVIDIA debug dump.

Per-frame content hashes at those boundaries will locate the first component
that repeats stale pixels.

## Evidence

- `docs/CUVSLAM_NVIDIA_REFERENCE_DEBUG_DESIGN.md`
- `tools/evaluate_odometry_bag.py`
- `tools/simulation_truth_distance_profile.py`
- `tools/gz_motion_window.py`
- `reports/nvidia_reference_debug_20260730/cam0.02450.png`
- `reports/nvidia_reference_debug_20260730/cam0.02500.png`
- `reports/nvidia_reference_debug_20260730/cam0.02750.png`
- Jetson:
  `/home/tseng/isaac_ros_ws/data/nvidia_reference/control_output/`
- Jetson:
  `/home/tseng/isaac_ros_ws/data/nvidia_reference/gazebo_1m_debug/`

## Verification and Shutdown

```text
Launch contract tests: 5 passed
Gazebo truth/parser math tests: 5 passed
cuvslam-nvidia-truth.service: inactive
cuvslam-d435i-bridge.service: inactive
cuvslam-d435i-gazebo.service: inactive
cuVSLAM and rosbag recorder processes: none
/clock: no message during a 3-second sample
```
