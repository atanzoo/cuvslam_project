# cuVSLAM stereo synchronization / yaw correction audit — 2026-08-03

## Scope

This audit addresses two suspected causes of the corner-map misalignment:

1. stale or poorly synchronized left/right stereo frames;
2. failure of keyframe/PoseGraph updates to correct the turn yaw.

Wheel odometry was not used as an input, controller signal, or ground truth.
Gazebo native pose was used only for offline comparison.

## Stereo checks and correction

The cuVSLAM input calibration was checked from the recorded topics:

- image size: `640 x 360`;
- adapted right `CameraInfo.P[3]`: `-15.723149...`;
- projected stereo baseline: `0.050 m`;
- nearest left/right timestamp skew: mean `0.039 ms`, maximum `35 ms`;
- only one pair exceeded `5 ms` (the same pair also exceeded `10 ms` and
  `20 ms`).

The raw Gazebo right `CameraInfo` has `P[3] = 0`, which is expected before the
adapter. The cuVSLAM-facing topic is `/cuvslam/input/infra2/camera_info`, and
its baseline is correct. Therefore the baseline was not changed.

The launch input gate was initially changed from:

```text
img_jitter_threshold_ms = 50.0
```

to `5 ms`, but the first closed-loop runtime log showed that this parameter
was being interpreted as the interval between consecutive synchronized image
messages: normal 30--35 ms frame intervals were reported as above threshold.
NVIDIA's official parameter documentation defines the parameter that way and
gives `34.0 ms` as the default. The final backend run exposed a separate
runtime bottleneck: cuVSLAM output was only about `13.5 Hz`, with gaps of
`65--70 ms` and occasional gaps over one second. Low-volume backend trials
therefore disable expensive landmark/observation visualization, while the
GUI visualization mode remains enabled. The simulation setting is:

```text
img_jitter_threshold_ms = 40.0
```

This remains compatible with the nominal 30 Hz camera cadence. The left/right
pair skew is still audited separately from recorded CameraInfo/image
timestamps; it must not be forced through this consecutive-frame parameter.
Low-volume backend trials use
`CUVSLAM_ENABLE_SLAM_VISUALIZATION=false` so visualization overhead is not
mistaken for a SLAM input failure.

An ORB-based live audit also found abundant positive-disparity matches (about
`0.97` or higher positive-disparity fraction in the turn sample). Its vertical
residual had a few outliers, so it is useful as an outlier alarm but is not a
direct replacement for cuVSLAM's internal LK/NCC tracker.

## Keyframe / PoseGraph result

The low-volume recorder now stores:

- `/visual_slam/vis/pose_graph_nodes`;
- `/visual_slam/vis/pose_graph_edges`;
- `/visual_slam/tracking/slam_path`;
- VO, status, native pose, TF, CameraInfo and IMU.

In the final post-fix corner run, the route completed normally:

- first leg: `1.0005 m`;
- turn: `90.04 deg` from IMU/native control;
- second leg: `0.5019 m`;
- tracking: `vo_state=1` throughout.

PoseGraph activity during the same run:

| Phase | Node count | Added nodes | Edge count trend |
|---|---:|---:|---:|
| First leg | `2 -> 26` | `24` | `2 -> 48` |
| Turn | `26 -> 64` | `38` | `48 -> 122` |
| Second leg | `65 -> 83` | `18` | `124 -> 160` |

The turn therefore did generate keyframe/PoseGraph updates. It is not a
keyframe-free interval and does not violate a fixed NVIDIA rule such as
"90 degrees must create a keyframe"; NVIDIA's keyframe selection is driven by
tracking quality and graph conditions, not by a fixed turn angle.

## Yaw A/B result

### Pure in-place turn

The isolated 90-degree turn at `20 deg/s` produced approximately:

- truth yaw: `88.30 deg`;
- estimated yaw: `87.09 deg`;
- yaw error: `-1.21 deg`;
- translation residual: `3.07 cm`.

This indicates that the turn controller and the visual pose estimate can work
well when the scene/viewpoint is held constant.

### Full open corner after the 5 ms gate

The final `1 m -> 90 deg -> 0.5 m` run produced:

- first-leg yaw error: `-0.91 deg`;
- turn yaw error: `-8.31 deg`;
- turn translation residual: `3.30 cm`;
- second-leg yaw error: `+0.66 deg`;
- complete-route yaw error: `-8.62 deg`.

`slam_path` and VO were identical in this run. The open L-shaped route does
not revisit a previously observed place, so no loop closure was available to
provide a global yaw correction. PoseGraph node/edge growth alone is not the
same as a loop-closure correction.

## Conclusion

The camera baseline and stereo audit are valid, but the temporary 5 ms
`image_jitter_threshold_ms` change was invalid. The final simulator setting is
40 ms, based on the official parameter semantics and measured 35 ms cadence.
The first closed-loop run made with 5 ms is not accepted as a cuVSLAM A/B
result. The keyframe hypothesis is also not supported:
keyframes and graph edges were actively created during the turn.

The remaining issue is scene-conditioned frontend/relative-pose quality and
runtime frame drops. A subsequent closed-loop experiment, including a
visual-only start/end anchor and a 15-second post-revisit wait, still produced
identical VO and `/visual_slam/tracking/slam_path`; see
`reports/cuvslam_loop_backend_20260803.md`. Do not force keyframes or inject
wheel odometry, because that would hide the actual visual-SLAM behavior.

## Reproducibility / cleanup

The runner now forwards `CUVSLAM_POSE_ONLY`, `CUVSLAM_STOP_DISTANCE`, and the
active world SDF, and pose-only bags include the PoseGraph and SLAM path
topics. After the audit, the three intermediate audit bags were removed from
Jetson. The baseline bag and final post-fix bag were retained for traceability:

- `gui_native_corner_20260803_110534` — baseline;
- `sync5_corner_20260803c_20260731_289993` — final post-fix corner run.

## References

- [cuVSLAM paper](https://arxiv.org/abs/2506.04359)
- [NVIDIA Isaac ROS cuVSLAM documentation](https://nvidia-isaac-ros.github.io/concepts/visual_slam/cuvslam/index.html)
- [cuVSLAM API](https://nvidia-isaac.github.io/cuVSLAM/python/api.html)
- [NVIDIA cuVSLAM repository](https://github.com/nvidia-isaac/cuVSLAM)
