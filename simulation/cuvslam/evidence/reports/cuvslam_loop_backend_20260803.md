# cuVSLAM closed-loop backend audit — 2026-08-03

## Objective

Add a native-pose-controlled closed route so cuVSLAM mapping can revisit the
start and give the backend a real opportunity to apply loop closure. The route
controller uses only Gazebo native pose for distance termination and D435i gyro
integration for turn termination. Wheel odometry is not used.

## Implementation

Added:

- `tools/simulation_native_loop_profile.py`: four linear legs and four
  timestamped 90-degree IMU-gated turns;
- `native_loop` branch in the container trial runner;
- configurable loop leg lengths;
- low-volume recording of VO, SLAM path, PoseGraph nodes/edges, status, truth,
  CameraInfo, TF and IMU;
- loop-closure analyzer comparing native truth, VO and the final SLAM path;
- GUI route changed from the open L path to the closed route;
- a visual-only, no-collision start/end anchor in the dense-turn SDF.

The anchor is only a simulation feature for place recognition. It does not
change collision geometry, LiDAR clearance, or the cuVSLAM input data flow.

## Valid small-loop result

The final anchor run used a `0.4 m x 0.3 m` rectangle, stereo-only cuVSLAM,
mapping enabled, 20 deg/s turns, and 15 seconds of settling after the final
revisit.

Native controller result:

- four turns: approximately 90 degrees each;
- tracking: `vo_state=1` throughout;
- truth closure translation: `0.0288 m`;
- truth closure yaw: `+6.60 deg`.

Backend comparison:

| Output | Closure translation | Closure yaw | Error relative to truth |
|---|---:|---:|---:|
| VO | `4.0242 m` | `-131.00 deg` | `4.0023 m / -137.60 deg` |
| SLAM path | `4.0242 m` | `-131.00 deg` | `4.0023 m / -137.60 deg` |

The two streams were identical for all `1273/1273` comparisons. The
PoseGraph still grew from `2` to `113` nodes and from `2` to `218` edges, so
keyframe/graph creation was active. However, no loop-closure optimization was
observable in `/visual_slam/tracking/slam_path`.

For comparison, the same small loop without the visual anchor also produced
identical VO and SLAM path (`2.9854 m`, `-110.88 deg`) while the graph grew to
`117` nodes and `226` edges. The anchor therefore did not make the backend
accept a loop constraint.

## Runtime/input finding

The NVIDIA parameter `image_jitter_threshold_ms` is the acceptable interval
between consecutive synchronized image messages, not the left/right camera
pair skew. The temporary `5 ms` setting was wrong and was removed. The launch
now uses `40 ms`: the simulator is nominally 30 Hz but its actual runtime can
produce longer frame intervals under Jetson load.

The tests also showed that enabling the expensive landmark/observation
visualization can reduce cuVSLAM output to roughly 13.5 Hz and produce
65--70 ms gaps or longer. Disabling that visualization removes the graph
topics as well, so it cannot be used for a PoseGraph audit. The GUI therefore
keeps visualization enabled for Foxglove, while low-volume diagnostics expose
the trade-off explicitly.

## Conclusion

The closed-loop route is implemented and the backend graph is active, but this
simulation does **not** yet demonstrate a successful loop-closure correction.
The remaining blocker is upstream of the final graph output: the visual
front-end accumulates a very large relative-pose error before the revisit, and
the backend does not accept the resulting loop candidate. Adding keyframes,
waiting longer, shrinking the loop, and adding a visual anchor did not change
that conclusion.

The next engineering step should be reducing Jetson runtime frame drops while
preserving PoseGraph publication, then inspecting the actual visual loop
candidate/rejection path. Do not inject wheel odometry or native-pose
correction; that would make the backend result meaningless.

## References

- [NVIDIA Visual SLAM parameters](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html)
- [NVIDIA Isaac ROS cuVSLAM documentation](https://nvidia-isaac-ros.github.io/concepts/visual_slam/cuvslam/index.html)
- [cuVSLAM paper](https://arxiv.org/abs/2506.04359)
