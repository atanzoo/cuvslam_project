# Gazebo Stereo Image Boundary Trial

Date: 2026-07-31 (Asia/Taipei)
Target: Jetson AGX Orin, ROS domain 43
Partition: `cuvslam_boundary_20260731`

## Scope

This was the first controlled follow-up to the 2026-07-30 handoff. It used
the independent `cuvslam_mapping_simple.sdf` world and a transient
Gazebo/bridge pair. No TF, CameraInfo, cuVSLAM parameter, Nav2, or production
workspace changes were made.

The same short straight-motion interval was captured at Gazebo Transport,
bridged ROS image topics, and NVIDIA cuVSLAM debug-dump images. Native truth
was used only to command and evaluate the motion.

## Motion and pixel result

The clean capture drove native truth from `x=0.5148 m` to `x=1.0308 m`.
Truth started moving at `265.15 s` and stopped at `269.55 s`.

All three boundaries showed changing pixel content throughout that motion
window. Repetition began at approximately `269.545 s`, after native truth had
stopped, in Gazebo Transport, both ROS image topics, and the NVIDIA debug
dump. The ROS left image had 344 frames and 136 unique pixel hashes; its long
repeated runs were before motion and after motion. The right image behaved the
same way.

Conclusion: this trial does **not** identify an in-motion stale-pixel owner.
The observed repetition is explained by the stationary scene after the
commanded motion. The previous report's in-motion freeze hypothesis remains
unresolved because this run did not reproduce it.

## cuVSLAM result

The bag contained 332 tracking-odometry and 332 status samples. Every status
sample had `vo_state=1`.

Relative cuVSLAM tracking odometry changed from
`(0.0282, 0.0059, -0.0149) m` to `(0.1317, 0.0489, -0.1458) m`, a relative
translation norm of approximately `0.172 m` versus `0.516 m` native truth.
This remains a failed trajectory-quality result despite continuous tracking.

## Evidence and shutdown

```text
/home/tseng/isaac_ros_ws/data/boundary_20260731/retry1/transport/
/home/tseng/isaac_ros_ws/data/boundary_20260731/bags/retry1/retry1_0.db3
/home/tseng/isaac_ros_ws/data/boundary_20260731/debug_dump/
```

The transient Gazebo and bridge services are inactive, no related host
process remains, and the Isaac ROS container is exited. No code or deployed
simulation contract was changed.

## Next single hypothesis

Reproduce the original 1 m test from a cold Gazebo start while recording the
same three boundaries and a synchronized native-truth pose stream. Do not
change estimator parameters until the three pixel streams are proven to
freeze while truth is still moving.
