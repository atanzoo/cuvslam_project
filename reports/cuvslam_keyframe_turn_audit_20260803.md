# cuVSLAM keyframe / turn audit — 2026-08-03

## Question

Determine whether the poor corner-map result is caused by the 90-degree turn
not satisfying cuVSLAM's keyframe-update rule.

## NVIDIA reference model

The cuVSLAM paper describes keyframe selection in the 2D frontend: feature
tracks are propagated with coarse-to-fine Lucas–Kanade tracking and NCC
validation; when the number of successfully tracked keypoints falls below a
threshold, a new keyframe is created. In stereo mode that keyframe triggers
cross-camera observations, triangulation, local-map insertion, and asynchronous
refinement. The rule is therefore tracking-quality driven, not a fixed
"90 degrees" or "20 degrees/second" rule.

NVIDIA's current API also exposes automatic keyframe selection, a survivor-track
threshold, and a maximum time between keyframes. The Isaac ROS ROS-level
visualization contract exposes `pose_graph_nodes` and `pose_graph_edges`, but
does not publish a per-frame boolean saying "this frame was selected as a
keyframe". The PoseGraph topics are therefore the appropriate observable proxy
for this experiment, not a direct keyframe flag.

## Diagnostic run

- World: corrected visual-only corner-feature world
- Sensors: stereo, IMU fusion disabled, 30 Hz camera / 200 Hz IMU available
- Motion: `1.0 m -> 90 degrees -> 0.5 m`
- Turn command: 20 degrees/s
- Recorder: pose, status, native truth, TF, `pose_graph_nodes`,
  `pose_graph_edges`; no images or point clouds
- Bag size: 4.9 MiB
- Tracking: `vo_state=1` for `884/884` samples

PoseGraph output was phase-aligned to the rosbag recorder timestamps and the
native Gazebo pose. The cumulative PoseGraph node count changed as follows:

| Phase | PoseGraph messages | node count | positive increases | added nodes |
|---|---:|---:|---:|---:|
| First leg | 56 | 2 -> 26 | 24 | 24 |
| Turn | 44 | 26 -> 68 | 35 | 42 |
| Second leg | 17 | 69 -> 85 | 16 | 16 |

Edges showed the same behavior: 50 -> 130 during the turn, adding 80 edge
points. Thus the turn was not a keyframe-free interval; it produced the
largest PoseGraph growth of the three phases.

The same bag's pose evaluation was:

- first leg yaw error: `+0.05°`
- turn yaw error: `-8.41°`
- turn SE(3) translation residual: `3.45 cm`
- second leg yaw error: `-0.01°`
- complete-route yaw error: `-8.37°`
- tracking status: all valid

## Conclusion

The hypothesis is **not confirmed** in its simple form. The turn did satisfy
the observable keyframe/PoseGraph update behavior: node and edge updates
continued through the turn and were denser than on the second leg. Therefore,
the map/pose defect is not explained by “the turn never generated keyframes.”

The remaining issue is downstream of, or coupled to, a keyframe update: the
turn has active PoseGraph growth but still under-estimates yaw by `8.41°`.
The most likely next isolation target is the quality of the observations and
the stereo/3D pose solve during the turn (feature survival, cross-camera
matches, triangulation conditioning, and pose-graph optimization), rather than
the existence of keyframes itself.

## Implementation

The low-volume recorder now includes `pose_graph_nodes` and
`pose_graph_edges` in pose-only trials, and the outer runner correctly forwards
`CUVSLAM_POSE_ONLY` so future keyframe audits cannot accidentally record the
large image/point-cloud streams.

