# cuVSLAM 1 m Observability Fix

Date: 2026-07-31 (Asia/Taipei)  
Environment: Jetson AGX Orin, ROS 2 Humble container, ROS domain 43  
Estimator: Isaac ROS Visual SLAM 2.1.0 / cuVSLAM 11.4  
IMU fusion: disabled

## Result

The 1 m simulation failure was reproduced, isolated, and fixed without
changing camera geometry, TF, CameraInfo, stereo baseline, timing, estimator
parameters, or output-frame ownership.

The root cause was insufficient near-field visual observability in
`cuvslam_mapping_simple.sdf`. The forward view was dominated by uniform grey
surfaces and features on the far wall. At approximately 208 px focal length
and a 0.05 m baseline, features 3.6–4.6 m away provide only about 2–3 px of
stereo disparity.

An independent world variant added asymmetric, high-contrast, visual-only
boards beside the route. It was subsequently expanded to cover every leg of
the square mapping route. The original world remains unchanged.

## Reliable test ownership

Earlier interactive SSH attempts overlapped with older cleanup shells, which
later stopped a newer container and produced exit code 137. Docker reported
`OOMKilled=false`; this was orchestration ownership, not an OOM failure.

The accepted test uses:

- `tools/run_jetson_1m_acceptance.sh`;
- `tools/run_container_1m_trial.sh`;
- one transient systemd owner;
- a cleanup trap that publishes zero velocity and stops recorder, cuVSLAM,
  bridge, Gazebo, and the container.

## Baseline 1 m result

Bundle:

```text
/home/tseng/isaac_ros_ws/data/acceptance_1m_20260731_18946
```

| Metric | Native truth | cuVSLAM |
|---|---:|---:|
| Translation | 1.0116 m | 0.0749 m |
| Rotation | 0.00 deg | 1.56 deg |
| Relative X | +1.0116 m | +0.0561 m |
| Relative Y | 0.0000 m | +0.0205 m |
| Relative Z | 0.0000 m | -0.0451 m |
| Status | - | 492 / 492 `vo_state=1` |

The baseline underestimates translation by approximately 92.6 percent.

During native motion:

- ROS left: 258 frames, 258 unique hashes;
- ROS right: 258 frames, 258 unique hashes;
- NVIDIA debug left/right: 243 frames each, every frame unique;
- 258 exact bridged stereo timestamp matches;
- median timestamp interval: 35 ms.

This excludes stale pixels as the cause of this baseline failure.

## Observable-world A/B result

World:

```text
deployment/slam_gazebo/worlds/cuvslam_mapping_observable.sdf
```

Bundle:

```text
/home/tseng/isaac_ros_ws/data/acceptance_1m_observable_20260731_21873
```

| Metric | Native truth | cuVSLAM |
|---|---:|---:|
| Translation | 1.0128 m | 1.0232 m |
| Rotation | 0.00 deg | 0.98 deg |
| Relative X | +1.0128 m | +1.0217 m |
| Relative Y | 0.0000 m | +0.0543 m |
| Relative Z | 0.0000 m | +0.0066 m |
| Status | - | 486 / 486 `vo_state=1` |

The translation-norm error is approximately +1.0 percent. The 3D residual is
`0.0554 m`; orientation error is `0.98 deg`.

During native motion, ROS and NVIDIA debug left/right streams each contained
256 frames, every frame had a unique hash, every bridged stereo timestamp
matched exactly, and the median timestamp interval was 35 ms.

The test passes:

- truth distance `0.95–1.05 m`;
- cuVSLAM translation `0.95–1.05 m`;
- orientation error at most `15 deg`;
- no scale multiplier or truth correction.

## Change boundary

Changed:

- added a simulation-only observable world variant;
- added deterministic test and analysis helpers;
- added a C1 design record.

Unchanged:

- `cuvslam_mapping_simple.sdf`;
- stereo resolution, FOV, baseline, and CameraInfo;
- TF and `odom -> base_link` ownership;
- IMU fusion;
- cuVSLAM parameters;
- production LiDAR/Nav2 and real-robot workspaces.

Jetson validation:

```text
baseline SHA-256:
de9fe0515ae0142ab9d3cb54f9b0af16b96e0776703c26a333431cfc1d39791f

observable SHA-256:
9a6e937c027779c199618ea3a7d39883439c53d636cc2fc58c9643acb794d66f

ign sdf -k:
Valid.
```

## Conclusion

The original 1 m failure was a simulation-scene observability failure. Unique
image hashes alone were insufficient: the sparse far-field view changed every
frame but did not provide enough robust stereo disparity and parallax for
accurate forward-motion estimation. Near-field asymmetric features restored
metric tracking without estimator or calibration changes.

The observable world is the accepted diagnostic and mapping-simulation entry
point. Two later cold-start repeats passed the same gate; see
`reports/cuvslam_observable_world_repeatability_20260731.md`. The sparse
baseline remains preserved as a negative control.
