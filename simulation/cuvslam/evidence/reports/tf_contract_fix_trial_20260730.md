# TF Contract Fix Trial

Date: 2026-07-30  
Environment: Jetson AGX Orin simulation, ROS domain 43  
Experiment:
`/home/tseng/isaac_ros_ws/data/experiments/tf_contract_fix_20260730_094434`  
Result: Failed first quantitative gate; runtime rolled back

## Change Under Test

The hybrid candidate:

- published a complete `base_link -> sensors` static tree;
- set cuVSLAM `base_frame` and `input_base_frame` to `base_link`;
- supplied `camera_infra1_frame` as the physical left camera frame;
- left `input_right_camera_frame` empty;
- retained the corrected right CameraInfo as the sole stereo-baseline source;
- delayed the cuVSLAM container for static TF readiness.

No SDF, image, CameraInfo adapter, cuVSLAM algorithm, or real-robot component
was changed.

## Structural Result

PASS:

```text
map
`-- odom
    `-- base_link
        |-- camera_link
        |   |-- camera_infra1_frame
        |   |   `-- camera_infra1_optical_frame
        |   |-- camera_infra2_frame
        |   |   `-- camera_infra2_optical_frame
        |   `-- camera_imu_frame
        `-- lidar_link
```

Additional scoped IMU and LiDAR aliases were connected under their normalized
frames. `map -> simulation_world_ground_truth` remained evaluation-only.

The measured left-camera extrinsic was:

```text
translation: (0.190, 0.025, 0.200) m
rotation: body to ROS optical convention
```

cuVSLAM reached `vo_state=1` and logged:

```text
left_pose_right reading from CameraInfo
Baseline is : 0.050000
```

## Formal Trial 1

Profile:

```text
mode: straight
speed: 0.12 m/s
duration: 5.0 s
controller ground-truth subscriptions: none
```

Bag:

```text
trial1/straight/bag
```

Results:

| Motion | Translation vector (m) | Norm | Rotation |
|---|---|---:|---:|
| Ground-truth base | `(+0.4476, 0, 0)` | `0.4476 m` | `0.00 deg` |
| cuVSLAM base | `(+0.4938, -0.1053, +0.1912)` | `0.5399 m` | `10.93 deg` |

Metrics:

```text
3D distance error = abs(0.5399 - 0.4476) / 0.4476 = 20.6%
horizontal estimate = hypot(0.4938, -0.1053) = 0.5049 m
horizontal distance error = 12.8%
absolute Z drift = 0.1912 m
false rotation = 10.93 deg
endpoint timestamp skew = 20 ms
```

The 3D distance, Z, and false-rotation gates failed. Tracking and timestamp
gates passed.

## Invalid Recorder Attempt

An earlier motion attempt used an incorrectly quoted SSH recorder command.
The controller completed safely and published zero velocity, but the bag path
and recorder PID were not captured as intended. The recorder was stopped by
its exact PID and the bag was preserved under:

```text
invalid_recorder_attempt/bag
```

It is explicitly excluded from the formal trial count.

## Decision And Rollback

The acceptance rule requires all metrics in all three independent trials to
pass. Because formal trial 1 failed, trials 2 and 3 and the turn profile were
not run.

The candidate launch was preserved as:

```text
deployment/launch.candidate.py
```

The original launch was restored from:

```text
deployment/launch.before.py
```

Restored checksum:

```text
7d66fdca3a2d03f9c60d00c983950597c2a9e00ba909142dbf63f2dcc70f3c5f
```

After a cold restart, the restored runtime returned to `vo_state=1`.

## Conclusion

The hybrid candidate corrected TF topology and output naming, but it did not
bring trajectory accuracy within the required 15 percent gate. In particular,
static coordinate changes cannot remove the observed `10.93 deg` rotation
magnitude error; rotation magnitude is invariant under a valid change of
basis.

Further tuning of physically measured TF values to cancel this estimator
error would create a false calibration and is not approved. The next diagnosis
must return to simulated image-motion geometry, camera FOV/intrinsics, and
cuVSLAM canonical input semantics before another TF candidate is proposed.
