# NVIDIA-Native TF Motion Acceptance

Date: 2026-07-30  
Environment: Jetson AGX Orin simulation, ROS domain 43  
Experiment:
`/home/tseng/isaac_ros_ws/data/experiments/nvidia_native_tf_acceptance_20260730_1115`  
Result: Failed formal Trial 1; Trials 2 and 3 skipped by the declared gate  
Final runtime state: Simulation stopped

## Scope

Test the deployed native Isaac ROS frame architecture:

```text
map -> odom -> base_link -> sensors
```

No cuVSLAM algorithm, Gazebo world, CameraInfo calibration, sensor model,
Nav2, real-robot workspace, or motor interface was changed.

The estimator-only controller subscribed to `/scan` and
`/visual_slam/status`. It did not subscribe to ground truth. Ground truth was
recorded only for offline evaluation.

## Test Procedure

Each formal trial was designed to begin from cold Gazebo, bridge, and cuVSLAM
processes and run:

1. 10 seconds stationary;
2. straight command at `0.12 m/s` for `5.0 s`;
3. in-place turn at `-0.12 rad/s` for `5.0 s`.

The runner installed an `EXIT/INT/TERM` trap that published zero velocity and
stopped the exact rosbag recorder PID. An initial runner invocation failed
before recording or motion because ROS setup was sourced under shell
`nounset`; it was corrected and is excluded from the formal trial count.

## Formal Trial 1

All controller status samples observed during motion reported `vo_state=1`.
Minimum measured scan clearance remained at least `0.740 m`.

### Stationary

| Metric | Gate | Result | Status |
|---|---:|---:|---|
| Translation drift | `<=0.01 m` | `0.0000 m` | PASS |
| Rotation drift | `<=1.0 deg` | `0.00 deg` | PASS |
| Endpoint timestamp skew | `<=50 ms` | `10 ms / 5 ms` | PASS |

### Straight

Ground truth:

```text
translation=(+0.4512, 0, 0) m
distance=0.4512 m
rotation=0.00 deg
```

cuVSLAM `base_link`:

```text
translation=(+0.5466, +0.0800, +0.1289) m
3D distance=0.5673 m
horizontal distance=0.5524 m
rotation=6.55 deg
```

| Metric | Gate | Result | Status |
|---|---:|---:|---|
| 3D distance relative error | `<=15%` | `25.7%` | FAIL |
| Horizontal distance error | informational | `22.4%` | FAIL |
| Absolute Z drift | `<=0.05 m` | `0.1289 m` | FAIL |
| False rotation | `<=5 deg` | `6.55 deg` | FAIL |
| Endpoint timestamp skew | `<=50 ms` | `20 ms / 15 ms` | PASS |
| Tracking | `vo_state=1` | remained `1` | PASS |

### In-Place Turn

```text
ground-truth rotation=29.46 deg
cuVSLAM rotation=19.75 deg
rotation magnitude error=33.0%
cuVSLAM inferred base translation=0.7243 m
```

| Metric | Gate | Result | Status |
|---|---:|---:|---|
| Turn-angle relative error | `<=15%` | `33.0%` | FAIL |
| In-place base translation | `<=0.05 m` | `0.7243 m` | FAIL |
| Endpoint timestamp skew | `<=50 ms` | `10 ms / 0 ms` | PASS |
| Tracking | `vo_state=1` | remained `1` | PASS |

## Gate Decision

The design requires every metric in every independent trial to pass and stops
at the first failed formal gate. Trial 1 failed the straight and turn gates.
Trials 2 and 3 were therefore not executed. This avoids adding motion after
the candidate has already failed its declared acceptance criteria.

The result confirms that the NVIDIA-native TF topology is structurally
correct, but it does not correct the simulated trajectory error. The remaining
failure is estimator/input geometry behavior: false Z, false rotation,
distance inflation, turn-angle underestimation, and large false translation
remain observable after both truth and estimate refer to `base_link`.

## Final Shutdown Evidence

After evaluation:

- `cuvslam-d435i-bridge.service`: `inactive`;
- `cuvslam-d435i-gazebo.service`: `inactive`;
- no cuVSLAM launch process;
- no rosbag recorder process;
- no `simulation_estimator_profile.py` process;
- no Ignition Gazebo process;
- no `parameter_bridge` process;
- ROS reported `Unknown topic '/cmd_vel'`.

The Mac Internet Sharing daemon was temporarily paused during this test and
initially resumed afterward. A later operations fix disabled Internet Sharing
for `en19` because it repeatedly removed the project's manual
`192.168.55.100/24` address. Passwordless SSH and 90-second link stability were
then verified. No simulation process was left active.
