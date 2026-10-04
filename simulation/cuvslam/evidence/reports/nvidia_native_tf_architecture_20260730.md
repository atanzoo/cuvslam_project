# NVIDIA-Native cuVSLAM TF Architecture

Date: 2026-07-30  
Environment: Jetson simulation only, ROS domain 43  
Change class: C2 frame and launch contract  
Status: Deployed; structural runtime passed; motion Trial 1 failed

## Objective

Configure Isaac ROS cuVSLAM to use its native configurable base-frame
mechanism and publish:

```text
map -> odom -> base_link
```

The change must not modify cuVSLAM algorithms, Gazebo sensor generation,
right-camera calibration, Nav2, the real-robot workspace, or any motor
interface.

## Selected Design

The approved Option 1 from `docs/CUVSLAM_TF_CONTRACT_FIX_DESIGN.md` was
implemented:

```text
base_frame = base_link
input_base_frame = base_link
input_left_camera_frame = camera_infra1_frame
input_right_camera_frame = empty
```

cuVSLAM remains the sole owner of `map -> odom` and `odom -> base_link`.
Simulation static publishers own the locked `base_link -> sensor` tree. The
corrected right CameraInfo remains the sole stereo-baseline authority.

The estimator container starts one second after the static publishers so its
base-to-left-camera lookup is available during initialization.

## Local Files

- `deployment/isaac_ros/isaac_ros_visual_slam_d435i_sim.launch.py`
- `deployment/isaac_ros/README.md`
- `docs/CUVSLAM_TF_CONTRACT_FIX_DESIGN.md`
- `docs/sensor_and_frame_contract.md`
- `tools/test_cuvslam_launch_contract.py`

## Local Verification

Passed:

```text
python3 -m compileall \
  deployment/isaac_ros/isaac_ros_visual_slam_d435i_sim.launch.py \
  tools/test_cuvslam_launch_contract.py

python3 tools/test_cuvslam_launch_contract.py -v
3/3 passed

cd tools
python3 test_frame_contract_math.py -v
3/3 passed

python3 test_static_tf_target_contract.py -v
3/3 passed
```

The package tests requiring `pytest` were not run because the current Mac
Python environment does not include that dependency. No dependency was
installed for this launch-only change.

## Connectivity Incident

Initial SSH attempts to both recorded Jetson endpoints timed out:

```text
192.168.55.1:22
192.168.2.2:22
```

The NVIDIA USB interface was a member of the Mac Internet Sharing bridge.
`bridge100` had `192.168.2.1`, while Jetson retained
`l4tbr0=192.168.55.1/24`; the Mac-side `192.168.55.100` address had
disappeared. Layer-2 carrier and ARP were present, but ICMP and TCP timed out.

A temporary Mac-side alias initially restored the declared USB subnet:

```text
bridge100 alias 192.168.55.100/24
```

Afterward, ping passed 3/3 at `0.8-1.3 ms` and SSH port 22 was reachable.
The temporary alias was later replaced by the durable option: macOS Internet
Sharing was disabled for this direct USB workflow, while the existing
`Linux for Tegra` network service retained its manual
`192.168.55.100/24` configuration. The previous NAT configuration was backed
up at `/tmp/com.apple.nat.before_20260730.plist`.

After the change:

- InternetSharing and `natpmpd` were not running;
- `en19` retained `192.168.55.100/24` for more than 90 seconds;
- ping passed 18/18 with `0.900 ms` mean latency;
- SSH passed with both `BatchMode=yes` and
  `PasswordAuthentication=no`.

Jetson SSH key authentication is therefore passwordless. Internet access
through Mac Internet Sharing is intentionally unavailable on this interface;
that is separate from direct Jetson control.

## Jetson Deployment

The active launch was backed up as:

```text
isaac_ros_visual_slam_d435i_sim.launch.py.bak_20260730_1058
```

Checksums:

```text
new: 7bf3e3c9c5c8f4a3247b5a73a6378c1947103364979bea6315638154292993de
old: 7d66fdca3a2d03f9c60d00c983950597c2a9e00ba909142dbf63f2dcc70f3c5f
```

The uploaded file passed Jetson `py_compile`. Only the cuVSLAM estimator was
restarted; Gazebo and the ROS bridge remained active.

## Structural Runtime Result

Passed:

- cuVSLAM 11.4 reached `vo_state=1`;
- logs reported `left_pose_right reading from CameraInfo`;
- logs reported stereo baseline `0.050000 m`;
- tracking odometry used `frame_id=odom`, `child_frame_id=base_link`;
- `/tf` had one publisher, `visual_slam_node`;
- `odom -> base_link` was available to a fresh TF subscriber;
- `base_link -> camera_infra1_optical_frame` was
  `(0.190, 0.025, 0.200) m` with the locked optical rotation;
- left optical to right optical was `(+0.050, 0, 0) m`;
- `base_link -> slam_bot/laser_frame/lidar` was `(0, 0, 0.130) m`;
- static transforms were delivered with transient-local durability.

The old-style `static_transform_publisher` argument form emitted deprecation
warnings but was accepted by the installed ROS 2 Humble runtime. It does not
affect the measured transforms.

Formal motion Trial 1 was later run and failed the straight and turn gates.
Trials 2 and 3 were skipped under the declared stop-at-first-failure rule, and
the simulation was fully stopped. See
`reports/nvidia_native_tf_motion_acceptance_20260730.md`.

## Rollback

If startup, ownership, TF lookup, tracking, or a motion gate fails:

1. Publish zero `/cmd_vel`.
2. Stop cuVSLAM.
3. Restore the timestamped Jetson launch backup.
4. Cold-start the previous runtime.
5. Preserve the failed logs and bag without calling the architecture
   accuracy-qualified.

The earlier failed motion trial remains valid evidence and is not superseded
by the local launch-contract tests.
