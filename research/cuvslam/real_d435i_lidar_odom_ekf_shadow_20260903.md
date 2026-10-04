# Real D435i + RPLIDAR A2M12: LiDAR odometry and EKF shadow implementation

Date: 2026-09-03 (Asia/Taipei)

## Current status (2026-09-10)

This record is retained as the future LiDAR odometry + EKF enhancement path.
For the currently tested indoor space, navigation development uses the
cuVSLAM-only odometry baseline with emitter OFF. The shadow stack remains
available for comparison and fault isolation, but it is deferred, does not
publish the formal `odom -> base_link` TF, and is not required for the current
navigation baseline. Revisit it only after cuVSLAM-only navigation exposes a
repeatable failure that a second odometry source is expected to address.

## Objective

Implement the first safe comparison path for the observed cuVSLAM turn
geometry problem, where a nominal approximately 90-degree turn reported up to
`0.696 m` XY translation. The selected architecture is a diagnostic branch:

```text
cuVSLAM /visual_slam/tracking/odometry ----┐
                                           ├-> robot_localization EKF shadow
RPLIDAR A2M12 /scan -> rf2o -> /lidar/odom-┘
```

The objective of this change is to make independent LiDAR odometry available
and to compare it with cuVSLAM before considering any estimator promotion.

## Scope and non-goals

In scope:

- an opt-in `rf2o_laser_odometry` node for `/scan` to `/lidar/odom`;
- an opt-in `robot_localization` EKF using cuVSLAM and LiDAR odometry;
- diagnostic output `/odometry/filtered_shadow`;
- GUI estimator selection, live source/comparison display, process-log entry,
  copyable summary, and saved session/trace fields;
- light/full recording topic coverage and dependency preflight.

Not in scope:

- changing the formal `odom -> base_link` TF owner;
- changing slam_toolbox's `map -> odom` ownership;
- Nav2, `/cmd_vel`, robot motion, calibrated extrinsics, or safety behavior;
- installing ROS packages or synchronizing files to Jetson;
- claiming LiDAR odometry, EKF, map quality, or navigation accuracy before a
  hardware run produces evidence.

## Runtime contract

The existing mapping runner remains the default:

```text
run_real_d435i_rplidar.sh start ... cuvslam
```

The diagnostic stack is explicit:

```text
run_real_d435i_rplidar.sh start imu 640x360x30 official standard ekf_shadow
```

The integrated launch starts the additional nodes only when the corresponding
launch switches are true. Both additional nodes have TF publication disabled.
The EKF output is remapped to `/odometry/filtered_shadow`; it is not the
standard `/odometry/filtered` production output and it does not publish TF.

The GUI compares relative poses from each source after its first accepted
message. The current reference is:

- PASS: XY difference `<= 0.10 m` and yaw difference `<= 5 deg`;
- REVIEW: XY difference `<= 0.20 m` and yaw difference `<= 10 deg`;
- FAIL: otherwise.

These thresholds are diagnostic review thresholds. They do not compensate for
time offset, extrinsic errors, scan matching failure, or independent source
latency.

## EKF contribution observability (2026-09-10)

The live collector and GUI now keep two distinct measurements:

1. A geometric contribution estimate projects the EKF relative pose onto the
   line between the cuVSLAM and LiDAR relative poses, separately for XY and
   yaw. The GUI displays the clipped operator-facing percentages and retains
   the raw projection. This is a trajectory-behavior estimate, not the EKF's
   internal Kalman gain.
2. The collector subscribes to `/diagnostics` and retains the
   `robot_localization` diagnostic status. The GUI work record surfaces the
   fields actually reported by that status, including processed, rejected,
   dropped, ignored, and frequency values when present. Missing fields remain
   unavailable rather than being inferred.

The source timestamps are also compared. A source skew above `100 ms` marks
the contribution estimate as `REVIEW`, because a percentage computed from
unsynchronized samples is not a reliable fusion explanation. The recorded
diagnostic status is still evidence about the EKF node, not proof that either
sensor is correct.

## Files changed

- `real_robot/cuvslam/deployment/isaac_ros/isaac_ros_visual_slam_d435i_real.launch.py`
  exposes the existing cuVSLAM odom-TF switch with default `true`.
- `real_robot/cuvslam/deployment/isaac_ros/isaac_ros_visual_slam_d435i_rplidar.launch.py`
  adds conditional LiDAR odometry and EKF shadow nodes, both default off.
- `real_robot/cuvslam/config/real_d435i_rplidar_lidar_odom.yaml`
  defines the diagnostic rf2o topic/no-TF configuration.
- `real_robot/cuvslam/config/real_d435i_rplidar_ekf_shadow.yaml`
  defines the diagnostic two-input EKF/no-TF configuration.
- `real_robot/cuvslam/tools/run_real_d435i_rplidar.sh`
  adds estimator selection, records the new topics, and exposes preflight.
- `real_robot/cuvslam/tools/collect_real_d435i_odom_live.py`
  collects three odometry sources and emits source/comparison telemetry.
- `real_robot/cuvslam/tools/real_d435i_axis_web_gui.py`
  records the selected stack and displays its comparison without changing TF.
- `real_robot/cuvslam/tools/check_real_d435i_lidar_ekf_dependencies.sh`
  performs a read-only Jetson-container dependency check.
- `real_robot/cuvslam/tools/test_real_d435i_rplidar_contract.py`
  statically checks the new ownership and topic contract.
- `real_robot/cuvslam/deployment/isaac_ros/Dockerfile.ekf_shadow`
  defines the persistent offline GeographicLib image layer used by the
  Jetson deployment.
- `real_robot/cuvslam/deployment/isaac_ros/ekf_shadow_sources.lock`
  records the exact external source refs used for the offline deployment.

## Verification status

Local static verification is required for this implementation:

- Python compilation for the collector, GUI, launch files, and contract test;
- Bash syntax check for the runner and dependency preflight;
- static RPLIDAR integration contract test;
- `git diff --check`.

Completed before hardware:

- Jetson dependency preflight in the deployed image;
- no-sensor startup checks for `rf2o_laser_odometry_node` and `ekf_node`;
- Jetson workspace build for `geographic_msgs`, `rf2o_laser_odometry`,
  `robot_localization`, and `isaac_ros_visual_slam`;
- local Python/Bash compilation and the static ownership contract test.

Not run yet:

- stationary topic/QoS/TF checks with D435i and A2M12 connected;
- pure-turn, straight-motion, or closed-loop comparison;
- EKF promotion or Nav2 integration.

## Risks and rollback

The external packages may be absent or expose different runtime parameters in
the Jetson image. The preflight is therefore a required gate and the runner
must fail clearly instead of installing packages implicitly. Scan matching can
also fail in feature-poor or geometrically ambiguous scenes; an EKF output can
look smooth while being wrong. The first comparison is consequently retained
as evidence and is not allowed to change formal TF ownership.

Rollback is to select `cuvslam` or omit the optional launch switches. This
leaves the prior cuVSLAM + A2M12 + slam_toolbox runtime path intact. No logs,
bags, maps, calibration, or historical reports are removed by this change.

## Jetson deployment completed (2026-09-10)

The selected shadow branch is now deployed to the canonical Jetson Isaac ROS
workspace at `/home/tseng/isaac_ros_ws`. The deployed runner uses the separate
image `isaac_ros_dev-aarch64:ekf-shadow`; the existing `fixed` image remains
available for the prior runtime and observer container.

Because the Jetson container could not resolve external apt/GitHub hosts, the
image contains an offline build of GeographicLib from pinned source commit
`48959dfc4a581ad29f76b526726a801473767c98`. The ROS sources were staged at
the Jetson workspace with these refs:

- `robot_localization` `8696ee5a9e4f959fcaae37835dcf2ed12ead581b`;
- `rf2o_laser_odometry` `b38c68e46387b98845ecbfeb6660292f967a00d3`;
- `geographic_info` `f70b81a438172cd7a066dc1b18314d70e0eb6389`.

The build completed successfully. The no-sensor checks confirmed:

- `ros2 pkg prefix robot_localization` and `rf2o_laser_odometry` resolve;
- `rf2o_laser_odometry_node` and `robot_localization/ekf_node` are installed;
- both nodes stay alive when no scan/odom messages are present and exit only
  when the bounded test timeout stops them;
- the deployed preflight reports
  `preflight_ok mode=ekf_shadow tf_owner=cuvslam`;
- the integrated launch exposes `enable_lidar_odom` and
  `enable_ekf_shadow`.

The runner and launch/config files were hash-verified against the local
project after synchronization. A pre-deployment backup is preserved at
`/home/tseng/isaac_ros_ws/backups/ekf_shadow_predeploy_19700101_090410`.
The Jetson clock is currently incorrect, so the backup directory timestamp and
build warnings are not trustworthy wall-clock evidence; this was recorded but
the system clock was not changed as part of this deployment.

No D435i/A2M12 runtime was started by this deployment, no motion test was run,
and no TF owner changed. The next hardware gate remains stationary first:
start the GUI with estimator `ekf_shadow`, verify `/scan`, `/lidar/odom`,
`/odometry/filtered_shadow`, timestamps and TF ownership, then proceed to
labelled pure-turn and straight-motion tests only if the stationary gate
passes.

## Stationary hardware gate (2026-09-10)

The first stationary start exposed a deployment mismatch: the Jetson source
package still had the older cuVSLAM real-camera launch file, so the intended
`publish_odom_to_base_tf` argument was not present. The existing runtime was
stopped, all three deployed copies were backed up, and the current real-camera
launch file was synchronized before retesting.

The corrected stationary run used:

```text
mode=imu
profile=640x360x30
lighting=official
diagnostics=standard
estimator=ekf_shadow
```

Observed data-path results:

- `/scan`: approximately `12.0 Hz`;
- `/lidar/odom`: approximately `11.3 Hz`;
- `/odometry/filtered_shadow`: approximately `29.3 Hz`;
- `/map`: one publisher and one subscriber;
- both `/rf2o_laser_odometry` and `/ekf_shadow` resolved
  `publish_tf=False`;
- production `/tf` had three publishers: cuVSLAM and two slam_toolbox
  endpoints; neither shadow node appeared there;
- the shadow broadcasters were isolated to
  `/shadow/lidar_odom_tf_unused` and `/shadow/ekf_tf_unused` as a defensive
  remap;
- the stop flow returned `std_srvs/srv/Empty_Response`, and no process held
  `/dev/ttyUSB0` afterward.

This is a stationary interface/ownership gate only. It does not establish
LiDAR odometry accuracy, EKF correction quality, pure-turn behavior, straight
motion, closed-loop accuracy, or suitability for Nav2. The mapping container
was stopped after the gate, while the separate `real_d435i_rviz` observer was
left running by design.

## Multi-source odom visualization and covariance guard (2026-09-10)

The local collector and GUI were extended for the next comparison step. The
GUI now reconstructs bounded common-`odom` tracks for three sources and draws
them together: cuVSLAM, `/lidar/odom`, and `/odometry/filtered_shadow`. The
mapping canvas remains the map-frame cuVSLAM path; the three-source view is a
separate local-odom diagnostic and does not imply a TF handoff.

The collector now emits compact pose/twist covariance summaries for every
source. The summary distinguishes invalid covariance, zero covariance in the
planar fields used by the shadow filter, and zero values in unused fields.
The GUI work record and saved R2 session retain these warnings. They are
currently isolated as observability warnings only: no source covariance is
rewritten and the shadow EKF remains `publish_tf=false`.

Pairwise timestamp skew is written to the process log and summary. If the
three-source skew exceeds 100 ms, the geometric projection is marked
unavailable instead of being displayed as a misleading contribution
percentage. This percentage remains a trajectory projection, not an internal
robot_localization Kalman gain.

Verification completed locally:

- Python compilation passed for the collector, GUI, and contract test;
- `REAL_D435I_RPLIDAR_CONTRACT_PASS`;
- `git diff --check` passed.

Not performed in this change: Jetson synchronization, runtime startup,
stationary hardware verification, motion testing, covariance replacement, or
promotion of EKF to the formal TF owner. The next action is to sync the local
tools to Jetson, run the stationary gate, and then perform labelled pure-turn
and straight-motion comparisons.

## Fixed A2M12 mounting yaw plumbing (2026-09-10)

The launch had a `lidar_yaw` input for the static `base_link -> rplidar_link`
TF, but the real mapping runner and Mac GUI did not carry the fixed mounting
reference through to that input. The local runner now accepts the optional
seventh `start` argument and falls back to `CUVSLAM_LIDAR_YAW`; the GUI reads
the same local setting, forwards it to Jetson, and records the requested value
in the process log and saved session. Non-numeric yaw values are rejected
before the container is started.

The default remains `0.0` radians. This preserves the current physical mount
as the reference and avoids guessing a 180-degree reversal from a plotted
trajectory alone. The change does not alter `odom -> base_link` ownership,
`map -> odom` ownership, EKF shadow policy, or scan ordering. It is not an
extrinsic calibration or motion-accuracy result; Jetson synchronization and
the labelled stationary/straight/pure-turn verification remain pending.

## Scheme A runner synchronization (2026-09-10)

The approved runner deployment is complete. The remote runner now accepts the
optional yaw argument and forwards it to the integrated launch. The previous
Jetson copy was preserved at
`/home/tseng/isaac_ros_ws/backups/run_real_d435i_rplidar.sh.before_yaw_20260910`.
The Mac GUI's ignored local setting requests `2.334 rad`, which corresponds to
the approximately `+133.7°` rotation estimated from the latest common-odom
trace. The remote launch source and installed launch link already contained
the `lidar_yaw` input, so no package rebuild was required.

The local and remote runner hashes matched and syntax/contract checks passed.
No runtime was started during synchronization; stationary and labelled motion
tests are still required before treating the candidate yaw as calibrated.
