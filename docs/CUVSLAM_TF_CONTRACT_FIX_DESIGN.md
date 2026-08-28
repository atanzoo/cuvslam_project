# cuVSLAM TF Contract Fix

Date: 2026-07-30  
Status: Option 1 selected as the official architecture; runtime acceptance pending  
Change class: C2 frame and launch contract  
Scope: Jetson simulation only, ROS domain 43

## Objective

Publish a complete robot/sensor TF tree and make cuVSLAM publish the tracked
robot pose as `odom -> base_link`, while retaining the verified stereo
baseline in the corrected right CameraInfo.

After acceptance, the selected transforms become a locked simulation
calibration artifact. Further changes require a new C2 review and the same
three-run acceptance test.

## Existing Evidence

The current tree is:

```text
map -> odom -> camera_infra1_optical_frame
```

The output child is named as an optical frame, but a straight body motion is
reported primarily on `+X` rather than optical `+Z`. The installed Isaac ROS
node converts cuVSLAM poses to a canonical ROS body basis before publishing.
Therefore assigning an optical name to `base_frame` violates the output frame
contract.

The previous physical-frame experiment also supplied the stereo baseline
through left/right TF. It regressed straight tracking and was rolled back. The
right CameraInfo baseline path was subsequently verified and must remain the
sole stereo authority.

## Option 1: Hybrid Native cuVSLAM Base Transform

Selected.

Configure:

```text
base_frame = base_link
input_base_frame = base_link
input_left_camera_frame = camera_infra1_frame
input_right_camera_frame = empty
```

Publish the static robot tree before starting the cuVSLAM component:

```text
base_link
|-- camera_link
|   |-- camera_infra1_frame
|   |   `-- camera_infra1_optical_frame
|   |-- camera_infra2_frame
|   |   `-- camera_infra2_optical_frame
|   `-- camera_imu_frame
|       `-- slam_bot/camera_imu_frame/d435i_imu
`-- lidar_link
    `-- slam_bot/laser_frame/lidar
```

The left physical frame gives the node the chassis-to-camera lever arm in the
canonical ROS body basis. The right camera frame parameter remains empty, so
the node continues to read the stereo baseline from corrected CameraInfo.

Advantages:

- uses the node's native base-to-left compensation;
- output child and numeric basis both describe `base_link`;
- keeps the already verified CameraInfo stereo contract;
- does not add a runtime odometry conversion node;
- one node remains owner of `map -> odom` and `odom -> base_link`.

Disadvantages:

- requires several simulation-only static publishers;
- initialization depends on static TF readiness;
- relies on the installed node's documented canonical-basis behavior.

Failure modes:

- an incorrect physical-to-optical quaternion disconnects image semantics;
- a wrong lever-arm sign creates turn translation;
- starting cuVSLAM before static TF is available prevents initialization;
- duplicate static publishers make the tree nondeterministic.

Mitigation:

- start the component one second after static publishers;
- verify each edge numerically before motion;
- retain one CameraInfo stereo authority;
- cold-start each acceptance run.

## Option 2: External Pose Normalization Adapter

Leave cuVSLAM tracking the current left-optical child, disable its
`odom -> base` publication, and add a node that converts every cuVSLAM pose to
`base_link`.

Advantages:

- does not change cuVSLAM input frame parameters;
- conversion can be isolated and unit-tested;
- can preserve a raw estimator topic for debugging.

Disadvantages:

- adds latency and another runtime failure mode;
- requires explicit handling of map, odom, pose, path, covariance, and twist;
- ownership of `odom -> base_link` moves to a custom node;
- the current output is already mislabelled as optical, so the adapter must
  encode implementation-specific canonical behavior.

This option is rejected for the first fix because it adds a permanent runtime
component to compensate for a launch-level naming error.

## Static Numeric Contract

| Edge | Translation (m) | Quaternion XYZW |
|---|---|---|
| `base_link -> camera_link` | `(0.19, 0, 0.20)` | `(0,0,0,1)` |
| `camera_link -> camera_infra1_frame` | `(0,+0.025,0)` | `(0,0,0,1)` |
| `camera_link -> camera_infra2_frame` | `(0,-0.025,0)` | `(0,0,0,1)` |
| physical infra -> optical infra | `(0,0,0)` | `(-0.5,+0.5,-0.5,+0.5)` |
| `camera_link -> camera_imu_frame` | `(0,0,0)` | `(0,0,0,1)` |
| `camera_imu_frame -> slam_bot/camera_imu_frame/d435i_imu` | `(0,0,0)` | `(0,0,0,1)` |
| `base_link -> lidar_link` | `(0,0,0.13)` | `(0,0,0,1)` |
| `lidar_link -> slam_bot/laser_frame/lidar` | `(0,0,0)` | `(0,0,0,1)` |

Right CameraInfo remains:

```text
baseline = 0.05 m
P[3] = -10.416588...
```

## Quantitative Acceptance

Run three independent trials. Every profile begins from a cold Gazebo and
cuVSLAM start. All three trials must pass; averaging cannot hide one failure.

For each trial:

1. Static for at least 10 seconds.
2. Fixed straight command, evaluated using measured ground-truth distance.
3. Fixed in-place turn, evaluated using measured ground-truth angle.

Metrics:

```text
straight distance error =
  abs(estimated_base_distance - truth_base_distance) / truth_base_distance

turn angle error =
  abs(estimated_base_angle - truth_base_angle) / truth_base_angle
```

Pass gates:

- static translation drift `<= 0.01 m`;
- static rotation drift `<= 1.0 deg`;
- straight distance relative error `<= 15%`;
- straight absolute Z drift `<= 0.05 m`;
- straight false rotation `<= 5 deg`;
- turn angle relative error `<= 15%`;
- in-place-turn inferred base translation `<= 0.05 m`;
- `vo_state=1` throughout;
- endpoint timestamp skew `<= 50 ms`;
- the TF tree and publisher ownership match the Frame Contract.

The user's `15%` requirement applies independently to every relative metric in
every trial. Zero-truth metrics use the explicit absolute gates above because
relative error against zero is undefined.

## Deployment And Rollback

Deployment target:

```text
/workspaces/isaac_ros-dev/src/isaac_ros_visual_slam/
  isaac_ros_visual_slam/launch/isaac_ros_visual_slam_d435i_sim.launch.py
```

Before deployment, save the exact previous launch beside the experiment
bundle. The install tree is a symlink to this source file, so no package rebuild
is required.

If startup, TF ownership, static, or first straight gate fails:

1. publish zero `/cmd_vel`;
2. stop cuVSLAM;
3. restore the saved launch;
4. cold-start the prior configuration;
5. record the failed result without replacing it.

## Initial Test Result

Option 1 passed the startup and structural gates:

- cuVSLAM reached `vo_state=1`;
- the live tree was `map -> odom -> base_link -> sensors`;
- `base_link -> camera_infra1_optical_frame` was
  `(0.190, 0.025, 0.200)` with the expected optical rotation;
- cuVSLAM continued to report
  `left_pose_right reading from CameraInfo`;
- the reported stereo baseline remained `0.050000 m`.

It failed the first formal straight-motion gate:

| Metric | Truth/gate | Candidate result | Status |
|---|---:|---:|---|
| 3D base distance | `0.4476 m`, error `<=15%` | `0.5399 m`, error `20.6%` | FAIL |
| Horizontal distance | `0.4476 m` | `0.5049 m`, error `12.8%` | Informational |
| Absolute Z drift | `<=0.05 m` | `0.1912 m` | FAIL |
| False rotation | `<=5 deg` | `10.93 deg` | FAIL |
| Endpoint skew | `<=50 ms` | `20 ms` | PASS |
| Tracking | `vo_state=1` | remained `1` | PASS |

Trials 2 and 3 and the turn profile were not run because the predeclared rule
rejects the candidate after any failed trial. The deployed launch was restored
to checksum:

```text
7d66fdca3a2d03f9c60d00c983950597c2a9e00ba909142dbf63f2dcc70f3c5f
```

At that point the candidate remained only in the experiment bundle and was
not approved as an accuracy-qualified configuration. The later architecture
decision below adopts its frame topology without changing that failed accuracy
result.

## Architecture Decision

After confirming the installed Isaac ROS 2.1 implementation and NVIDIA's
documented frame parameters, Option 1 is the selected project architecture:

```text
map -> odom -> base_link
```

cuVSLAM natively owns both dynamic edges. The static robot description owns
`base_link -> sensor` transforms. An external pose adapter is not part of the
normal runtime.

The earlier failed motion gate remains valid evidence. Selecting the official
frame architecture does not waive or overwrite it. The deployed configuration
must be described as structurally correct but not accuracy-qualified until
three fresh trials each satisfy the acceptance metrics above.
