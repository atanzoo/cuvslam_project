# cuVSLAM In-Place Turn Pose Fix

Date: 2026-07-31  
Platform: Jetson AGX Orin, Isaac ROS Visual SLAM 2.1.0 / cuVSLAM 11.4  
Mode: stereo-only, mapping enabled, D435i simulation at 640x360 and nominal 30 Hz  

## Result

The pure 90-degree turn pose gate now passes in two independent cold starts.
The accepted configuration uses a 20 deg/s command and the dense-turn v3
world, which provides non-repeating near-field geometry continuously across
the full 0--90 degree field of view.

| Cold start | Native yaw | cuVSLAM yaw | Yaw error | cuVSLAM false translation | SE(3) translation residual | Tracking lost |
|---|---:|---:|---:|---:|---:|---:|
| v3 run 1 | 90.47 deg | 89.72 deg | -0.75 deg | 0.0030 m | 0.0217 m | 0 |
| v3 run 2 | 90.70 deg | 90.78 deg | +0.08 deg | 0.0167 m | 0.0366 m | 0 |

Acceptance limits were absolute yaw error no greater than 5 degrees, false
translation no greater than 0.05 m, and zero tracking loss. Both runs passed.

## Data Contract

Wheel odometry is prohibited from the active bridge, recorder, controller,
cuVSLAM, and evaluator:

- Gazebo DiffDrive remains only as the `/cmd_vel` actuator.
- `/ground_truth/odom` is not bridged or recorded.
- turn termination uses timestamped D435i gyro integration;
- physical truth comes only from Gazebo native dynamic pose and is used
  offline;
- the runner fails immediately if `/ground_truth/odom` appears.

The physical turn controller and truth cross-check remained stable: the bag
IMU and Gazebo native yaw previously agreed to approximately 0.02 degree.

## Isolation Results

The investigation separated speed, IMU fusion, camera lever arm, backend
mapping, render range, and scene observability.

| Configuration | Yaw error | False translation | Result |
|---|---:|---:|---|
| baseline 5 deg/s | -16.95 deg | 0.5316 m | FAIL |
| baseline 10 deg/s | -3.92 deg | 0.2336 m | FAIL |
| baseline 12.7 deg/s | -1.78 deg | 0.1315 m | FAIL |
| baseline 20 deg/s | -1.12 deg | 0.1223 m | FAIL |
| baseline 30 deg/s | -3.81 deg | 0.1427 m | FAIL |
| VIO, 12.7 deg/s | -9.81 deg | 0.4000 m | FAIL |
| camera at base origin, 20 deg/s | -5.31 deg | 0.1854 m | FAIL |
| VO-only backend, 20 deg/s | -4.58 deg | 0.1787 m | FAIL |
| 1.5 m render range, 20 deg/s | -31.60 deg | 0.9315 m | FAIL |
| dense v1, 20 deg/s | -1.09 deg | 0.0763 m | FAIL |
| dense v2 run 1, 20 deg/s | +0.16 deg | 0.0454 m | PASS |
| dense v2 repeat, 20 deg/s | -0.45 deg | 0.0684 m | FAIL |
| dense v3 run 1, 20 deg/s | -0.75 deg | 0.0030 m | PASS |
| dense v3 run 2, 20 deg/s | +0.08 deg | 0.0167 m | PASS |

Increasing turn speed fixed most of the yaw under-estimation but left a
10--14 cm translation floor in the original world. IMU fusion made the result
worse. Moving the camera to the chassis origin and disabling the mapping
backend also made the result worse, excluding the camera lever arm and
backend map optimization as primary causes.

The baseline 20 deg/s current observations were already mostly geometrically
valid (94.6 percent within 0.25 m of an SDF surface), but 5.4 percent were
outliers and feature coverage changed through the turn. Limiting rendering to
1.5 m improved the pass percentage to 96.1 percent while reducing median
points per cloud from 166.5 to 87 and causing catastrophic pose failure.
Feature count, depth diversity, and continuous angular coverage therefore
matter more than simply clipping distant observations.

Dense v1 added near-field panels at coarse headings. Dense v2 added the
11.25- and 78.75-degree transitions but remained marginal. Dense v3 also
filled 33.75 and 56.25 degrees. The resulting continuous, asymmetric geometry
removed the false-translation floor in both independent runs.

## Root Cause and Scope

For this simulation, the failed turn was caused by insufficient and
discontinuous scene observability during rotation. cuVSLAM stayed in
`vo_state=1`, but tracking health alone did not guarantee an accurate
translation/rotation decomposition.

The following were excluded as primary causes for the accepted stereo-only
path:

- wheel odometry;
- D435i gyro rate, axis, or timestamp integration;
- the 0.19 m camera lever arm and `base_link` output transform;
- map-backend optimization;
- a pure left/right camera calibration error;
- IMU fusion.

This is a simulation-scene correction, not evidence that every real scene
will pass. The real robot must maintain near-field, non-repeating,
three-dimensional visual structure through turns.

## Accepted Baseline and Next Gate

- world:
  `cuvslam_mapping_observable_640x360_range2p5_dense_turn.sdf`;
- turn command: 20 deg/s;
- IMU fusion: disabled;
- mapping: enabled;
- camera offset: 0.19 m;
- wheel odometry: prohibited.

The next test is the controlled `1 m -> 90 deg -> 0.5 m` mapping route in the
dense v3 world. Its linear distance controller must use Gazebo native pose for
test control or another non-wheel source; wheel odometry must not be
reintroduced. Evaluate trajectory first, then current observations and final
landmark geometry.
