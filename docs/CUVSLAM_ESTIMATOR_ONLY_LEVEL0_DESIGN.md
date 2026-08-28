# cuVSLAM Estimator-Only Level 0 Design

Date: 2026-07-28 (Asia/Taipei)
Change class: C4 simulation `/cmd_vel` test workflow

## Objective

Exercise simulated D435i stereo cuVSLAM using a repeatable low-speed motion
profile that resembles the intended real architecture. Gazebo ground truth
must not control motion, correct TF, or enter cuVSLAM.

## Selected Option: Command Profile

One simulation-only node owns `/cmd_vel` and executes four equal straight
segments separated by equal clockwise turns.

Inputs:

- `/visual_slam/status`: stop if `vo_state != 1`;
- `/scan`: stop if the scan is stale or an obstacle is inside the declared
  clearance.

Outputs:

- `/cmd_vel`: bounded linear and angular velocity.

Ground truth is not a node input. `/ground_truth/odom` may be recorded in the
bag and read only after the run for evaluation.

Advantages:

- isolates cuVSLAM estimation from route control;
- matches a teleoperation or scripted commissioning run;
- avoids circularly using the estimator to create its own reference route;
- has one command owner and deterministic zero-command cleanup.

Disadvantages:

- the physical route need not close exactly;
- acceleration and wheel dynamics affect the executed shape.

Failure behavior:

- publish zero velocity for tracking loss, stale scan, insufficient clearance,
  interruption, exception, or normal completion.

Rollback:

- stop the node; it is not installed in a launch or systemd service.

## Rejected For This Stage: cuVSLAM Waypoint Feedback

The controller would subscribe to `/visual_slam/tracking/odometry` and use it
to reach square waypoints.

Advantages:

- resembles a future localization-driven navigation controller;
- can close a commanded route without Gazebo truth.

Disadvantages:

- the current run showed about 3 m of cuVSLAM closure error;
- estimator drift would directly steer the robot and confound diagnosis;
- using the estimator for both control and evaluation creates correlated
  errors.

This option is deferred until estimator-only trajectory quality passes.

## Data Contract

```text
simulated stereo images -> cuVSLAM -> /visual_slam/tracking/odometry
                                  -> /visual_slam/tracking/slam_path

motion profile -> /cmd_vel -> Gazebo robot
/scan + /visual_slam/status -> motion safety stop only

/ground_truth/odom -> rosbag -> offline evaluation only
```

## Verification

1. Restart Gazebo and cuVSLAM from a clean state.
2. Confirm `/cmd_vel` has zero publishers before the run.
3. Confirm the controller process has no ground-truth subscription.
4. Record camera, scan, commands, cuVSLAM outputs, TF, and ground truth.
5. Confirm every status sample and final zero velocity.
6. Evaluate truth and cuVSLAM trajectories only after recording stops.

### Drift Isolation Sequence

When the full profile fails, restart Gazebo and cuVSLAM and record three
separate stages in one estimator session:

1. stationary baseline;
2. one short straight command;
3. one in-place turn.

The motion node may expose `straight` and `turn` diagnostic modes, but its ROS
contract and safety inputs remain unchanged. Each bag is evaluated for 2D/3D
translation, Z drift, rotation, tracking state, camera calibration, image
timestamp pairing, and relevant TF. Ground truth remains recorder-only.
