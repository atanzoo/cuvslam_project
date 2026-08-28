# cuVSLAM Stage 3 Image Timing And Motion Sampling

- Date: 2026-07-30
- Environment: Jetson simulation workspace, ROS domain 43
- Change class: C1 diagnostic

## Objective

Determine whether camera cadence, stereo synchronization, rendered intensity,
Gazebo real-time factor, motion sampling, or cuVSLAM processing time explains
the remaining false Z, false translation, and rotation error.

Do not modify intrinsics, baseline, TF, stereo topic ordering, world geometry,
or cuVSLAM input assumptions in this stage.

## Procedure

Measure nine-second windows for:

1. stationary;
2. five-second straight motion at `0.12 m/s`;
3. five-second in-place turn at `-0.12 rad/s`.

The monitor subscribes to both images, `/clock`, `/cmd_vel`, and
`/visual_slam/status`. Motion remains owned by the established safety profile,
which also watches `/scan` and tracking state.

## Acceptance Gates

- Each image stream: `28.5-31.5 Hz`.
- Maximum image stamp interval: `<=50 ms`.
- Exact left/right pair fraction: `>=99%`.
- Maximum nearest stereo stamp skew: `<=1 ms`.
- Gazebo real-time factor: `0.95-1.05`.
- Median image mean intensity: `10-245`.
- Median image standard deviation: `>=10`.
- Every cuVSLAM status sample: `vo_state=1`.
- Maximum cuVSLAM track time: `<33.3 ms`.
- Static optical-flow P95: `<=0.25 px`.

Motion optical flow, arrival jitter, clipping fractions, and temporal intensity
changes are recorded as diagnostic metrics. They are not independently treated
as trajectory-accuracy proof.

## Failure Actions

- Cadence or RTF failure: isolate Gazebo load and update rate before changing
  cuVSLAM.
- Pair loss or skew: fix bridge/sensor publication timing without changing
  calibration.
- Frame-budget failure: reduce processing load or input rate in a controlled
  experiment.
- Exposure/contrast failure: modify only simulation lighting/material input in
  an independent world.

## Controlled Rate Experiment

The 30 Hz baseline failed the RTF and occasional frame-budget gates. A static
measurement without cuVSLAM improved RTF but still remained below real time,
showing that both Gazebo rendering and cuVSLAM contribute load.

Two minimal options were considered:

1. Create a temporary 20 Hz stereo-camera variant while retaining image size,
   geometry, world, LiDAR, and all calibration contracts.
2. Reduce LiDAR resolution or remove a sensor, which would change another
   subsystem and weaken the safety profile.

Option 1 is selected as the controlled Stage 3 experiment. The original world
is not modified; the derived SDF is temporary and selected only through the
existing `world` launch argument.

## Experiment Decision

The 20 Hz variant passed every timing gate but substantially regressed
trajectory behavior, including multi-meter false translation during an
in-place turn. It is therefore rejected and must not replace the original
30 Hz contract. The temporary SDF is an experiment artifact only.

## Rollback

Remove the monitor, unit test, and this note. No runtime interface is changed.
