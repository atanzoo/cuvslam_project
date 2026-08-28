# Gazebo Caster Contact-Plane Fix

Date: 2026-07-30  
Change class: C2 simulation geometry  
Deployment target: `/home/tseng/jetson_slam_ws` only

## Evidence

The two drive-wheel centers are at model `z=0` with radius `0.06 m`, so their
contact plane is model `z=-0.06 m`. The caster center is currently at
`z=-0.025 m` with radius `0.025 m`, placing its bottom at model `z=-0.05 m`.
It is therefore `0.01 m` above the wheel contact plane.

Gazebo-native 6DoF truth measured approximately `13.8 deg` of static pitch.
During a nominal straight run, native truth moved `0.1058 m` vertically while
moving `0.4305 m` horizontally, consistent with motion along the pitched body
axis.

## Options

### Option A: Lower the caster by 0.01 m

Set the caster center to `z=-0.035 m`. Its bottom then lies at model
`z=-0.06 m`, matching both drive wheels.

Advantages:

- changes only the proven geometric mismatch;
- preserves wheel radius, model origin, sensor extrinsics, and TF;
- easy to roll back.

Disadvantages:

- a fixed spherical caster remains an approximation of real caster dynamics.

### Option B: Raise the wheels or alter the complete model pose

Advantages:

- could also make all contacts coplanar.

Disadvantages:

- changes wheel odometry geometry or every world-relative sensor height;
- expands the test surface without evidence that those values are wrong.

## Decision

Select Option A. Do not change TF, camera calibration, image rate, cuVSLAM
parameters, wheel radius, or model origin.

## Acceptance

Before motion:

- absolute roll and pitch below `1 deg`;
- native 6DoF drift below `0.005 m` over 10 seconds.

For three independent straight trials:

- cuVSLAM distance relative error below `15%` each;
- native vertical displacement below `0.02 m`;
- cuVSLAM false rotation below `1 deg`;
- tracking remains `vo_state=1`.

If static leveling fails, revert immediately. If leveling passes but cuVSLAM
acceptance fails, keep the geometry correction only if it removes the proven
physical defect, and continue visual-translation diagnosis separately.

## Option A Result

Rejected. Gazebo-native truth remained unchanged at approximately `13.8 deg`
pitch after lowering the caster. The model still settled to the same position
and quaternion.

The support polygon explains the result. Both wheel contacts lie on `x=0`, the
caster is behind them at `x=-0.13 m`, and the aggregate center of mass is
slightly in front of the wheel axle because the camera assembly is at
`x=0.19 m`. Lowering the rear caster cannot support a center of mass outside
the polygon.

## Follow-Up Options

### Option B: Move the chassis inertial center rearward

Keep the rear caster, lower it to the wheel plane, and set the 8 kg chassis
inertial center to `x=-0.01 m`. This places the aggregate center of mass inside
the existing rear-caster support polygon without changing visible geometry,
wheel odometry, sensors, or TF.

### Option C: Move the caster in front of the wheel axle

Moving the caster to `x=+0.13 m` also encloses the current center of mass, but
changes the robot's physical layout.

Select Option B for the next static-only trial because it preserves the current
rear-caster architecture. Apply the original acceptance gates before motion.

## Option B Result

Accepted as a simulation-physics correction:

- static pitch changed from approximately `13.8 deg` to effectively `0 deg`;
- the model settled at the declared `(-1.8, -1.8, 0.06)` pose;
- native 6DoF pose was unchanged over the following 10 seconds;
- three straight trials had effectively zero native vertical displacement.

Rejected as a complete cuVSLAM correction. The three straight distance
estimates were `83.58%` to `86.50%` below native 6DoF truth and false rotation
remained `1.31 deg` to `1.80 deg`.

The geometry change remains because it removes a proven physical defect.
Further estimator diagnosis must retain the level chassis and investigate
horizontal-motion image observability as a separate variable.
