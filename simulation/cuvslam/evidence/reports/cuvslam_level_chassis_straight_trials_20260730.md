# cuVSLAM Level-Chassis Straight Trials

Date: 2026-07-30  
Environment: Jetson AGX Orin simulation, ROS domain 43  
Estimator: Isaac ROS Visual SLAM 2.1.0, cuVSLAM 11.4  
Camera: simulated D435i stereo, 424 x 240 x 30 Hz  
IMU fusion: false  
Final runtime state: Simulation stopped

## Change

The original robot had both wheel contacts at model `z=-0.06 m`, while the
rear caster bottom was at `z=-0.05 m`. Lowering only the caster did not change
the measured `13.8 deg` pitch because the aggregate center of mass remained
slightly in front of the `x=0` wheel axle and outside the rear-caster support
polygon.

The accepted simulation-only correction:

- places the caster bottom on the wheel contact plane;
- moves the 8 kg chassis inertial center from `x=0` to `x=-0.01 m`;
- does not change TF, sensors, visible geometry, wheel radius, CameraInfo,
  image rate, or cuVSLAM parameters.

## Static Acceptance

Before:

```text
position=(-1.78554, -1.80000, 0.06000)
pitch=approximately 13.8 deg
```

After:

```text
position=(-1.80000, -1.80000, 0.06000)
orientation quaternion approximately (0, 0, 0, 1)
10-second drift=0 within reported precision
```

The physical leveling correction passed.

## Straight Trials

Each trial restarted Gazebo and cuVSLAM, retained the original 30 Hz
stereo-only estimator, and ran the same 5-second `0.12 m/s` diagnostic profile.
The motion controller did not subscribe to ground truth.

| Trial | Native 6DoF distance | cuVSLAM distance | Distance error | Native Z | False rotation |
|---|---:|---:|---:|---:|---:|
| 1 | 0.4596 m | 0.0624 m | 86.42% low | approximately 0 | 1.31 deg |
| 2 | 0.4506 m | 0.0740 m | 83.58% low | approximately 0 | 1.80 deg |
| 3 | 0.4200 m | 0.0567 m | 86.50% low | approximately 0 | 1.41 deg |

Weighted distance underestimation was `85.48%`. All status samples remained
`vo_state=1`, so tracking state alone did not reveal the failure.

## Decision

Keep the chassis center-of-mass and caster-plane correction because it removes
the proven physical pitch and vertical-motion defect.

Do not accept cuVSLAM straight motion. None of the three trials met the required
`15%` distance error or `1 deg` false-rotation limits.

The result narrows the next diagnosis:

- TF was unchanged and is not the tested variable;
- native vehicle motion is now planar and correct;
- the estimator receives synchronized, calibrated stereo but observes only
  about `13%` to `16%` of the true forward displacement;
- the next controlled test must measure horizontal straight-run optical flow,
  tracked feature count/distribution, and native camera pose together.

No IMU, TF, scale multiplier, or post-estimation correction should be added
before that image-observability measurement.

## Evidence

- `docs/GAZEBO_CASTER_CONTACT_PLANE_FIX_DESIGN.md`
- `deployment/slam_gazebo/worlds/cuvslam_mapping_simple.sdf`
- `reports/caster_com_fix_20260730/trial1/`
- `reports/caster_com_fix_20260730/trial2/`
- `reports/caster_com_fix_20260730/trial3/`

Local validation:

```text
SDF XML: valid
Related contract tests: 14 passed
```

## Final Shutdown

```text
cuvslam-caster-bridge.service: inactive
cuvslam-caster-gazebo.service: inactive
cuVSLAM launch/component/helper processes: none
/clock: no message during a 3-second sample
```
