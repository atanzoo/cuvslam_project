# Frame Contract Motion Audit

Date: 2026-07-30  
Environment: Jetson AGX Orin simulation, ROS domain 43  
cuVSLAM: Isaac ROS Visual SLAM 11.4, stereo only, IMU fusion disabled  
Data source: existing corrected-right-CameraInfo bags  
Change class: C1 offline diagnostic; no runtime changes

## Purpose

Determine how much of the reported turn translation is caused by comparing
Gazebo `slam_bot/base_link` directly with cuVSLAM
`camera_infra1_optical_frame`.

The new evaluator applies the declared static transform:

```text
base_link -> camera_infra1_optical_frame
translation in base axes: (0.19, 0.025, 0.20) m
body-to-optical rotation: X right, Y down, Z forward
```

It reports:

- ground-truth base motion;
- predicted ground-truth left-optical motion;
- raw cuVSLAM left-optical motion;
- cuVSLAM motion transformed back to the base reference point;
- residual motion.

Design:
`docs/FRAME_AWARE_MOTION_DIAGNOSTIC_DESIGN.md`

Tool:
`tools/evaluate_frame_contract_bag.py`

## Verification

Local checks:

```text
python3 -m unittest test_frame_contract_math.py
...
Ran 3 tests
OK
```

Jetson command pattern:

```bash
python3 evaluate_frame_contract_bag.py <experiment>/bag
```

The evaluator was run inside the existing Humble Isaac ROS container. Bags
were opened read-only. Endpoint timestamp skew was at most 20 ms.

## Static Result

Bag:

```text
camera_info_baseline_20260729_0955/static/bag
```

| Metric | Result |
|---|---:|
| Ground-truth base translation | 0.0000 m |
| Predicted left-optical translation | 0.0000 m |
| cuVSLAM translation | 0.0000 m |
| Ground-truth rotation | 0.00 deg |
| cuVSLAM rotation | 0.00 deg |

Static remains a valid zero-motion control.

## Straight Result

Bag:

```text
camera_info_baseline_20260729_0955/straight/bag
```

| Motion | Translation vector (m) | Norm | Rotation |
|---|---|---:|---:|
| Ground-truth `base_link` | `(+0.2270, 0, 0)` in body axes | 0.2270 m | 0.00 deg |
| Predicted left optical | `(0, 0, +0.2270)` in optical axes | 0.2270 m | 0.00 deg |
| cuVSLAM declared left optical | `(+0.2711, +0.0231, -0.0626)` | 0.2792 m | 6.75 deg |

The expected forward direction in a ROS optical frame is `+Z`. The cuVSLAM
odometry topic instead reports its dominant straight component on `+X`, while
its child frame is named `camera_infra1_optical_frame`.

This is direct evidence that the output frame name and output numeric axis
semantics do not currently satisfy the Frame Contract. It is consistent with
a canonical/body-frame output being labelled as an optical frame.

Because of this axis-contract failure, component-wise transformed residuals
must not yet be interpreted as estimator accuracy. Translation norm and
rotation magnitude remain useful invariant diagnostics.

## Turn Result

Bag:

```text
camera_info_baseline_20260729_0955/turn/bag
```

| Motion | Translation norm | Rotation |
|---|---:|---:|
| Ground-truth base | 0.0000 m | 36.73 deg |
| Predicted left-camera lever-arm arc | 0.1207 m | 36.73 deg |
| cuVSLAM output | 1.1374 m | 9.04 deg |
| cuVSLAM output converted using declared frame contract | 1.1304 m | 9.04 deg |

The camera should move because it is approximately 0.192 m from the chassis
yaw axis in the horizontal plane. For a 36.73 degree turn, the expected camera
arc chord is about 0.1207 m.

The observed cuVSLAM translation is about 9.4 times that expected arc:

```text
1.1374 / 0.1207 = 9.42
```

Therefore direct base-versus-camera comparison did overstate the error, but
the physical lever arm explains only a small part of the observed translation.
The turn result still contains a large translation-magnitude error and a large
rotation underestimate. These conclusions are invariant to an axis
permutation.

## Findings

1. The historical evaluator was frame-unaware and should not be used alone to
   label turn translation as false motion.
2. The corrected comparison predicts 0.1207 m of legitimate camera motion,
   not zero.
3. A remaining observed magnitude of approximately 1.14 m cannot be explained
   by the camera mounting offset.
4. The cuVSLAM output topic is labelled as an optical child frame but behaves
   like a body/canonical frame during straight motion.
5. Foxglove is not the source of this mismatch; it consumes the inconsistent
   TF/message contract.
6. This evidence does not identify a cuVSLAM algorithm defect. It identifies
   an unresolved simulator-to-cuVSLAM input/output frame contract.

## Decision

Do not tune mapping, planar constraints, or navigation parameters yet.

The next controlled C2 experiment must establish an output frame whose name
and axes agree. The experiment must preserve the current corrected CameraInfo
and compare:

1. cuVSLAM output normalized to `base_link`; and
2. left-optical estimator output converted by a separate pose adapter that
   explicitly accounts for cuVSLAM's canonical basis.

Acceptance requires:

- straight body motion appears on `base_link +X`;
- straight optical motion appears on optical `+Z`;
- a 30-40 degree chassis turn produces the predicted camera lever-arm arc;
- inferred base translation during an in-place turn is near zero;
- one publisher owns each authoritative TF edge.

