# cuVSLAM Stage 2 Stereo Geometry Audit

Date: 2026-07-30  
Environment: Jetson AGX Orin simulation, ROS domain 43  
Result: Stereo disparity and epipolar geometry passed; trajectory still failed  
Final runtime state: Simulation stopped

## Scope

This stage measured live left/right image geometry while retaining the Stage 1
FOV, K/P, baseline projection, TF, camera rate, exposure, and cuVSLAM
parameters.

The diagnostic used exact-timestamp pairs and mutual two-direction descriptor
matching. Ground truth was not available to the image validator and was
recorded only for offline trajectory evaluation.

## Baseline World

The baseline `indoor_gz_sim.sdf` image contained mostly flat floor and wall
surfaces. Across 20 exact-timestamp pairs, ORB produced:

```text
matches per pair=14
median disparity=8.0525 px
positive disparity fraction=100%
median absolute vertical residual=0 px
worst per-pair vertical P95=0.2980 px
```

KAZE independently measured:

```text
matches per pair=5
median disparity=8.1807 px
positive disparity fraction=100%
worst per-pair vertical P95=0.7881 px
```

Both methods agree that disparity polarity and epipolar alignment are correct.
The predeclared 15-match gate failed because the baseline view has too few
distinct corners, not because vertical geometry or left/right ordering failed.

## High-Contrast Cross-Check

The existing independent `cuvslam_mapping_simple.sdf` world was selected
through the existing `world` launch argument. The baseline world was not
modified.

ORB across 20 pairs:

```text
minimum/median matches=67/67
median disparity=2.4883 px
positive disparity fraction=100%
worst vertical P95=1.0080 px
```

ORB exceeded the 1-pixel gate by `0.008 px`, consistent with its pyramid
keypoint quantization. KAZE's subpixel-localized independent result passed:

```text
minimum/median matches=19/19
median disparity=2.4806 px
positive disparity fraction=100%
worst vertical P95=0.2579 px
status=PASS
```

This proves, under a view with sufficient observable texture, that
correspondences have positive disparity and remain on the same epipolar row.

## Runtime Verification

cuVSLAM reached `vo_state=1`. The established safety runner then recorded
timestamp-aligned static, straight, and turn bags. The controller subscribed
only to `/scan` and `/visual_slam/status`; minimum scan clearance remained at
least `0.739 m`.

### Static

```text
truth drift=0
cuVSLAM drift=0
endpoint skew=-5 ms / -5 ms
```

### Straight

```text
ground truth=0.4110 m
cuVSLAM 3D distance=0.5695 m
3D relative error=38.6%
cuVSLAM horizontal distance=0.5282 m
horizontal relative error=28.5%
false Z=0.2130 m
false rotation=4.90 deg
endpoint skew=+10 ms / -10 ms
```

### Turn

```text
ground-truth rotation=28.81 deg
cuVSLAM rotation=33.46 deg
rotation magnitude error=16.1%
false base translation=0.3865 m
endpoint skew=-5 ms / +15 ms
```

The high-contrast world is a different route and cannot be used as a strict
A/B accuracy comparison with the previous baseline trial. It does show that
increasing match count and passing the image-geometry gate is not sufficient
to eliminate false translation, false Z, or rotation error.

## Stage Decision

The hypotheses of swapped stereo topics, negative disparity, or material
vertical epipolar misalignment are rejected. No stereo geometry correction is
justified.

The baseline world has poor visual texture, so it should not be the sole
mapping-quality acceptance scene. The independent high-contrast world remains
available without altering the baseline.

Proceed to Stage 3: quantify image cadence, left/right timestamp pairing,
exposure behavior, dropped frames, and motion sampling under static,
translation, and rotation.

## Evidence

- `tools/validate_stereo_geometry.py`
- `tools/test_validate_stereo_geometry.py`
- `docs/CUVSLAM_STAGE2_STEREO_GEOMETRY_DESIGN.md`
- `reports/stage2_stereo_left.png`
- `reports/stage2_stereo_matches.png`
- `reports/stage2_textured_left.png`
- `reports/stage2_textured_matches.png`
- Jetson experiment:
  `/workspaces/isaac_ros-dev/data/experiments/stage2_stereo_geometry_20260730`

Local tests:

```text
Ran 4 tests
OK
```

## Final Shutdown Evidence

```text
cuvslam-stage2-gazebo.service: inactive
cuvslam-stage2-bridge.service: inactive
cuvslam-d435i-gazebo.service: inactive
cuvslam-d435i-bridge.service: inactive
cuVSLAM PID count: 0
rosbag/profile PID count: 0
```

After shutdown, DDS temporarily retained discovered topic names, but `/clock`
produced no message during a 3-second sample and no Gazebo or parameter bridge
process remained.
