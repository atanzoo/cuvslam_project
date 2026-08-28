# cuVSLAM Near-Field Observability Design

Date: 2026-07-31
Change class: C1 simulation-only diagnostic world variant

## Evidence

The accepted cold-start 1 m baseline moved native truth `1.0116 m`, while
cuVSLAM moved only `0.0749 m`. All 492 status samples reported `vo_state=1`.

During the 8.5-second truth-motion window:

- all 258 bridged left images had unique pixel hashes;
- all 258 bridged right images had unique pixel hashes;
- all 243 NVIDIA debug left images had unique hashes;
- all 243 NVIDIA debug right images had unique hashes;
- all bridged stereo timestamps matched exactly.

The failure is therefore not caused by stale pixels in this run. The rendered
view is dominated by low-texture grey surfaces and a far wall. At the declared
approximately 208 px focal length and 0.05 m baseline, features 3.6–4.6 m away
produce only about 2–3 px of disparity.

## Option A: Independent near-field observable world

Generate `cuvslam_mapping_observable.sdf` from the locked baseline and add
visual-only, asymmetric, high-contrast near-field boards beside every leg of
the square mapping route, plus sparse floor markers at varied depths.

Advantages:

- preserves camera, TF, CameraInfo, timing, estimator, and baseline world;
- provides larger stereo disparity and forward-motion parallax;
- supports a direct one-variable A/B test;
- rollback is removal of one generated world file.

Disadvantages:

- proves simulation scene observability, not real-camera accuracy;
- the diagnostic geometry is deliberately easier than an unprepared room.

## Option B: Change camera or estimator parameters

Increase baseline/resolution, reduce frame rate, enable IMU fusion, force
planar mode, or alter cuVSLAM parameters.

Advantages:

- may increase apparent motion or constrain the estimator.

Disadvantages:

- changes multiple locked contracts;
- does not explain why the official NVIDIA bag works;
- would make the result incomparable with the accepted baseline;
- risks hiding the actual scene-observability limitation.

## Decision

Use Option A. Add a new world variant only. Do not modify
`cuvslam_mapping_simple.sdf`, camera geometry, frame ownership, timing,
CameraInfo, IMU fusion, or cuVSLAM parameters.

## Acceptance

- generated SDF parses successfully;
- baseline SHA-256 remains unchanged;
- D435i, `/scan`, `/clock`, `/cmd_vel`, and truth topics remain unchanged;
- every square-route leg has near-field features on both sides;
- cold-start truth motion is `0.95–1.05 m`;
- every in-motion stereo frame is non-stale and timestamp-paired;
- cuVSLAM translation is `0.95–1.05 m`;
- orientation error is at most `15 deg`;
- all processes stop and zero velocity is published after the test.
