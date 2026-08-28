# cuVSLAM Stage 2 Stereo Geometry Audit

- Date: 2026-07-30
- Environment: Jetson simulation workspace, ROS domain 43
- Change class: C1 diagnostic

## Objective

Measure whether corresponding structures in the live left and right simulated
images have positive horizontal disparity and remain on the same image row.
Do not modify the Stage 1 intrinsics, TF, timing, exposure, or cuVSLAM
parameters.

## Method

Subscribe to exact-timestamp left/right pairs and compute ORB descriptors.
Retain only matches that pass a 0.75 nearest-neighbor ratio test in both
directions. KAZE with L2 descriptors is available as an independent
subpixel-localized cross-check when ORB pyramid quantization is close to the
vertical-residual gate. For every pair, report:

- left and right keypoint counts;
- mutually consistent match count;
- median and P10/P90 horizontal disparity;
- median and P95 absolute vertical residual;
- fraction of matches with positive disparity.

The first pair is saved as an annotated match image.

## Acceptance

- At least 10 valid image pairs.
- At least 15 mutual matches per valid pair.
- Worst per-pair P95 vertical residual no greater than 1 pixel.
- Every valid pair has at least 90 percent positive disparity.

These gates verify image geometry only. They do not prove depth accuracy or
cuVSLAM trajectory accuracy.

## Failure Actions

- Vertical residual failure: inspect camera optical rotations and whether the
  images are actually rectified before changing calibration.
- Disparity polarity failure: inspect left/right topic ordering and physical
  baseline direction.
- Insufficient matches: add diagnostic texture to the simulation world in a
  separate world-only change; do not loosen geometry gates blindly.

## Rollback

Remove the validator, its unit test, and this note. The diagnostic publishes no
ROS topic and changes no runtime contract.
