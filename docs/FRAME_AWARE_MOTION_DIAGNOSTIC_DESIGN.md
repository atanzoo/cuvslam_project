# Frame-aware Motion Diagnostic Design

Status: Approved for local diagnostic implementation  
Change class: C1 Local implementation  
Date: 2026-07-30

## Problem

`simulation/cuvslam/tools/evaluate_simulation_bag.py` reports the independent endpoint
displacements of `/ground_truth/odom` and cuVSLAM pose topics. Ground truth
describes `slam_bot/base_link`, while the current cuVSLAM launch describes
`camera_infra1_optical_frame`.

During chassis rotation, the camera follows a physical arc because its origin
is offset from `base_link`. Comparing the two raw displacements labels this
valid lever-arm motion as false translation.

## Selected Option

Add a separate offline evaluator that:

1. reads ground-truth base pose and cuVSLAM camera odometry from a fixed bag;
2. applies the Frame Contract `base_link -> left optical` extrinsic;
3. predicts ground-truth camera motion;
4. converts cuVSLAM camera motion back to an inferred base motion; and
5. reports residual translation and rotation in like-for-like frames.

Advantages:

- preserves historical evaluator output;
- changes no runtime topic, TF edge, estimator parameter, or deployment;
- makes the reference point and static extrinsic explicit;
- can be replayed against all existing experiment bags.

Disadvantages:

- the default extrinsic is specific to the current simulation SDF;
- endpoint comparison does not replace full trajectory alignment;
- timestamp matching is nearest-sample rather than interpolation.

Failure modes:

- a bag with no overlapping timestamps is rejected;
- missing topics or fewer than two samples are rejected;
- observed message frame IDs are printed so a mismatched bag is visible;
- zero-length quaternions are rejected.

Rollback:

- delete the new evaluator and math helper; no runtime state is affected.

Verification:

- unit-test transform composition, optical axes, and pure-yaw lever-arm motion;
- run against the corrected-baseline turn bag;
- compare the predicted camera arc with the raw cuVSLAM translation;
- retain the command and output in a report.

## Rejected Option

Modify `evaluate_simulation_bag.py` in place.

This would make old command output non-comparable with existing reports and
silently change the meaning of previously used metrics. A separate diagnostic
is safer while the frame contract is still being validated.
