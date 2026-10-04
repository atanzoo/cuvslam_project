# cuVSLAM turn-pose fix — 2026-08-03

## Decision

The full route now has a usable turn orientation estimate with stereo-only cuVSLAM and a 20 deg/s IMU-gated command. The strict false-translation gate is not yet passed, so this is suitable for exploratory mapping validation, not the final pose acceptance baseline.

Wheel odometry was not used. Gazebo native pose and D435i IMU were recorded only as offline references/control signals.

## What failed

The first attempt added an independent static feature cluster with collision geometry at the route corner. Its first infrared frame became almost black (0–111 gray level, only 19 distinct values), and cuVSLAM stayed in `Visual tracking is lost`. Removing that cluster restored the original rich image and `vo_state=1`.

The scene was therefore changed to visual-only corner boards inside the existing observability model. They add asymmetric features around the actual corner without collision geometry or a new physics object.

## Stereo-only 20 deg/s full route

Bag: `corner_pose_only_20260803_1157_visual`

Route truth was `1.0011 m → 90.04 deg → 0.5013 m`; `vo_state=1` for all 849 estimate samples.

| Phase | Truth yaw | cuVSLAM yaw | Yaw error | SE(3) translation residual |
|---|---:|---:|---:|---:|
| First leg | +0.98° | +2.40° | +1.42° | 6.28 cm |
| Turn | +88.15° | +83.56° | **−4.60°** | **12.59 cm** |
| Second leg | +2.36° | +2.38° | +0.02° | 8.08 cm |
| Complete route | +91.49° | +88.34° | **−3.15°** | 16.59 cm |

Trajectory errors were median `6.95 cm` translation and `6.65°` rotation; p95 was `16.59 cm` and `8.40°`. The turn-yaw gate of ±5° passes. The stricter 5 cm false-translation gate does not.

## Speed and IMU A/B

- `10 deg/s`, stereo-only: turn yaw error `+42.56°`, translation residual `46.16 cm`; rejected.
- `20 deg/s`, stereo-only: best result above; keep as the route default.
- `20 deg/s`, IMU fusion enabled: turn yaw error `−12.07°`, translation residual `15.59 cm`, roll/pitch maxima about `18.28°/17.25°`; rejected for this simulated extrinsic/data contract.

## Consequence

The original catastrophic turn failure (`about −75°`) is fixed by placing non-colliding corner features in the active SDF. Before formal landmark-map acceptance, either the 12.59 cm turn translation residual must be reduced, or the project must explicitly adopt a looser exploratory pose gate. The pose-only bags do not contain landmark geometry, so no new map-quality percentage is claimed from this run.

