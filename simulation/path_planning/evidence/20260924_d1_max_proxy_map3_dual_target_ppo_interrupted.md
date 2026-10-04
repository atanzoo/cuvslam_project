# D1 Max proxy map-3 dual-target PPO attempt (interrupted)

Date: 2026-09-24  
Status: Interrupted at the user's request after an explicit review of the
runtime estimate. This is a throughput/preflight record, not a trained-policy
result.

## Scope

- Host: local Mac workstation; CPU SB3 PPO and MuJoCo.
- Robot model: D1 Max kinematic proxy (pose-hold base); no D1 gait.
- Map: `real_robot/cuvslam/evidence/maps/20260924_092630_current_cuvslam_a2m12/map.yaml`, read-only.
- Scene: map scale 3; each episode uses a 10 m route, one fixed object, and one
  mirrored/balanced pedestrian from crossing, same-direction, accelerating,
  turning, or irregular motion.
- PPO settings: `full_randomization`, seed 42, intended 300,000 steps, 50 s
  train/eval episodes, MPPI 32 samples x 30 horizon, collision penalty 20,
  timeout penalty 4, `n_steps=256`, batch 64, learning rate 3e-4, gamma 0.99,
  GAE lambda 0.95, entropy coefficient 0.01, checkpoint every 25,000 steps.
- Candidate map hashes: YAML
  `be42f16b9730106c54f0bb75e83b04ec48b7c2c69f49e12c4fb4183ca21feb5d`; PGM
  `41e913c872bd118fc971b313024339f78e23247aec94fde00a18e51aa62efd74`.

## Preflight evidence

- Focused D1 SB3 smoke suite passed, including scale-3 identity, exact 10 m
  route, two targets, five pedestrian families, mirrored placement, full
  episode known-free checks, and reset/step integration.
- 100 deterministic generated episodes passed; five modes were cycled across
  both mirror directions. One candidate route needed deterministic retry;
  total scene-generation time was about 5.9 s for 100 resets.
- Disk preflight: 28 GiB free; the unique experiment output path did not exist.

## Interrupted run

Training reached 6,400 / 300,000 steps. The last SB3 output reported 20 FPS,
311 seconds elapsed, mean episode length 402, and mean reward -33.4. Observed
throughput was approximately 20.6 steps/s. The resulting linear estimates are
about 20 min for 25k, 41 min for 50k, 1 h 21 min for 100k, and 4 h 03 min for
300k PPO training, before roughly 7-12 min for the planned paired 10-trial
evaluation.

The process was stopped with `KeyboardInterrupt` (exit 130) at the user's
request. Since the first checkpoint was scheduled at 25,000 steps, this run
produced no resumable policy, checkpoint, metadata, or final summary. The sole
preserved output is:

`output/d1_max_proxy_map3_dual_target_balanced_300k_20260924/tensorboard/PPO_1/events.out.tfevents.1790237060.MacBook-Pro.local.76153.0`

No success, collision, safety, navigation, or learning-quality conclusion can
be drawn from these first 6,400 training steps. No Jetson, ROS runtime, or
physical robot was involved. Do not restart until the user selects a new step
or time budget.
