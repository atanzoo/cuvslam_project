# D1 Max proxy map-3 dual-target PPO 25k pilot

Date: 2026-09-24 (Asia/Taipei)  
Status: Complete; preliminary simulation result, not policy acceptance.  
Host: local Mac, CPU Stable-Baselines3 PPO + MuJoCo.  
Robot: D1 Max kinematic proxy; pose-hold base, no gait or torque control.

## Experiment identity

- Candidate map: `real_robot/cuvslam/evidence/maps/20260924_092630_current_cuvslam_a2m12/map.yaml` (read-only).
- YAML SHA-256: `be42f16b9730106c54f0bb75e83b04ec48b7c2c69f49e12c4fb4183ca21feb5d`.
- PGM SHA-256: `41e913c872bd118fc971b313024339f78e23247aec94fde00a18e51aa62efd74`.
- Map scale: 3x; cell resolution remains 0.05 m/cell.
- Scenario: 10 m route; one static object plus one pedestrian, balanced over crossing, same-direction, accelerating, turning, and irregular motion with mirrored sides.
- Training: `full_randomization`, seed 42, 50 s episode cap, 32 MPPI samples x 30 horizon, collision penalty 20, timeout penalty 4, PPO `n_steps=256`, batch size 64, learning rate `3e-4`, gamma `0.99`, GAE lambda `0.95`, entropy coefficient `0.01`.
- Requested budget: 25,000 steps; completed at 25,088 because PPO collected a complete 256-step rollout. SB3 reported 2,323 s (38 min 43 s) training elapsed, excluding paired evaluation.
- Checkpoints: saved at 5k, 10k, 15k, 20k, and 25k steps.

## Training trends

Each row counts completed training episodes whose terminal step fell in the
nominal 5k-step window. Rates use completed episodes in that window as the
denominator. These exploratory training outcomes are distinct from the fixed-
seed evaluation below; the rates need not sum to 100% because map violation and
other terminal conditions are separate.

| Training steps | Episodes | Success | Collision | Timeout |
|---:|---:|---:|---:|---:|
| 0–5k | 12 | 0/12 (0.0%) | 4/12 (33.3%) | 5/12 (41.7%) |
| 5k–10k | 12 | 0/12 (0.0%) | 5/12 (41.7%) | 3/12 (25.0%) |
| 10k–15k | 10 | 0/10 (0.0%) | 4/10 (40.0%) | 5/10 (50.0%) |
| 15k–20k | 12 | 0/12 (0.0%) | 5/12 (41.7%) | 5/12 (41.7%) |
| 20k–25k | 11 | 0/11 (0.0%) | 2/11 (18.2%) | 6/11 (54.5%) |

Across the 57 completed training episodes, success remained 0; 20 episodes
ended in collision and 24 in timeout. TensorBoard's rolling mean episode reward
had a low around `-40.4` early in training and improved to `-27.0` at step
25,088, with fluctuations. Reward improvement did not translate into observed
goal completion during training.

## Matched fixed-seed evaluation

The final PPO policy and CRUISE baseline were evaluated on the same 10
deterministically generated map scenarios. Each percentage is `count / 10`.

| Outcome | CRUISE baseline | PPO 25k |
|---|---:|---:|
| Success | 0/10 (0%) | 0/10 (0%) |
| Collision | 4/10 (40%) | 1/10 (10%) |
| Timeout | 1/10 (10%) | 4/10 (40%) |
| Map violation | 5/10 (50%) | 5/10 (50%) |

The same five scenario seeds triggered map violations for both policies. This
points to a scenario/route execution limitation that must be diagnosed; it is
not evidence that the policy solved the route. PPO reduced collisions by three
trials but increased timeouts by three, and neither policy reached a goal.
PPO's valid WAIT choice rate when waiting was allowed was 89.2%; invalid WAIT
rate among WAIT decisions was 54.6%, a diagnostic signal consistent with
excessive stopping but not a causal proof.

Conclusion: the 25k pilot is a useful learning-pipeline and failure-mode
baseline, but the policy is not successful. Diagnose repeated map violations,
WAIT behavior, and the timeout/collision trade-off before another training
budget. This result applies only to this scale-3 offline kinematic proxy; it
does not establish D1 gait, real-map accuracy, ROS/Jetson behavior, navigation
safety, or real-robot performance.

## Artifacts and verification

- Policy: `output/d1_max_proxy_map3_dual_target_balanced_25k_20260924/policy.zip`.
- Policy metadata: `output/d1_max_proxy_map3_dual_target_balanced_25k_20260924/policy.metadata.json`.
- Paired evaluation and per-trial rows: `output/d1_max_proxy_map3_dual_target_balanced_25k_20260924/summary.json`.
- TensorBoard events: `output/d1_max_proxy_map3_dual_target_balanced_25k_20260924/tensorboard/PPO_1/`.
- Resumable checkpoints: `output/d1_max_proxy_map3_dual_target_balanced_25k_20260924/checkpoints/`.
- Python compilation, outcome-window unit assertions, map-3 SB3 reset/step smoke, and map hash/scale checkpoint-compatibility smoke passed.
- No Jetson, ROS runtime, hardware, gait, or deployment was used.
