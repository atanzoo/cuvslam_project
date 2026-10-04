# D1 Max Proxy Map-3 Dual-Target PPO Experiment Plan

Date: 2026-09-24  
Status: 25k-step pilot completed; preliminary reward improvement did not produce goal successes, and matched evaluation exposed a timeout/collision trade-off plus map violations.  
Change class: C3 algorithm / training-distribution experiment.

## Objective and boundaries

Train a new 300,000-step SB3 PPO high-level decision policy in the local
MuJoCo D1 Max kinematic proxy on the read-only candidate cuVSLAM map at 3x map
scale. Each 10 m episode will contain one fixed obstacle and one pedestrian.
The pedestrian family will be balanced across crossing, same-direction,
accelerating, turning, and irregular motion, with mirrored left/right layouts.

This is an offline proxy experiment only. It does not establish D1 Max gait,
real-robot navigation, map accuracy, or safety acceptance. It will not modify
the source map, Jetson, robot, ROS runtime, protected configuration, or existing
checkpoints. The prior D1 Edu checkpoint is not a compatible initialization.

## Alternatives and selection

1. **Single crossing mode plus fixed obstacle.** Lowest implementation and
   validation cost; useful as a narrow smoke test, but weak coverage of the
   user's requested motion diversity and susceptible to learning one encounter
   pattern.
2. **Balanced five-family pedestrian plus fixed obstacle (selected).** Better
   distribution coverage and left/right symmetry, but needs trajectory and
   known-free-map validation across modes and seeds; the resulting policy will
   be more costly to train and evaluate.

The user selected alternative 2 and explicitly chose the balanced pedestrian
schedule. The fixed obstacle remains present in every episode.

## Compatibility and implementation scope

Expected code scope: `simulation/path_planning/tools/d1_edu_sb3_env.py` and its
existing smoke test `simulation/path_planning/tools/test_d1_edu_decision_layer_sb3.py`.
The change remains opt-in through the existing candidate-map/D1 Max proxy path;
non-map D1 Edu scenarios and the existing observation/action interface remain
unchanged. MuJoCo must be initialized with exactly two target bodies on the
first map episode and keep that body count for subsequent resets.

Scale 3 changes map geometry only. D1 proxy and actor footprints retain their
configured physical dimensions. Map YAML/PGM bytes and prior model artifacts
remain read-only.

## Baseline and comparison

Training baseline: the existing high-level CRUISE policy with no learned PPO
decision overrides, evaluated on the same generated episode seeds as PPO.
The previous research recipe used a fresh 300,000-step PPO run, full
randomization, 10 m routes, 50 s episodes, MPPI 32 samples x 30 horizon,
collision penalty 20, timeout penalty 4, `n_steps=256`, batch size 64, learning
rate 3e-4, gamma 0.99, GAE lambda 0.95, entropy coefficient 0.01, and 25,000
step checkpoints. This experiment preserves that recipe where compatible and
uses a new map/scale-specific output identity.

Preliminary, non-acceptance probe: a temporary scale-3/two-target wrapper ran a
50 s CRUISE episode, timed out 1.146 m from goal, had minimum geometric
clearance -0.00319 m, and reported no physical contact or map violation. This
single prototype episode is only a baseline signal, not a result for the final
balanced generator.

## Risks and failure modes

- Actor paths may leave inflated known-free space or overlap the fixed object;
  reject such generated episodes before training rather than weakening map
  constraints.
- A turning or irregular trajectory can be valid at spawn but invalid later;
  validate its full 50 s sampled trajectory against the map.
- Building a one-target MuJoCo model and later adding a second target would
  invalidate joint indexing; keep the two-target topology constant from reset 1.
- A 300k run may take hours and create checkpoints/TensorBoard output; use a
  unique checked output directory and preserve checkpoints. Measure throughput
  with a bounded run and agree on a time budget before committing to the full
  run.
- A proxy policy may exploit kinematic assumptions; do not infer physical gait
  or robot safety from this run.

## Acceptance and evidence

Before training:

- verify the exact candidate-map identity and scale 3; route length is 10 m;
- validate start, goal, and sampled route points against the inflated known-free
  map;
- across multiple deterministic seeds, verify exactly two targets, one fixed
  object, all five pedestrian modes, mirrored sides, full-horizon map
  traversability, target separation, and reproducibility;
- run the focused SB3 environment test and a short reset/step smoke test;
- check the unique output target does not exist and available disk space is
  sufficient.

After training:

- preserve final policy, metadata, summary, periodic checkpoints, and logs in a
  new scale-3 dual-target directory;
- compare CRUISE baseline and PPO using the same 10 deterministic balanced
  evaluation episodes; report success, timeout, collision/contact, map
  violation, route progress, and minimum clearance;
- report exact steps, run duration, output paths, map hashes, and any unrun
  checks. Any observed collision, map violation, invalid episode, or failed
  artifact load is a failed/limited experiment, not hidden or filtered.

## Rollback and deployment

Rollback means revert only the changes introduced for this experiment in the
two scoped Python files and leave the plan, logs, evidence, pre-existing dirty
worktree contents, map, and all old checkpoints intact. The new checkpoint is
not loaded by any runtime and will not be synchronized to Jetson. No production
or hardware deployment is authorized.

## Interrupted training attempt and revised time budget

The fresh 300,000-step run was started in the unique output directory, then
stopped at the user's request after 6,400 steps. The last SB3 table reported
20 FPS and 311 seconds elapsed, giving an observed throughput of about 20.6
environment steps/s. Linear estimates from this short run are approximately:

| Training steps | Estimated PPO training time | Approx. time including 10 paired evaluation trials |
|---:|---:|---:|
| 25,000 | 20 min | 27-32 min |
| 50,000 | 41 min | 48-53 min |
| 100,000 | 1 h 21 min | 1 h 28-33 min |
| 300,000 | 4 h 03 min | 4 h 10-15 min |

Evaluation estimates extrapolate from the early mean episode length and may
vary. The original callback cadence was 25,000 steps, so this interruption
occurred before any resumable checkpoint. The only file left by the attempt is
the small TensorBoard event log under
`output/d1_max_proxy_map3_dual_target_balanced_300k_20260924/`; no policy,
metadata, summary, or checkpoint was produced. The log is retained and is not a
performance result. That interrupted 300k attempt remains preserved as a
separate historical run.

## 25k pilot result (2026-09-24)

The user selected a 25,000-step first-stage run with a 5,000-step checkpoint
and training-outcome window. The run completed at 25,088 steps because PPO
finishes full 256-step rollouts. Full configuration, trend tables, evaluation
rates, artifacts, and limitations are recorded in
`simulation/path_planning/evidence/20260924_d1_max_proxy_map3_dual_target_ppo_25k.md`.

The rolling reward mean improved from a low near `-40.4` early in training to
`-27.0` at the final step, but all five training-outcome windows had zero
successes. The 10 paired fixed-seed evaluations also had `0/10` successes for
both baseline and PPO. PPO reduced collision from `4/10` to `1/10`, while
timeouts rose from `1/10` to `4/10`; both had map violations in the same `5/10`
scenario seeds. This is a short simulation-only pilot, not a successful policy
acceptance result. Investigate map-violation episodes and the timeout/collision
trade-off before allocating another training budget.
