# D1 Max proxy map-3 PPO 25k diagnostic review

Date: 2026-10-01 (Asia/Taipei)

Status: Option A selected and delegated to Luna. The local route/controller change passes the focused regression but fails the path-only completion gate; no new PPO training is authorized by this result.

Environment: local Mac CPU, MuJoCo D1 Max kinematic proxy, SB3 PPO. This is not gait, Jetson, ROS/Nav2, hardware, or safety acceptance.

## Objective and evidence boundary

Identify what to change before the next 10 m, scale-3, fixed-object-plus-pedestrian training run. The initial diagnostic phase did not modify the candidate map, 25k policy, checkpoints, simulation source, or Jetson runtime; the subsequent delegated simulation-only edit is reported below. Diagnostics replayed the stored ten-trial evaluation seeds in memory. The prior configuration and outcome table are in `20260924_d1_max_proxy_map3_dual_target_ppo_25k.md`; the original machine-readable data are in `output/d1_max_proxy_map3_dual_target_balanced_25k_20260924/summary.json`.

The source map identity was rechecked on 2026-10-01:

| File | SHA-256 |
|---|---|
| `map.yaml` | `be42f16b9730106c54f0bb75e83b04ec48b7c2c69f49e12c4fb4183ca21feb5d` |
| `map.pgm` | `41e913c872bd118fc971b313024339f78e23247aec94fde00a18e51aa62efd74` |

## Prior pilot result

The prior run ended at 25,088 PPO steps. All 57 completed training episodes had zero successes. In ten matched evaluation scenes, the CRUISE baseline and PPO both had zero successes and the same five map-violation scene seeds. PPO collision fell from 4/10 to 1/10, while timeout rose from 1/10 to 4/10. Those rates remain the baseline for any future comparison; the diagnostic replays below are not a replacement ten-trial evaluation.

## What the replays showed

### Five shared map-violation scenes, CRUISE baseline

The fixed evaluation scene was reproduced with `reset(seed=10042 + trial, options={"curriculum_index": trial})`, the original 50 s limit, 10 m route, scale 3 map, two actors, and action 0 (CRUISE). All five again ended in map violation.

| Trial | Elapsed (s) | Robot path (m) | Path lateral error at exit (m) | Distance to fixed object center at exit (m) |
|---:|---:|---:|---:|---:|
| 4 | 23.54 | 4.04 | +1.61 | 4.41 |
| 5 | 35.69 | 6.26 | +2.41 | 4.33 |
| 6 | 32.02 | 5.92 | +3.44 | 5.44 |
| 7 | 40.87 | 7.86 | -1.03 | 1.34 |
| 9 | 27.78 | 4.69 | +2.53 | 5.76 |

Four exits were more than 4 m from the fixed object's center. This weakens the hypothesis that the fixed object alone causes the shared violations; it does not prove whether the person, route geometry, or controller is the primary cause.

The original map planner inflates blocked/unknown space by `0.49 + 0.12 = 0.61 m`, then uses a shortest grid route. Several selected path starts were only 0.05–0.10 m inside the resulting traversable mask. This is **extra robot-center room beyond the 0.61 m inflation**, not distance to a physical wall. One PPO map-violation scene (trial 9) left that mask after 1.224 s and 0.255 m of travel, about 5.15 m from the fixed object. It requested `AVOID_LEFT` repeatedly, including steps without a reported risk. The observations show little allowance for lateral control error at some starts.

The current map CRUISE command uses the episode's final goal vector for forward speed and a cross-track correction for body-frame lateral speed; its yaw command is zero. The AVOID branch receives a path lookahead target. This difference could explain loss of route tracking around bends or encounters, but needs a controlled path-only test before calling it the cause.

### Four PPO timeout scenes

The saved 25k policy was replayed deterministically on the four prior timeout scenes. All four again timed out at 50 s; none approached the 0.30 m goal tolerance.

| Trial | Final goal distance at 50 s (m) | Robot path (m) | Executed WAIT time (s) | `AVOID_LEFT` requests / 521 decisions |
|---:|---:|---:|---:|---:|
| 0 | 2.655 | 8.975 | 2.69 | 326 |
| 1 | 3.948 | 7.306 | 4.90 | 359 |
| 3 | 3.589 | 7.833 | 4.51 | 301 |
| 8 | 4.231 | 7.407 | 4.51 | 397 |

The nominal CRUISE forward speed is 0.22 m/s. Ten metres at that speed alone is about 45.5 s; the 50 s limit leaves little time for acceleration, detours, pedestrian yielding, and terminal slowdown. This is a plausibility calculation, not a lower bound on every actual trajectory because lateral motion and path geometry vary. The repeated `AVOID_LEFT` choices and a few seconds of executed WAIT show that the learned policy spends substantial time away from nominal progress; these counts do not isolate the reward as the cause.

For a bounded counterfactual, the same four layouts, actors, seeds, and route polylines were verified identical under a 70 s episode limit. Evaluation with the unchanged 25k policy produced **four collisions before reaching the goal**, at 52.98, 63.70, 59.38, and 65.48 s. Their final goal distances were 1.81–2.47 m. Thus more time alone did not recover these four scenes; it changed their observed terminal failure. The policy observes remaining-time information, so this A/B also changes that input and is not a pure post-50 s continuation.

### Extra route margin feasibility check

An in-memory planner with obstacle margin `0.42 m` (total inflation `0.91 m`) retained 86,158 traversable cells and a roughly 35.33 m long connected route, versus 126,184 cells and 35.37 m with the original `0.12 m` margin. All ten evaluation seeds still generated a 10 m route with exactly one fixed object and one pedestrian. Each new route centerline was at least about 0.316 m inside the **original** traversable mask.

Only the first two CRUISE rollouts were completed before the user asked to stop and organize: trial 0 collided at 31.42 s, 1.75 m from goal; trial 1 violated the map mask at 25.78 s, 6.19 m from goal. Trial 2 was interrupted and is not a result. The wider route margin is feasible for scene generation but has **not** shown that the baseline can finish routes by itself.

## Next iteration — C3 design options and selection

### Option A — route and controller feasibility first (recommended)

1. Keep the user-selected scale 3, 10 m distance, two actors, and original map bytes. In the local simulation branch, make CRUISE track the path lookahead and heading while retaining terminal-goal slowdown and the existing safety shield. Use a test with no actor, then the fixed object, then the balanced pedestrian distribution. The path-only gate should reach the goal without map violation across fixed seeds before PPO training begins.
2. Choose map routes with a measured extra centerline buffer (initial candidate: 0.20–0.30 m beyond the existing 0.61 m inflation), and verify a bypass around the fixed object inside that same safe area. Keep challenging cases as a separate evaluation set rather than silently dropping them.
3. Once the baseline is feasible, run a bounded fresh PPO pilot in a unique output directory, with early 5k checkpoint/evaluation gates and a maximum budget chosen by the user. Record per-episode final/minimum goal distance, time, path progress, map margin, invalid WAIT, AVOID requests when risk is false, collision, timeout, and success by motion family. Preserve the original ten scenes as a regression set alongside the new feasible training distribution.

Advantages: addresses shared baseline/PPO failures before tuning reward; makes the next result interpretable. Costs: local controller and route-generation changes require focused C3 review and validation, and the first pilot will use a changed training distribution. Risks: an overly generous route filter could hide difficult geometry; preserve the original seeds as a hard-case set. Rollback: keep the 25k checkpoint and original simulation paths/parameters, and revert only the approved local simulation changes if the path-only gate fails. Deployment impact: none; no Jetson or robot use.

### Option B — tune only the current PPO objective and deadline

Keep the current route/controller, increase the timeout penalty or reduce unproductive WAIT/AVOID, and possibly extend the 50 s cap; then resume or retrain the PPO policy.

Advantages: smaller implementation scope and faster first experiment. Disadvantages: the shared five map violations remain in the unchanged baseline, and 70 s alone turned all four sampled PPO timeouts into collisions. Stronger timeout pressure may trade waiting for unsafe motion. Compatibility: no public ROS contract, but learned policy dynamics change. Rollback: restore the original reward/limit parameters and retain the 25k model. Verification: paired original-seed evaluation must report success, collision, timeout, and map violation; any increase in collision is a failed gate. Deployment impact: none.

Option A was selected by the user on 2026-10-01 with the instruction that Luna do the implementation and the main agent assign and analyze. Option B is rejected for this iteration because the unchanged baseline fails on the same five map scenes and the 70 s counterfactual converted four timeouts into collisions. Option A separates a potentially infeasible or poorly tracked route from policy learning. Do not infer a successful solution from fewer timeouts alone.

The first delegated unit is limited to local simulation route/controller feasibility. It may edit up to three scoped simulation source/test files, must preserve the existing map, policy and checkpoints, and must not start PPO training until the baseline gates are reviewed. Maintenance cost is limited by keeping the change in the existing simulation path-planning tools and exercising both map and non-map behavior. A future map or control interface change requires renewed review. The selection should be revisited if path-only completion cannot be achieved on fixed seeds without weakening the current collision or map-boundary checks.

## Review gate and current status

The next behavioral change is C3 under `ENGINEERING_GUIDELINES.md`; the two options and their trade-offs were recorded before implementation, and the user selected Option A. No new training, model resume, source edit, map edit, Jetson process, or robot motion occurred during the diagnostic review. The partially completed wider-route baseline was stopped at the user's request. The old policy and checkpoints remain the reproducible baseline.

Checks performed: original 25k summary inspected; five baseline failure seeds and four PPO timeout seeds replayed at the original settings; four matched layouts checked at 70 s and evaluated; ten wider-margin scene resets validated; map source hashes matched the previous evidence. The only wider-margin motion results are the two completed trials above. Hardware, ROS, gait, and deployment checks were not run because this is local proxy analysis.

## Delegated Option A phase-1 result (2026-10-01)

Luna changed only `d1_edu_sb3_env.py`, `d1_edu_nav2_global_path.py`, and `test_d1_edu_decision_layer_sb3.py` under `simulation/path_planning/tools/`. For the opt-in scale-3, 10 m candidate-map mode, route selection now uses an extra `0.25 m` centerline buffer and CRUISE receives bounded path-heading feedback. The original runtime map-violation mask remains `0.49 + 0.12 = 0.61 m` (13 cells); the route-selection mask uses 18 cells. The map YAML/PGM, old checkpoint, physical-contact check, and ROS/Jetson/robot runtime were not changed. This controller and scene-distribution revision is **not** compatible evidence for resuming the old 25k policy; the checkpoint metadata does not encode the revision.

The project interpreter (`.venv/bin/python`, with Gymnasium 1.3.0 and MuJoCo 3.11.0) ran `simulation/path_planning/tools/test_d1_edu_decision_layer_sb3.py`; the main agent independently reran the same command with exit code 0 and five PASS lines (SB3 reset/step, D1 Max proxy reset/step, scale-3 dual-target curriculum, per-5k trend aggregation, and map hash/scale checkpoint guard). Static inspection found 92,087 cells in the new route-selection mask versus 126,184 in the unchanged runtime mask and a 35.62 m whole-map route. These are structural checks, not route-completion evidence.

For the behavioral gate, Luna reset ten fixed seeds `10042`–`10051`, disabled both generated actors after reset, and supplied action 0 (CRUISE) on every step with a 50 s cap. All ten ended in timeout: **0/10 success, 0/10 collision, 10/10 timeout, 0/10 map violation**. Final goal-distance median was `2.039 m`, range `0.721–3.240 m`; every distance exceeded the `0.30 m` success radius. Although all requested actions were CRUISE, the environment's safety/planner mode could still enter AVOID near map walls. The first seed already failed the gate; the remaining nine rollouts were more than needed for early stopping, but are retained as observations. Since the path-only gate failed, fixed-object, pedestrian-mirror, and original-scene controller-only versus controller-plus-margin comparisons were **not run**. Consequently the independent effects of heading feedback and route buffer remain unmeasured.

| Reset seed | Result | Final goal distance (m) | Robot travel (m) |
|---:|---|---:|---:|
| 10042 | timeout | 0.721 | 9.99 |
| 10043 | timeout | 2.072 | 8.48 |
| 10044 | timeout | 1.290 | 9.31 |
| 10045 | timeout | 2.006 | 8.51 |
| 10046 | timeout | 3.240 | 8.10 |
| 10047 | timeout | 1.341 | 9.28 |
| 10048 | timeout | 1.647 | 8.84 |
| 10049 | timeout | 3.168 | 7.99 |
| 10050 | timeout | 2.143 | 8.87 |
| 10051 | timeout | 2.966 | 8.00 |

A read-only 50/70 s diagnostic on path-only reset seed `10042` verified identical episode/map scene seed (`1369567760`), start/goal, 10 m route, and route SHA-256 (`423c8d567a52e04fc51277845b39f6d88e27f31ae1f5dfb086a111e7ac7cd181`); both actors remained disabled. At 50 s, the robot had reached `92.82%` route progress but was `0.7208 m` from the goal and timed out. With a 70 s cap on the same scene, it reached the `0.30 m` success radius at `53.472 s`, without collision or map violation. The 50 s trace had path lateral error about `+0.006 m` at timeout, and the sampled commanded forward speed was `0.040 m/s`; sampled modes included AVOID despite action 0, so the result is not a pure unconstrained CRUISE-speed measurement. Whole-episode mode counts were not saved. This supports a tight-time-limit explanation **for this one seed only**; the other nine final distances and the previously observed 70 s PPO collisions prevent generalizing it to the full distribution. The diagnostic inline harness and exact stdout were not saved, so the table above is a reported result, not a fully archived replay artifact.

**Decision:** do not start fresh PPO or resume the 25k policy. Next, in the same local proxy scope, isolate 50 s route-progress loss and safety-mode intervention on a small fixed-seed set, then set a completion gate and explicit time-budget choice before additional training. Preserve the original ten dual-target seeds as a separate hard-case regression. No real-robot, gait, ROS/Nav2, or safety claim follows from these tests.
