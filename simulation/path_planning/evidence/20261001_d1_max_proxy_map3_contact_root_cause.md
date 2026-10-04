# D1 Max proxy map-3 contact root-cause diagnosis

Date: 2026-10-01 (Asia/Taipei)  
Status: **read-only diagnosis complete; no fix or new training authorized**  
Classification: C3 controller/planner investigation, local simulation only.

## Scope and provenance

Used the project `.venv` on the local Mac with Gymnasium 1.3.0, MuJoCo 3.11.0,
SB3 2.9.0; D1 Max kinematic proxy, `full_randomization`, candidate map scale 3,
10 m routes, `people_balanced_multi_target`, 70 s duration, MPPI 32 samples x
30 steps (0.10 s each), seed 42, `goal_hysteresis`. No ROS, Jetson, robot,
source/map/checkpoint edits, PPO learning, resume, or changes to collision and
map-boundary checks.

Read the project instructions, engineering policy, handoff, docs index, the
11,520-step stop report and the preceding map-3 diagnostic review. The 10k
checkpoint was read-only and loaded directly with SB3, without the runner's
metadata guard. It has no metadata sidecar and is not accepted as compatible or
deployable. The old 25k policy was not loaded.

SHA-256 before and after the diagnostic was unchanged:

| Artifact | SHA-256 |
|---|---|
| `tools/d1_edu_sb3_env.py` | `97571573f60aed02152bb670f70968f2063cbbb27caf6b49c9b0423b15faf484` |
| `tools/d1_edu_nav2_global_path.py` | `cb90854493842887723eca9d924d36b9f91f44c98b1aeb00f46cc4d0f8ea735f` |
| `tools/run_d1_edu_decision_layer_sb3.py` | `1b9134afbcc616567b8ab6b543dbd76f6dd416936b4aa143304cea7daa9f546f` |
| candidate map `map.yaml` | `be42f16b9730106c54f0bb75e83b04ec48b7c2c69f49e12c4fb4183ca21feb5d` |
| candidate map `map.pgm` | `41e913c872bd118fc971b313024339f78e23247aec94fde00a18e51aa62efd74` |
| stopped-run 10k checkpoint | `4b1e1da8733fb51d0001dc1ca266c9c1c3354ef4ab02812d0b8d0620501d7a62` |
| prior 25k policy (not loaded) | `45fdfb1d7da066d90f5df3b52c341aed35d92b49cd775530687757ce1f0f5f08` |

## Bounded replay and direct contact evidence

Replayed action-0 on reset seeds 10042/curriculum 0, 10043/curriculum 1, and
10046/curriculum 4 (crossing, same-direction, irregular). Also replayed the
deterministic 10k checkpoint on the exact 10042 scene. Paired 10042 identity
matched: episode/map-scene seed `1369567760`, route SHA-256
`423c8d567a52e04fc51277845b39f6d88e27f31ae1f5dfb086a111e7ac7cd181`, identical
start/goal and actor parameters. The 10046 run was repeated with curriculum 4
after an exploratory curriculum-2 attempt was identified as the wrong family;
the curriculum-2 attempt is excluded from the evidence below. No fourth unique
seed was run.

The diagnostic wrapped the existing MuJoCo contact query in memory and copied
the first non-floor geom pair, contact point, robot pose, actor poses, current
controller state, and clearances. It returned the original contact result
unchanged. It did not relax or replace any safety predicate.

| Scene / policy | First physical contact | State at first contact | End state |
|---|---|---|---|
| 10042 crossing / action-0 | `BASE_LINK_collision` (`BASE_LINK`) ↔ `moving_obstacle_geom` (`moving_obstacle`, fixed object); penetration `-0.00094 m`; point `(-3.14896, 2.06846, 0.60)` | `t=34.880 s`; robot `(-3.03575,1.60651)`; object center distance `0.720 m`; progress `0.779`; runtime map free; nearest map-wall clearance `0.794 m`; pedestrian inactive and `4.933 m` away | collision; no timeout/map violation |
| 10042 crossing / checkpoint 10k | Paired rollout physically contacted the same fixed-object actor in this scene; the terminal geoms were not retained in the compact checkpoint output, so no geom-pair claim is made for this policy here | `t=46.824 s`; object center distance `0.725 m`; progress `0.779`; AVOID active, `avoid_side=+1`, requested action SLOWDOWN; selected track was `1.501 m` from object and `5.359 m` from inactive person | collision; no timeout/map violation |
| 10043 same-direction / action-0 | `moving_obstacle_geom` (`moving_obstacle`, fixed object) ↔ `FAR_FOOT_LINK_collision` (`FAR_FOOT_LINK`); penetration `-0.00093 m`; point `(-2.87519,3.35963,0.00344)` | `t=28.288 s`; robot `(-3.09607,2.93874)`; object center distance `0.810 m`; progress `0.437`; runtime map free; nearest map-wall clearance `1.161 m`; pedestrian inactive and `1.881 m` away | collision; no timeout/map violation |
| 10046 irregular / action-0 | `BASE_LINK_collision` (`BASE_LINK`) ↔ `moving_obstacle_geom` (`moving_obstacle`, fixed object); penetration `-0.00047 m`; point `(0.67381,9.26493,0.60)` | `t=55.552 s`; robot `(0.75589,8.79622)`; object center distance `0.734 m`; progress `0.778`; runtime map free; nearest map-wall clearance `0.599 m`; pedestrian inactive and `4.947 m` away; risk true, TTC `2.0 s`, AVOID active with `avoid_side=+1` | collision; no timeout/map violation |

These are direct simulation observations. Across all three baseline scenes,
physical contact is with the fixed-object actor, not a map-wall geom or the
pedestrian; no map-boundary violation occurred. The 10042 baseline's exact
rectangle clearance to the fixed object fell from `+0.0319 m` at `t=34.656 s`
to `+0.0060 m` at `34.752 s` and `-0.0199 m` at `34.848 s`; it was still
executing requested action-0 in CRUISE at the last sample. The selected track
then was about `1.475 m` from the fixed-object center and `5.363 m` from the
person; latest risk was true but TTC was `3.0 s`, and the baseline decision
mode was CRUISE. This directly shows the decision/planner did not keep an
effective local avoidance command through that final approach. It does not
prove why the tracker selected that particular LiDAR cluster.

At 10043's last controller sample before contact, the selected track was about
`2.203 m` from the fixed object and `3.932 m` from the person; risk was false
and execution was CRUISE. At 10046 contact, AVOID was active and MPPI requested
local velocity approximately `[0.079, +0.212, 0] m/s`; the side was still
`+1`. Thus the baseline has a CRUISE/decision-layer failure mode, while an
AVOID-active sample also collided with the fixed object. The 10k replay on
10042 likewise ended in contact despite AVOID at its terminal approach. These
facts argue against “PPO action selection alone” as the root cause.

## Bypass-side hypothesis and detection timing

The actor generator's fixed-object check accepts a placement if it finds at
least one bypass side using the runtime map planner. The runtime robot radius
is `0.49 m`; the object's circumscribed radius is `0.354 m`; adding the runtime
`0.12 m` margin gives `0.964 m` required bypass-center offset. The scale-3
route-selection planner adds a separate `0.25 m` center buffer, so its
corresponding offset is `1.214 m`. Side checks sampled 26 points across 2.5 m
of route around the object. `+1` means the route-left normal; for the near-north
route segments here, that is also approximately local `+y`.

| Seed | `-1` runtime-free at 0.964 m | `+1` runtime-free at 0.964 m | `-1` route-mask-free at 1.214 m | `+1` route-mask-free at 1.214 m |
|---:|---:|---:|---:|---:|
| 10042 | 26/26 | 12/26 | 26/26 | 7/26 |
| 10043 | 26/26 | 11/26 | 16/26 | 5/26 |
| 10046 | 26/26 | 2/26 | 24/26 | 0/26 |

The map-aware-generation premise “there is a runtime-clear bypass” holds for
all three at the runtime safety margin, but the available side is consistently
`-1`. The MPPI implementation initializes `avoid_side=+1` when first given
points and has no map-feasibility input for that sign. In 10042 and 10046, the
`+1` side is plainly infeasible even under the unchanged runtime mask. This
supports the review hypothesis as a likely contributor, particularly for the
10046 contact while AVOID/+1 was active. It is not by itself proof that every
contact would be prevented by choosing `-1`: 10042/10043 also show CRUISE
approaches, and 10043's stricter route mask has gaps even on `-1`.

At 10042 reset, the LiDAR yielded 50 points and multiple tracker candidates;
first risk/AVOID was at `0.096 s` while the robot was `7.967 m` from the fixed
object. The selected track was `7.470 m` from that object and `2.070 m` from
the pedestrian, so this initial AVOID was not evidence of fixed-object
detection. At the final approach the selected track remained roughly
`1.48–1.52 m` from the fixed-object center. The harness did not isolate the
first raw LiDAR return on the fixed object's footprint; an object-specific
first-detection distance is therefore **not established**. Do not equate the
initial tracker/AVOID event with detection of the fixed object.

MPPI's horizon is `3.0 s`, with nominal `vy` bounded to `±0.34 m/s`; a
straight-line upper bound is `1.02 m` lateral displacement before SDK
acceleration/delay effects. That is only `0.056 m` above the `0.964 m` runtime
offset and `0.194 m` below the `1.214 m` route-selection offset. This is a
geometric plausibility check, not a measured reachable-set bound. The data
supports side mismatch and decision disengagement more directly than a claim
that horizon length alone caused the contacts.

### Reproduction

The episode/policy loop is the exact inline harness in the adjacent
`20261001_d1_max_proxy_map3_dual_target_balanced_70s_25k.md` (its `identity`
and `rollout` functions and `D1DecisionEnv` constructor); use the same project
interpreter and constructor arguments below, but restrict the reset matrix to
the three rows below. For this contact diagnosis, the in-memory instrumentation
was added immediately after each reset and removed in `finally`:

1. Save `d1_edu_sb3_env.mujoco_obstacle_contacts`, then replace it with a
   wrapper that calls the saved function exactly once and returns its result
   unchanged. On the first `True`, inspect `env.data.contact[0:env.data.ncon]`
   and retain every contact where one geom ID is in `env.obstacle_geom_ids`
   and the other is not a floor or another obstacle. Record both geom/body
   names (`env.model.geom(id).name`, `env.model.body(id).name`), `contact.dist`,
   and `contact.pos`. The dynamic actor index is its position in
   `env.obstacle_geom_ids[:len(env.obstacle_specs)]`; the remainder are map-wall
   IDs.
2. Wrap `env._command_for_action` once, calling the saved method unchanged.
   Before and after it, record `env.elapsed`, requested action, returned
   command, `env.active_mode`, `env.latest_baseline_mode`, risk/TTC, planner
   `avoid_side`, latest selected track, route progress/lateral/heading error,
   planner mean/costs, and the SDK command/applied/actual velocities. Keep
   samples while the fixed-object center is within 3 m and at the terminal
   approach. Actor positions come from
   `actor.state(float(env.elapsed))`; an inactive actor must not be included as
   an active cause. Read map-wall clearance from the unchanged wall obstacle
   rectangles. Stop each episode at its original termination.
3. For the side test, use `env.global_path` at the fixed-object center to get
   projected route station and local normal. Sample 26 stations, 0.10 m apart
   over ±1.25 m; at each station query `env.map_planner` at signed normal
   offsets ±0.964 m and `env.map_route_planner` at ±1.214 m. These are separate
   read-only queries; do not mutate either planner or its safety radius.
4. Restore the original contact and command methods and call `env.close()` in
   `finally`. Run action-0 for 10042/curriculum 0, 10043/curriculum 1, and
   10046/curriculum 4. For checkpoint comparison, reset the same 10042 scene
   and use `PPO.load(<10k checkpoint>, env=env, device="cpu")` with
   `deterministic=True`; assert the scene identity tuple matches before
   comparing outcomes. Loading is offline diagnostics only, not a compatibility
   or deployment check.

Exact reset/policy matrix (each scene reset separately before baseline/model):

```text
10042 curriculum 0 crossing: action 0; then deterministic PPO.load(10k)
10043 curriculum 1 same_direction: action 0
10046 curriculum 4 irregular: action 0
```

Environment arguments:

```text
profile=full_randomization, robot_profile=d1_max_proxy,
kinematic_proxy=True, scenario_mode=people_balanced_multi_target,
seed=42, duration=70, route_length=10, mppi_samples=32,
mppi_horizon=30, collision_penalty=20, timeout_penalty=4,
cruise_mode=goal_hysteresis, candidate_map_yaml=<map above>, map_scene_scale=3
```

For each reset, compare the full identity tuple `(episode_seed,
map_scene_seed, curriculum/family, route SHA-256, start/goal, actor records)`;
stop at mismatch. For bypass checks, project the fixed object's center onto
the active route, form its local route normal, and sample both signed offsets
at `0.964 m` and `1.214 m` every `0.10 m` along route `[-1.25,+1.25] m`;
query both the unchanged runtime planner mask and stricter route-selection
mask. The command prefix is:

```bash
PYTHONPATH=simulation/path_planning/tools .venv/bin/python -u - <<'PY'
# Construct D1DecisionEnv with the exact arguments above; wrap contact and
# command methods once; run only the reset/policy matrix above; print compact
# JSON per scene; restore methods and env.close() in finally.
PY
```

The five-seed report supplies the exact standalone PPO load context, scene
identity function, and per-step rollout loop. This section specifies the
additional contact/body and bypass instrumentation. The diagnostic wrapper
was transient and was not saved as a separate executable; no source file was
edited to install it.

## Root-cause assessment (facts vs. inference)

**Hard evidence:** fixed-object MuJoCo geom contact in three distinct
action-0 scenes, at two different robot collision bodies; 10042 action-0 and
10043 action-0 approach in CRUISE at/through negative rectangle clearance;
10046 action-0 contacts while AVOID/+1; sampled runtime bypass availability is
consistently `-1`, while `+1` is blocked; walls remain separated at contact;
pedestrian is inactive at all three contacts.

**Most likely combined cause:** the inserted fixed target lies on the
precomputed 10 m centerline, but route generation only requires one viable
runtime side and does not communicate that side to the local MPPI. MPPI's
deterministic `+1` commitment can point into unknown/occupied cells. Separately,
the CRUISE safety decision relies on risk from an unlabelled LiDAR track; when
the fixed actor is not the selected/predicted blocker (or risk falls below the
immediate threshold), it returns to CRUISE and continues toward the fixed
object. These mechanisms explain the observed AVOID-side and CRUISE contacts
better than PPO action choice alone.

**Not established:** whether all actor/object contacts would be eliminated by
side-aware MPPI; exact first LiDAR detection time of the fixed object; whether
3 s horizon is independently insufficient; whether tracker association,
terminal-path risk, or MPPI local minima is the dominant component across the
full distribution. No statistical rate or generalization claim is made.

## Two minimal C3 design options for review (no implementation selected)

| Option | Minimal design | Benefits | Trade-offs / failure modes |
|---|---|---|---|
| A — map-aware MPPI side commitment | Pass feasible passing-side information from the map-3 scene to MPPI; select only a side whose sampled center corridor is runtime-mask-free, and fail the scene gate if neither side exists. Keep current tracker, map-boundary test, physical contact test and penalties unchanged. | Directly addresses measured `+1` vs `-1` mismatch; smallest behavior delta; preserves actor/route identities for controlled A/B. | Side corridor sampling can miss geometry; a valid side does not guarantee local MPPI convergence; CRUISE may still fail to enter/maintain AVOID. Three-second horizon remains unchanged. |
| B — route around fixed geometry before rollout | Include the fixed object's footprint as static geometry in the map-3 route/local planning layer, so the 10 m path itself uses a valid bypass; keep pedestrian dynamic tracking separate. Reject reset if a collision-free route of required length cannot be formed. | Removes the demonstrated dependence on a transient tracker/risk trigger for the static target; route and local controller share the obstacle topology. | Changes route shape and potentially actor/start/goal placements, so prior trial-index comparisons become different distributions; more planner integration and scene-generation validation; local MPPI can still mishandle the pedestrian. |

Both are C3 algorithm experiments and require explicit parent/user selection
before implementation. Neither option changes the runtime safety mask, contact
predicate, map source, timeout, goal tolerance, or protected checkpoints.

**Acceptance gate before any fresh PPO:** first run fixed-seed path-only and
fixed-object baselines with geom-level contact capture; require no contact,
no map violation, a recorded feasible bypass, and preserved start/goal/route
identity for controller-only comparisons. Then run the fixed object on both
mirrored sides and pedestrian families with unchanged 70 s and safety checks;
inspect all failures. Only after the parent reviews those gates should anyone
propose a new short PPO identity/output. Rollback is to revert only the later
approved C3 code delta; preserve all current reports, maps, policies and
checkpoints. This diagnosis itself changed no source or runtime behavior.

## Verdict

The 11,520-step run remains stopped. The 10k checkpoint is diagnostic-only and
must not be resumed. Evidence points to a fixed-obstacle scene/planner interface
defect (wrong or unavailable MPPI side plus CRUISE not maintaining static
obstacle avoidance), not a pedestrian-caused contact. The next action is parent
review of Options A/B and the acceptance gate—not training.
