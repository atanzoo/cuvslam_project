# Nav2 MPPI + IMM + PPO interface design

Date: 2026-10-02 (Asia/Taipei)

Status: **user waived additional mirrored/varied fixed-placement testing on
2026-10-02; user selected Option A and approved the staged 15-file scope.
Luna max implements bounded stages; the parent agent reviews evidence before
each next stage. Execution resumed following the user's request to continue;
G1 is accepted; G2 is in progress and later gates remain pending.**

## Objective and scope

Connect the selected PPO high-level actions and IMM pedestrian predictions to
the isolated ROS 2 Humble Nav2 MPPI controller used by the D1 Max kinematic
proxy. This design covers only simulation and training interfaces. It does not
authorize deployment to a robot, a production Nav2 workspace, or a motor
command path.

The existing v3 MPPI gate qualifies the original 10.962 m static-only route
and a 10.174 m boundary-approach route in the same scene. With `vx_max=0.28`
and unchanged bridge hard limits, both succeeded without collision, map
violation, timeout, or overspeed commands. It uses Nav2 MPPI `1.1.12` on
Jetson AGX Orin and a local Mac MuJoCo proxy. The gate deliberately disables
the pedestrian; mirrored/varied fixed placements remain unqualified. See
[speed-headroom acceptance](NAV2_MPPI_BOUNDARY_GATE_COMMAND_HEADROOM_20261002.md).
The current SB3 environment instead uses a local `ObstacleMPPI`; its five
PPO actions do not yet reach Nav2. The speed-headroom approval did not select
or authorize a new ROS interface/critic package from this separate design.

## User-directed gate reduction (2026-10-02)

The user explicitly requested: “我認為鏡射不同定物配置不用測 直接加入imm ppo接口驗收就好”.
Accordingly, additional **fixed-object** left/right mirrors and varied
placements are removed as prerequisites for this bounded simulation pilot.
They remain untested, not passed. This does not waive human approval of the
new interface/plugin implementation, dynamic-prediction and five-action
interface tests, or zero-contact/map-violation requirements on each run.

The accepted v3 two-route baseline remains preserved. No map, footprint,
clearance, bridge speed/timeout threshold, TF owner, or command owner changes
with this gate reduction. Its scope ends at the first bounded 25k pilot; any
fixed-object or boundary violation stops the affected gate/pilot and requires
review. It cannot support claims of general obstacle-layout safety or robot
deployment. Owner/approver of the gate reduction: the human user in this chat.

## Required runtime contract

```text
simulated observations -> tracker / IMM -> timestamped human prediction tube ─┐
PPO policy -> CRUISE / AVOID_LEFT / AVOID_RIGHT / SLOWDOWN / WAIT_YIELD ────────┤
                                                                              v
Nav2 MPPI: path cost + static costmap cost + human prediction/preference cost
                                -> isolated /cmd_vel -> guarded MuJoCo proxy
```

- Nav2 planner remains the global-path owner; `controller_server` remains the
  sole ROS publisher of `/cmd_vel`.
- IMM input contains scene identity, frame, observation timestamp, time step,
  actor identity, confidence, body radius, and time-indexed fused positions
  with uncertainty radius. The existing predictor is IMM-style CV/CA/CTRV
  fusion: it exposes a mean trajectory, scalar uncertainty tube, and mode
  weights, not separate per-mode covariance trajectories. Version 1 reuses
  that actual output without changing the predictor algorithm or claiming a
  full probabilistic IMM. The critic evaluates candidates at matching times;
  mode weights are retained for diagnostics.
- PPO outputs only one of the five named preferences, with scene identity,
  timestamp, and expiry. The preference can shape passing side, speed
  preference, or waiting cost; it cannot command velocity or relax collision
  costs.
- Static occupied and Unknown map cells stay lethal. Retain the independent
  contact/geometric checks and the `0.61 m` static center-to-box-edge
  acceptance floor. The proxy checks collisions after simulation steps;
  these are not a pre-command swept-volume safety shield.
- Invalid scene identity, frame, timestamp, or expired input disarms the
  simulation bridge and yields zero velocity. The existing two accepted
  static routes are the retained baseline before person predictions are enabled;
  additional fixed-placement tests are waived as stated above.
- Training observations must come from the same simulated sensing/tracking
  contract used by the controller. Ground-truth actor state is reserved for
  scoring and must not leak into the PPO observation.

## Option A — dedicated MPPI critic plugin (recommended)

Add one isolated simulation ROS package, `nav2_imm_ppo_critic`, containing a
custom MPPI critic and one versioned `InteractionContext` message contract.
The combined input carries IMM prediction tubes and one PPO preference on
`/simulation/interaction_context`. The critic scores candidate trajectories
against time-aligned human predictions and the interaction preference.
Nav2's built-in critics continue to score global-path tracking and static
costmap costs. The combined contract keeps the first isolated experiment to
one new ROS package and one public input contract, rather than adding separate
message and critic packages before their independent reuse is established.

**Advantages:** preserves the time ordering of crossing predictions; represents
IMM uncertainty directly; gives PPO a bounded, inspectable influence on the
MPPI objective; does not require a Nav2 fork. Nav2 Humble exposes a critic
plugin interface and loads configured critics through pluginlib.

**Costs and failure modes:** adds one simulation-only ROS package with message
generation and a critic that must build for the pinned ARM64 Humble image;
a stale or mismatched
prediction can create an unsafe or overly conservative score; excessive critic
work can miss the 10 Hz controller deadline. All inputs therefore need strict
expiry, frame, scene, and sample-count checks, and the critic must be profiled
on Jetson before training.

**Rollback:** remove the custom critic from the simulation controller
configuration and restart the preserved stock MPPI profile. Keep the plugin,
messages, maps, logs, and failed trials as evidence; no production workspace is
changed.

## Option B — stock MPPI with a path and costmap adapter

Keep the released critic set. Translate IMM predictions into a temporary
costmap occupancy tube and translate PPO preferences into bounded path,
speed-limit, or critic-weight updates.

**Advantages:** avoids a custom MPPI critic and retains the installed Nav2
controller binary; may reach a first end-to-end smoke test sooner.

**Costs and failure modes:** collapsing time-indexed predictions into a spatial
union loses whether the robot or person reaches a location first, which can
block safe crossings and increase timeout. Path or parameter updates can also
arrive late or conflict with the active controller cycle. It is less suitable
for learning passing-side and wait decisions from predicted interactions.

**Rollback:** disable the simulation-only costmap/path adapter and return to the
preserved stock MPPI configuration. No production map or controller profile is
changed.

## Comparison and proposed selection

| | Option A: critic plugin | Option B: stock critics + adapter |
|---|---|---|
| Uses prediction time directly | Yes | No; predictions become spatial costs |
| PPO passing-side preference | Explicit critic term | Indirect path/weight updates |
| New ROS package/API | Yes | Adapter and runtime topic/parameter plumbing |
| Main risk | build/runtime deadline and stale input | lost timing, over-blocking, delayed updates |
| Fit to selected IMM + PPO design | Strong | Partial |

**Selected: Option A.** The selected architecture depends on time-aware
IMM predictions and bounded PPO preferences inside one MPPI optimization. The
released Humble plugin API supports a separate critic implementation, so a
Nav2 fork is unnecessary.

The human explicitly selected “Ａ：設階段使luna max完成 並階段性檢查 直到a完成”
after the two-option proposal and linked 15-file scope. This approves Option A,
the one-package/one-contract scope exception, and delegated implementation
with parent review between stages. Option B is rejected for loss of prediction
timing and indirect preference control. Model delegation is `gpt-6-luna` with
reasoning effort `max`; actual dispatch and stage results must be recorded.

## Change class, package ownership, and staged file scope

This is a **C4 simulation-only change**, including a C2 message/input contract,
C3 trajectory-cost plugin, and fail-closed handling of expired prediction or
preference input. The higher class applies even though no motor path is used.
Approval is now recorded above. The earlier waiver alone did not select the
implementation; the subsequent explicit Option A selection does.

Package owner: the independent navigation-simulation workstream, not cuVSLAM,
the production navigation stack, or the robot SDK. A new package is justified
because an ARM64 ROS plugin and generated input message have a build/runtime
lifecycle absent from the existing Mac Python tools. The existing D1 model
description package must not host controller logic.

Target: the preserved Humble MPPI 1.1.12 ARM64 image, isolated domain 178,
and Mac project `.venv` for sensing, PPO, and MuJoCo. Use existing ROS/Nav2,
pluginlib, rclcpp, rosidl, and test dependencies only after read-only availability,
ABI/compiler, license, and disk/RAM preflight. Do not install missing packages,
pull images, or alter a production workspace to make a build succeed.

Approved source/config file list for **Option A** (implementation staged below):

| Stage | Path under `simulation/path_planning/` | Responsibility |
|---|---|---|
| 1 | `ros/nav2_imm_ppo_critic/package.xml` | Pinned-target package dependency and license metadata |
| 1 | `ros/nav2_imm_ppo_critic/CMakeLists.txt` | Generate message, build/export plugin and tests |
| 1 | `ros/nav2_imm_ppo_critic/msg/InteractionContext.msg` | One versioned scene/frame/time/prediction/preference contract |
| 1 | `ros/nav2_imm_ppo_critic/include/nav2_imm_ppo_critic/interaction_critic.hpp` | Critic lifecycle, input snapshot, and scoring interface |
| 1 | `ros/nav2_imm_ppo_critic/src/interaction_critic.cpp` | Time-aligned human/preference scoring with bounded work |
| 1 | `ros/nav2_imm_ppo_critic/critic_plugins.xml` | pluginlib export without a Nav2 fork |
| 1 | `ros/nav2_imm_ppo_critic/test/test_interaction_critic.cpp` | Cost, invalid input, horizon, empty-context, and action tests |
| 2 | `nav2_map3/interaction_context_contract.py` | Pure input validation and tracked-prediction serialization |
| 2 | `nav2_map3/test_interaction_context_contract.py` | Negative cases and golden message fixtures |
| 2 | `nav2_map3/map3_sim_bridge.py` | Existing bridge gains the versioned context transport and expiry stop |
| 2 | `nav2_map3/map3_mppi_imm_ppo_v1.yaml` | Separate v3-derived profile adding the critic only |
| 3 | `nav2_map3/run_nav2_interaction_gate.py` | Bounded IMM-only and scripted five-preference actual-Nav2 gate |
| 3 | `nav2_map3/test_nav2_interaction_gate.py` | Harness/adapter ownership, terminal outcome, and no-ground-truth-leak checks |
| 3 | `tools/d1_nav2_decision_env.py` | Separate SB3 adapter; PPO selects preference, never local MPPI velocity |
| 3 | `tools/run_d1_nav2_decision_sb3.py` | Separate 25k training/evaluation entrypoint and model identity |

This is an explicitly approved exception to the default 3-file/no-new-package
AI scope under engineering policy §§10.2 and 14: the one input contract
requires the generated message, plugin export/build, publisher, consumer,
tests, and training adapter together. Approval covers this **15-file,
one-package, one-contract** staged list; no other source/config file is in
scope. Each stage is independently reviewed/tested before the next. If a
missing dependency or API requires another file/package/contract, stop and
request renewed design. Scope exception expires after the listed interface
acceptance and first 25k pilot, or earlier on a containment failure; responsible
approver is the human user in this chat on 2026-10-02. Runtime work remains
only in uniquely named isolated simulation directories on Jetson.

Existing source maps, map exports, v3 baseline YAML, old policies/evidence,
`d1_edu_sb3_env.py`, predictor algorithms, `/home/tseng/slam_ws`, motor and
sensor drivers remain protected. The new training entrypoint must refuse
incompatible old architecture metadata rather than resume the local-controller
checkpoint. There is no commit, push, real-robot deployment, or automatic
dependency upgrade in this scope.

The message contract records source observation time separately from Jetson
ROS receipt time and rejects unverified clock/frame alignment. Prediction
coverage must span the controller's 3 s horizon plus accepted input age;
do not silently pad missing future samples or equate the two clocks. A fresh
sensor update with zero tracks differs from missing/stale sensing. Perceived
tracks supply the message and PPO observation; actor truth is scoring-only.
Input validation must be transactional: rejected packets do not advance the
accepted sequence/time watermark. A new simulation epoch may reset source
elapsed time only while disarmed, after clearing the previous context; pose
sequences continue to increase. Startup stationary pose publication does not
constitute fresh sensor observations and must not extend an old prediction's
expiry. The ROS receipt clock and simulation elapsed clock are distinct.
`WAIT_YIELD` while not waitable or after its finite budget is a valid policy
request requiring a bounded fallback, not a malformed sensing packet.
Expiry, sequence, scene, action enum, finite values, array bounds, confidence,
and horizon coverage are tested before motion. The bridge must return zero
when required context becomes invalid or stale. PPO may not reduce static or
human collision costs. Finite critic penalties and post-step collision checks
are not a formal collision-free guarantee.

Rollback: stop the new simulation runtime/bridge, verify zero/no active goal
and no simulation port/controller remaining, then restart the preserved stock
v3 baseline only when needed. Retain package, build manifest, failed contexts,
commands, action results, maps, and trials. Disable the new critic/profile;
never overwrite the baseline or delete evidence.

## Verification sequence after approval

1. Freeze and preserve the stock MPPI profile as the no-PPO baseline.
2. Validate interface messages: matching scene/frame, monotonic timestamps,
   expiry, finite values, prediction horizon coverage, and each PPO action.
3. Load the critic in the isolated Humble controller; verify lifecycle and
   10 Hz timing with the bridge disarmed.
4. Do not add separate mirrored/varied fixed-placement trials. Preserve the
   two accepted stock-MPPI routes and test neutral/empty fresh input against
   baseline scoring; on every subsequent interface run still record contact,
   map violation, timeout, and the unchanged `0.61 m` static clearance floor.
5. Enable IMM without PPO for crossing, same-direction, accelerating, turning,
   and irregular pedestrians with left/right mirrors. Record tracking age,
   per-horizon prediction error, TTC, success, timeout, collision, map
   violation, minimum clearance, progress, and controller timing.
   The user directed keeping the existing appearance/disappearance events
   rather than retiming them to force encounters. Record which trials actually
   encounter a pedestrian; do not infer avoidance coverage from trials where
   the person disappears before the robot arrives. No added encounter-count
   prerequisite or scenario-generator change is authorized by this follow-up.
6. Verify the five PPO preferences through the critic, including blocked-side
   fallback and bounded `WAIT_YIELD` expiry.
7. Only after these gates pass, run the previously approved 25k-step pilot
   with a 70 s episode timeout. Report success, timeout, fixed-object and
   pedestrian collision, map violation, and clearance in each 5k window; stop
   on any fixed-object or boundary violation.

## Decision required

The architecture and Option A implementation are selected. No further blanket
approval is required inside the listed scope. A new dependency, extra package,
source/config file outside the list, changed map/footprint/speed/watchdog, or
real-robot runtime requires renewed design. Failures are preserved and repaired
within scope; a failed stage is not accepted merely because the process starts.

## Delegated execution checkpoints (2026-10-02)

The parent owns documentation, target preflight, integration commands, and
acceptance decisions. One `gpt-6-luna`/`max` worker owns the assigned disjoint
source/config files. It may not self-advance past a checkpoint or start any
Jetson runtime, training, package installation, Git operation, or motor/sensor
process. The parent supplies the next bounded assignment only after review.

| Checkpoint | Worker deliverable | Parent acceptance | Initial state |
|---|---|---|---|
| G1 | Pure input contract/serialization and negative-case tests | Re-run tests, inspect clock/frame/expiry/horizon and no-truth-leak handling | accepted: parent reran 33 tests, 3.8 syntax and actual tracker serialization |
| G2 | Typed message, critic package, plugin export, tests and separate profile | Pinned ARM64 build/test, neutral cost regression, disarmed lifecycle/load and bounded compute-time evidence | in progress: Luna max assigned the approved eight-file slice |
| G3 | Bridge context transport and bounded IMM-only runner | Single publisher/TF ownership, source/receipt time alignment, live/stale input behavior, dynamic route outcomes and clearances | pending |
| G4 | Five PPO preferences, separate SB3 adapter/entrypoint and tests | Blocked-side fallback, WAIT expiry, all five inputs actually consumed, model identity, short interface-only SB3 smoke | pending |

Dispatch record: `gpt-6-luna`, reasoning `max`, agent
`01a0fb65-5312-7b90-8cf0-bff0a04405e2` (nickname Russell) received only the
two-file G1 assignment. The parent's Jetson dependency preflight passed;
the network-none read-only probe container exited and is preserved. G1 source
and test work is retained without accepting it. The worker was closed after
the user's interruption; no plugin runtime or training had been started.
On the user's subsequent request to continue, that worker was resumed with
the same two-file G1 assignment and max reasoning setting. Jetson connectivity
and resource/idle-runtime checks were refreshed; later gates remain pending.
Evidence: `simulation/path_planning/evidence/nav2_imm_ppo_a_20261002/preflight/`.

Complete A only when G1–G4 pass with saved evidence and the simulation runtime
is safely stopped/preserved. The current orchestration does not automatically
launch the full 25k learning run: it targets the selected interface acceptance.
The previously planned pilot budget remains recorded for a subsequent run.
