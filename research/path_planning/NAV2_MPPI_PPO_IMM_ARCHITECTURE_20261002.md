# Map-3 Nav2 MPPI + IMM + PPO architecture decision

Date: 2026-10-02 (Asia/Taipei)

Status: **Architecture selected by user; isolated Nav2 MPPI v3 passed two
static routes in one scene and its runtime is stopped/preserved. The user
waived additional mirrored/varied fixed-placement testing on 2026-10-02.
Dynamic prediction and policy interface acceptance still block PPO training.**

Scope: D1 Max kinematic-proxy navigation research on the 3x Map-3 candidate
map. This record does not approve real-robot control, gait claims, or changes
to the production workspace.

## Objective

Build a navigation experiment in which Nav2 owns global planning and the local
controller action, IMM tracks and predicts pedestrians, and PPO selects a
bounded interaction preference. The controller must check the robot footprint
against static map costs and time-indexed pedestrian predictions before
emitting each velocity command.

The training target remains the previously approved 25k-step pilot, but it
starts only after the retained static baseline, dynamic-prediction, and policy
interface gates below pass. Additional fixed-placement trials were waived by
the user, not qualified. New checkpoints must carry a new architecture and
observation/action identity; old checkpoints are not assumed compatible.

## Selected architecture

```text
Candidate map + fixed objects
        -> Nav2 global costmap / planner
        -> global route -------------------------------┐
                                                       v
Simulated sensing -> target tracker -> IMM future occupancy tube -> MPPI controller
                                                       ^              |
PPO observation -> CRUISE / LEFT / RIGHT / SLOW / WAIT preference ----|
                                                                      v
                                                   footprint/collision gate
                                                                      |
                                                   Nav2 controller output
                                                                      |
                                                   guarded MuJoCo bridge
```

Responsibilities and invariants:

- The Nav2 global planner produces routes on known-free cells. Unknown cells,
  fixed obstacles, inflation, and map boundaries remain blocked. The existing
  `0.61 m` proxy clearance contract remains the starting acceptance threshold.
- A tracker supplies IMM mode probabilities and timestamped future pedestrian
  positions with uncertainty. The tracker and controller use one declared
  frame and clock. Predictions cover the entire configured MPPI horizon.
- PPO returns one of the existing high-level modes: `CRUISE`, `AVOID_LEFT`,
  `AVOID_RIGHT`, `SLOWDOWN`, or `WAIT_YIELD`. It never publishes velocity.
  A mode is a preference only; it cannot relax lethal map costs, footprint
  clearance, or the collision gate.
- The Nav2 `controller_server` hosts MPPI as the sole ROS publisher of the
  controller command topic. MPPI handles path tracking, speed, lateral motion,
  and yaw in one optimization loop. There is no parallel custom MPPI command
  publisher.
- The released MPPI cost critic scores the complete proxy footprint against
  static costmap cells, but its collision cost is finite and is not a formal
  hard-safety guarantee. An independent simulated swept-footprint, contact,
  and map-boundary gate must reject any unsafe command. The IMM prediction-tube
  critic is a later integration gate, not part of the current static profile.
- `WAIT_YIELD` is gated by a waitability test. A wait that makes no progress
  expires into a controlled replan or a safe stop; it cannot become an
  unbounded zero-velocity episode.
- The simulation bridge remains fail-closed: stale pose, stale controller
  command, invalid scene identity, or out-of-range command disarms motion and
  returns zero. The bridge is the only MuJoCo model-command input.

## Controller runtime options

| Option | Advantages | Costs and failure modes | Rollback |
|---|---|---|---|
| **A. Dedicated isolated ROS 2 Humble Nav2 runtime with the released MPPI plugin (recommended)** | Uses the maintained Nav2 controller plugin and its critic interface; keeps the tested Foxy/RPP A2 run as a direct reference; avoids maintaining a Foxy backport. | Requires a separate ARM64 Humble/Nav2 runtime, resource and package preflight, and revalidation of the existing pose/clock/command bridge. Cross-runtime time, QoS, TF, or package mismatch can invalidate results. | Stop the new isolated runtime and use the preserved Foxy/RPP A2 profile. Keep both evidence sets and runtime directories intact. |
| **B. Keep Foxy and write/port a custom MPPI controller plugin** | Keeps the ROS distribution and current A2 runtime unchanged; can match the current PPO/MPPI action interface closely. | Nav2 Foxy 0.4.7 does not release `nav2_mppi_controller`; the project would own a controller-plugin port, optimizer, critics, and long-term compatibility. Plugin defects or Foxy API differences could make the control evidence misleading. | Disable the custom plugin and return to the preserved Foxy/RPP configuration. |

Selected direction: **Option A**. Read-only preflight on 2026-10-02 confirmed
that the existing `isaac_ros_dev-aarch64:fixed` image has ROS 2 Humble, Nav2
MPPI `1.1.12`, and the required planner/controller/map-server packages. Jetson
had 7.8 GB free storage and no active containers. No image pull or package
installation is needed. The image and existing workspace remain unchanged;
only a uniquely named simulation work area may be added.

## Staged implementation and acceptance gates

1. **Runtime preflight (passed):** the existing ARM64 image has Humble, Nav2
   MPPI `1.1.12`, and the required planner/controller/map-server packages;
   plugin identity is `nav2_mppi_controller::MPPIController`. Jetson had
   7.8 GB free and no active containers at preflight. The separate runtime
   was then qualified in ROS domain `178`: all lifecycle nodes active, one
   `/cmd_vel` publisher (`controller_server`), one bridge subscriber, one
   `/odom` publisher, and verified TF/action readiness. The bounded bridge
   was disarmed after each trial.
2. **Static map and controller baseline:** retain Foxy/RPP as the known A2
   reference. Under the isolated MPPI runtime, verify map classes, costmap
   classes, TF ownership, action results, controller-command ownership, and
   bridge watchdog. The original 10.962 m route and a 10.174 m boundary route
   now pass with v3. Per user instruction on 2026-10-02, do not run an additional
   mirrored/varied fixed-placement suite before interface integration; keep
   those configurations recorded as untested. Each accepted run still must
   reach its goal within 70 s with
   zero fixed-object contact, zero map violation, and minimum clearance at
   least `0.61 m`.
3. **Dynamic prediction without PPO:** connect simulated observations to the
   existing tracker/IMM. Evaluate crossing, same-direction, accelerating,
   turning, and irregular pedestrian families, with left/right mirrors. Record
   tracking age, mode probabilities, prediction error by horizon, TTC,
   minimum clearance, contact, timeout, and goal completion. Fixed-object and
   boundary violations remain zero.
4. **PPO interface gate:** pass each of the five discrete modes through the
   Nav2 MPPI preference input with expiry, scene identity, and a safe fallback.
   Verify that PPO cannot command velocity or override a blocked side, that
   `WAIT_YIELD` exits or safely stops, and that controller-server remains the
   sole ROS command publisher.
5. **Bounded training and comparison:** only after gates 1–4 pass, run the
   approved 25k-step pilot with a new checkpoint identity. Compare no-PPO
   baseline and PPO on identical scenario seeds. Report 5k-window success,
   fixed-object collision, pedestrian collision, timeout, map violation,
   minimum clearance, and goal progress. Stop training if any fixed-object or
   map-boundary violation appears.

## Failure modes and safe behavior

- Invalid or stale pose, prediction, preference, TF, or command: expire the
  preference, disarm the simulation bridge, and command zero; do not hold the
  last command.
- No collision-free MPPI sample: request bounded Nav2 recovery/replanning; if
  no safe route appears, stop and mark the trial failed instead of weakening
  map or footprint constraints.
- Turning-person prediction misses or underestimates uncertainty: retain the
  failed trace, improve the motion model/uncertainty calibration, and repeat
  the predictor gate before training.
- Persistent WAIT or controller oscillation: classify as deadlock/timeout,
  preserve the trial, and tune the wait gate or critic balance before training.
- ROS distribution, clock, QoS, or bridge mismatch: stop the isolated run and
  return to the last accepted A2 profile. Never route commands to robot
  drivers.

## Protected assets and rollback boundary

The source candidate YAML/PGM, current fixed-map exports, existing checkpoints
and stopped-run artifacts, production Jetson workspace, robot drivers, motors,
and real `/cmd_vel` path are protected. The simulation rollback is to stop only
the new isolated runtime and bridge, verify no simulation controller remains,
and use the preserved Foxy/RPP baseline. Retain all logs, checkpoints, and
failed trials.

## Current evidence and known gaps

- Foxy/RPP passed forward and reverse in one fixed-obstacle scene on 2026-10-02;
  mirror placements and deliberate boundary-approach cases remain open.
- Isolated Humble Nav2 MPPI `1.1.12` is qualified for one closed-loop,
  fixed-object-only Map-3 trial in ROS domain `178`. The first `0.22 m/s`
  profile timed out after 70 s with 2.232 m remaining, without contact,
  clearance-gate failure, or map violation. A separate simulation-only profile
  capped at `0.30 m/s` forward and `0.70 rad/s` yaw completed in 39.328 s with
  0.931 m minimum fixed-object edge distance (the unchanged acceptance floor is
  0.61 m), no contact, no map violation, and Nav2 FollowPath success. This is
  one seed/route only; it does not qualify other obstacle placements, boundary
  approaches, dynamic pedestrians, IMM, PPO, gait, or hardware. Full evidence:
  [`20261002_nav2_mppi_map3_fixed_gate_v1_v2.md`](../../simulation/path_planning/evidence/20261002_nav2_mppi_map3_fixed_gate_v1_v2.md).
- One pre-motion attempt was rejected because the already-scaled Nav2 export
  was mistakenly supplied as the MuJoCo source map, which would have scaled it
  a second time. The scene-identity/geometry guard stopped before movement;
  the preserved Nav2 action result is aborted. The corrected retry used the
  original source map for MuJoCo and the 3x export for Nav2.
- The v2 boundary trial later disarmed on a real command above the bridge's
  unchanged 0.30 m/s forward hard limit. After the user selected speed
  headroom (v3 `vx_max=0.28`), the 10.174 m boundary route succeeded in
  37.568 simulated seconds, and the original 10.962 m route succeeded in
  40.808 seconds. Both had zero contact, map violation, or timeout; minimum
  fixed-edge distances were 1.178471 m and 0.932213 m. Paired ROS command
  records contain 378/411 finite, within-bound commands. This expands the
  static evidence to two routes in the same scene only; it does not complete
  mirrored/varied placement, pedestrian, IMM, or PPO-interface qualification.
  See [`speed-headroom design and acceptance`](NAV2_MPPI_BOUNDARY_GATE_COMMAND_HEADROOM_20261002.md).
- The earlier path-fed custom MPPI timed out; a simple heading addition
  collided with the fixed object. This supports testing a unified Nav2
  controller loop and retaining hard footprint checks.
- In the existing custom MPPI source, angular velocity noise and bounds are
  fixed to zero. Its behavior is not a suitable final curved-route controller
  without redesign.
- Historical PPO runs show that additional training alone did not resolve
  safety failures. The 25k scale-3 pilot had zero successes, and the older
  balanced 500k evaluation had turning-person collisions. Reuse these as
  failure cases, not compatible checkpoints.
- No image or package has been installed or pulled. The Humble MPPI profile,
  parameters, and evidence reside in a uniquely named simulation directory;
  its container is stopped and preserved after the gate. No PPO training has
  started.

## Decision owner and review trigger

The user selected the A2 + Nav2 MPPI + IMM + PPO architecture on 2026-10-02.
The user subsequently waived additional mirrored/varied fixed-placement
testing for the first bounded simulation pilot and directed progression to
IMM/PPO interface acceptance. This reduces coverage, not collision/clearance,
Unknown, command-limit, or input-expiry requirements. The custom interface
implementation and staged scope exception require their separate selection:
[`IMM/PPO interface implementation design`](NAV2_MPPI_PPO_IMM_INTERFACE_DESIGN_20261002.md).
Review this decision if the isolated Humble runtime cannot be qualified, the
controller cannot provide the D1 proxy's measured motion model, or any fixed
obstacle/map-boundary violation recurs. A training success or simulation pass
does not authorize real-robot deployment.
