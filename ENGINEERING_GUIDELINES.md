# cuVSLAM Engineering Development Guidelines

Version: 1.0  
Status: Mandatory project policy  
Scope: ROS 2, Jetson Orin, C++, CUDA, LiDAR, camera, IMU, SLAM, localization, Nav2, simulation, and real-robot deployments  
Primary project path: `/Users/tsengpochien/Desktop/cuvslam_project`  
Effective date: 2026-07-24

## 0. Purpose and Authority

This document defines the engineering process for a long-lived robotics software
project. It is not a C++ formatting guide. It governs architecture, change
design, testing, deployment, Git usage, and AI-human collaboration.

These rules exist because a robotics regression can be physical, silent, and
distributed across sensors, time, middleware, transforms, configuration, and
vehicle behavior. A change that compiles is therefore not necessarily a safe
change.

### 0.1 Rule language

- **MUST** means mandatory. A change may not be merged or deployed without it.
- **MUST NOT** means prohibited unless an explicit exception is approved and
  recorded.
- **SHOULD** means the default practice. Deviations require a written reason.
- **MAY** means optional.

### 0.2 Current project boundaries

This independent project contains or interfaces with:

- Jetson Orin visual processing and CUDA runtime.
- Intel RealSense D435i stereo/RGB-D camera and IMU.
- cuVSLAM visual mapping, tracking, and localization experiments.
- Independent datasets, calibration records, evaluation reports, and deployment records.
- A later, explicitly reviewed interface to simulation or real-robot Nav2.

The existing LiDAR/Nav2 system under `/Applications/slam_v2` is an external
comparison baseline. It is not modified by this project. The two systems remain
separate until a later, evidence-based comparison approves any integration.

### 0.3 Precedence

When documents disagree, apply this order:

1. Explicit user safety or hardware instruction.
2. This document.
3. `PROJECT_HANDOFF.md` and the current data contracts.
4. Package-level design documents.
5. Historical handoffs, research logs, and patch logs.

The project handoff records current state. This document defines how that state
may be changed. Historical evidence must not silently override current policy.

## 1. Engineering Principles

### 1.1 Evidence over implication

Every claim about localization, navigation, safety, or performance MUST name:

- the exact environment: simulation or real robot;
- hardware, ROS 2 distribution, middleware, and relevant parameters;
- the test command or procedure;
- the success metric;
- the log, bag, screenshot, or report that proves the result.

Reason: topic existence, lifecycle `ACTIVE`, or process success is not proof
that the data path or robot behavior is correct.

### 1.2 Separate contracts from implementations

Stable interfaces such as topic names, message types, frame ownership, QoS,
timestamps, and safety behavior MUST be documented independently from the node
that currently implements them.

Reason: Gazebo odometry, LiDAR ICP, cuVSLAM, and a future fusion node must be
replaceable without rewriting Nav2 or the operator interface.

### 1.3 One owner per side effect

Exactly one component MUST own each of the following at a given runtime:

- `map -> odom`;
- `odom -> base_link`;
- final `/cmd_vel` output;
- the authoritative static map;
- each robot's model command input;
- each lifecycle group.

Reason: duplicate TF or velocity publishers create nondeterministic behavior
that is difficult to diagnose and unsafe on a real robot.

### 1.4 Safety is a system property

Velocity limiting, command timeout, localization health, sensor health,
planner failure, and emergency stop behavior MUST be designed together.

Reason: a correct planner can still produce unsafe motion if a downstream node
continues old commands after the planner or sensor has failed.

## 2. Project Architecture

### 2.1 Required architectural layers

The project MUST keep these conceptual layers distinct:

1. **Hardware and drivers**: camera, IMU, LiDAR, motor, and Gazebo bridge.
2. **Sensor normalization**: timestamps, frame IDs, QoS adaptation, and sensor
   health.
3. **State estimation**: LiDAR ICP, cuVSLAM, wheel odometry, or fusion.
4. **Mapping and localization**: 2D occupancy mapping, visual mapping, AMCL,
   or future visual relocalization.
5. **Navigation**: Nav2 planner, controller, costmaps, recovery, and goals.
6. **Safety**: command limiting, timeout, stop zones, emergency stop, and
   fault transitions.
7. **Operations and observability**: launch, GUI, Foxglove, diagnostics,
   recording, and test reports.

Nodes MUST NOT silently absorb responsibilities from adjacent layers. For
example, a camera driver MUST NOT publish a navigation TF correction, and an
odometry node MUST NOT modify the static occupancy map.

Reason: layered boundaries let one hypothesis be tested without changing all
other variables.

### 2.2 Module boundary rules

A module boundary MUST be created when at least one condition is true:

- it has an independent lifecycle or failure mode;
- it has a stable ROS interface used by more than one consumer;
- it may be replaced by another implementation;
- it needs a separate test fixture or hardware dependency;
- it has a different deployment target, such as Mac, Jetson simulation, or
  real robot;
- it has materially different performance or CUDA requirements.

Modules MUST communicate through documented messages, services, actions, or
explicit library APIs. They MUST NOT reach into another module's private files,
parameters, PID files, or internal state.

Reason: shared implementation details create hidden coupling and make future
AI-generated changes likely to cross ownership boundaries.

### 2.3 When to add a package

Adding a ROS package requires a design note and is justified only when one or
more of the following applies:

- the new module has an independent runtime responsibility;
- it introduces a reusable public interface;
- its dependencies should not be imposed on an existing package;
- it has an independent test and deployment lifecycle;
- keeping it in the current package would violate a module boundary.

The design note MUST state the proposed package owner, public interfaces,
dependencies, deployment targets, tests, and why an existing package is not
sufficient.

Do not add a package merely to avoid reading or understanding an existing
package. Do not create one package per small helper without an ownership or
deployment reason.

Reason: package fragmentation increases dependency, launch, build, and release
costs.

### 2.4 When to modify an existing package

Modify an existing package when the behavior belongs to its current owner and
the public contract remains compatible or is intentionally versioned.

Examples:

- Gazebo model and bridge changes belong in `slam_gazebo`.
- Localization provider changes belong in the localization layer.
- Nav2 parameters and launch belong in `navigation_bringup`.
- Operator workflow changes belong in the GUI/helper layer.
- Topic/frame/QoS contract changes require updates to the contract document and
  all readiness checks in the same change.

Do not place production algorithm changes in simulation packages, and do not
place temporary experiment logic in a production provider.

### 2.5 Separate solution tracks

The existing LiDAR ICP solution and this cuVSLAM solution MUST have separate:

- design documents;
- launch entry points;
- parameter files;
- recorded datasets and evaluation reports;
- runtime outputs and logs;
- acceptance criteria.

They MAY share generic evaluation tools, message definitions, and documented
frame conventions after review.

Reason: premature integration makes it impossible to tell whether a result came
from the new algorithm or from the old navigation stack.

### 2.6 Simulation and real-robot separation

Simulation-only code, worlds, timing workarounds, truth publishers, and bridge
adapters MUST be identifiable and MUST NOT be silently deployed into the real
robot workspace.

The current policy is:

- simulation changes deploy only to `/home/tseng/jetson_slam_ws`;
- real-robot changes deploy only to `/home/tseng/slam_ws` after explicit review;
- no automatic synchronization between the two;
- Gazebo truth MUST NOT be used as evidence of real localization accuracy.

Reason: simulation can hide sensor, timing, calibration, and mechanical errors.

## 3. Change Classification and Decision Gates

Every change MUST be classified before implementation.

### 3.1 Change classes

| Class | Examples | Required gate |
|---|---|---|
| C0 Documentation | Clarify a document without changing behavior | Review text and links |
| C1 Local implementation | Isolated helper or diagnostic with no contract change | Small design note and focused tests |
| C2 Interface change | Topic, frame, QoS, parameter, action, launch, or package dependency | Two-option design review and contract update |
| C3 Algorithm change | ICP, cuVSLAM, AMCL, controller, costmap, CUDA kernel | Experimental plan, baseline, metrics, rollback |
| C4 Safety or deployment change | `/cmd_vel`, timeout, emergency stop, real hardware, power | Human approval, simulation test, hardware test plan |
| C5 Architecture change | New solution track, namespace model, workspace split, map strategy | Architecture decision record and staged migration plan |

The higher class controls the required process even if the patch is small.

### 3.2 Mandatory design-before-code rule

C2-C5 changes MUST NOT begin implementation until a design proposal records at
least two options and compares:

- advantages;
- disadvantages;
- compatibility with current contracts;
- failure modes;
- deployment impact;
- maintenance cost;
- extensibility;
- rollback strategy;
- verification plan.

The selected option and rejected option MUST be recorded. A verbal decision is
not sufficient for a long-lived architectural change.

### 3.3 Analysis required before any implementation

The author MUST inspect:

- current package and launch ownership;
- current topic, TF, QoS, and parameter contracts;
- active and historical handoff state;
- existing uncommitted changes;
- target ROS 2 distribution and Jetson package availability;
- affected simulation and real-robot deployment paths;
- existing tests and known failure logs;
- safety implications and rollback path.

Reason: many previous robotics failures were caused by changing the wrong layer,
using stale evidence, or assuming a dependency existed on Jetson.

## 4. Standard Development Workflow

Every non-trivial change MUST follow these steps.

### Step 1: State the objective

Write one observable outcome, one non-goal, and the affected environment.
Example: "Validate visual map save/load in an isolated cuVSLAM workspace; do not
change the current LiDAR/Nav2 workspace."

### Step 2: Inspect before editing

Record relevant files, owners, contracts, current behavior, and existing
failures. Confirm the working tree and identify user changes. Never reset or
overwrite unknown work.

### Step 3: Classify and analyze risk

List the change class, affected modules, possible regressions, safety effects,
deployment targets, and rollback action.

### Step 4: Design when required

For C2-C5, present two options and obtain human approval for the selected one.
For C0-C1, a short design note is still required when a public behavior changes.

### Step 5: Define acceptance tests

Tests MUST be written before implementation in terms of observable signals:
topic samples, TF edges, lifecycle state, path error, command timeout,
tracking health, or measured resource use.

### Step 6: Implement the smallest change

Keep the patch inside the approved scope. Do not opportunistically format,
rename, reorganize, or upgrade unrelated files.

### Step 7: Verify in layers

Run the smallest fast checks first, then the relevant ROS integration and
hardware tests. Stop at the first failed gate and diagnose before proceeding.

### Step 8: Review evidence

Compare results with the baseline. A test that only proves the process started
does not prove the feature worked.

### Step 9: Update documentation

Update the applicable contract, handoff, test report, risk entry, and rollback
instructions in the same change.

### Step 10: Commit or preserve a reviewable worktree

Commit only a coherent verified unit. If the project is not yet under a single
Git root, preserve an equivalent change manifest and do not pretend the change
is versioned.

## 5. Risk Control

### 5.1 Protected assets

The following MUST NOT be modified, moved, renamed, or deleted without explicit
human approval:

- current production occupancy map content and its active `.pgm/.yaml` pair;
- AMCL tuning parameters;
- initial localizer algorithm;
- production `lidar_icp_odometry` algorithm;
- the backup directory `/Applications/slam_v2/backups/2026-07-08_154349`;
- user changes not created by the current task;
- real-robot workspace `/home/tseng/slam_ws` during simulation work;
- hardware power, motor, emergency-stop, or driver configuration;
- recorded evidence files used for an active acceptance result.

Reason: these assets define the current baseline or can affect physical safety.

### 5.2 Changes that must not be combined

The following changes MUST be separated into different reviewable units:

- algorithm changes and frame/TF changes;
- map changes and AMCL tuning changes;
- simulation timing fixes and production LiDAR ICP changes;
- Nav2 controller changes and velocity safety changes;
- namespace migration and a second robot's behavior changes;
- CUDA performance optimization and sensor calibration changes;
- dependency upgrades and feature implementation;
- cuVSLAM map design and integration into the existing Nav2 stack.

Reason: separating variables is necessary for meaningful regression diagnosis.

### 5.3 Regression containment

For every risky change, the author MUST provide:

- a known-good baseline command or dataset;
- a before/after metric;
- a rollback command or commit;
- a list of unaffected contracts;
- a list of expected changed signals;
- a safe-stop behavior if runtime validation fails.

### 5.4 Runtime safety gates

No navigation test may be called successful unless all are true:

- final command output is owned by the safety layer;
- command timeout produces zero velocity;
- localization and TF are valid at sensor timestamps;
- controller failure produces zero velocity;
- planner failure does not leave a previous command active;
- the operator has a tested stop procedure.

## 6. Git and Version Control

### 6.1 Repository policy

The project MUST eventually have one documented version-control boundary for
project-owned files. Until that migration is approved, nested repositories and
their uncommitted changes MUST be treated as independent protected assets.

Do not copy a package out of its owning repository merely to make a commit look
clean. Do not commit generated environments, logs, caches, or build products.

### 6.2 Branch strategy

Use:

- `main` or `trunk`: tested baseline only;
- `feature/<area>-<short-name>`: one coherent feature;
- `experiment/<method>-<dataset>`: research work with non-production results;
- `fix/<area>-<short-name>`: targeted defect correction;
- `release/<version>`: stabilization and acceptance only.

cuVSLAM work MUST remain in this independent workspace or its approved Git
branch. It MUST NOT be developed directly on the existing production navigation
branch.

### 6.3 Commit rules

Each commit MUST:

- have one purpose;
- build or pass the applicable validation gate;
- include required documentation or explicitly state why documentation is not
  applicable;
- avoid unrelated formatting and renames;
- be reversible without deleting user data.

Commit messages SHOULD use:

`<area>: <imperative outcome>`

Examples:

- `nav2: add measured command timeout gate`
- `cuvslam: record stereo-inertial mapping baseline`
- `docs: define visual map evaluation protocol`

Do not use messages such as `fix`, `update`, or `changes` without a measurable
purpose.

### 6.4 Commit size

One commit SHOULD modify no more than 8 project files. More than 8 files is
allowed only when the files form one inseparable contract migration and the
commit description lists every affected boundary.

More than 15 files, any package rename, or any cross-workspace deployment
requires a renewed design review.

## 7. ROS 2 Rules

### 7.1 Topic rules

Every production topic MUST have documented:

- name and namespace;
- message type;
- publisher owner;
- subscriber purpose;
- QoS reliability, durability, history, and depth;
- frame and timestamp meaning;
- rate or latency expectation;
- failure behavior.

Topic names MUST NOT be changed casually. A rename requires a compatibility
plan, remap strategy, readiness-check update, Foxglove update, and bag/replay
impact analysis.

Temporary diagnostic topics MUST use an explicit diagnostic namespace or clearly
documented prefix. They MUST NOT masquerade as production topics.

### 7.2 TF rules

Every dynamic TF edge MUST have one owner. Every static edge MUST have one
documented publisher. Frame IDs MUST describe physical or logical meaning and
MUST NOT be used as a display workaround.

The current single-robot contract is:

```text
map -> odom -> base_link -> laser_frame
                           -> camera_link / imu_link when present
```

For multi-robot operation, every non-shared frame MUST have a robot prefix.
`map` may be shared only when the robots intentionally use the same map.

An algorithm MUST NOT fix an apparent sensor orientation problem by silently
mirroring, rotating, or rewriting sensor data. First verify calibration, model
geometry, timestamps, and TF ownership.

### 7.3 Launch rules

Launch files MUST declare runtime inputs rather than embedding machine-specific
paths, credentials, or undocumented environment assumptions.

Launch files MUST make ownership and startup order explicit when QoS, static TF,
lifecycle, or discovery order affects correctness.

Simulation and real-robot launch files MUST be distinguishable. A simulation
truth publisher MUST NOT be included by a real-robot default launch.

### 7.4 Parameter rules

Parameters MUST be declared, typed, documented, and assigned a safe default or
made required. Every safety-critical parameter MUST state its unit and valid
range.

Runtime overrides MUST be recorded. A tuning change MUST include the baseline,
reason, test result, and rollback value.

AMCL tuning, footprint, velocity limits, and obstacle distances are not casual
parameters; they are acceptance-controlled configuration.

### 7.5 Node rules

Each node MUST have one primary responsibility, explicit startup/shutdown
behavior, useful diagnostics, and deterministic behavior when required inputs
are missing.

Nodes publishing velocity MUST define timeout and failure behavior. Nodes
publishing TF MUST log their ownership and target frames at startup.

Lifecycle nodes MUST be tested for configure, activate, deactivate, cleanup, and
shutdown paths when lifecycle is part of the runtime design.

### 7.6 Dependency rules

Adding a ROS, CUDA, Gazebo, or system dependency requires:

- target versions for Mac and Jetson;
- ABI and architecture compatibility review;
- package availability check on Jetson;
- build-time and runtime cost;
- license review;
- rollback or removal plan.

Large dependencies MUST NOT be installed directly on the robot. First propose
the package source, version, disk/RAM impact, and deployment procedure.

### 7.7 Namespace rules

Namespace design MUST precede multi-robot implementation. A robot namespace
must cover topics, services, actions, parameters, node names, model names, and
non-shared frame IDs consistently.

Before adding robot 2, robot 1 MUST run completely under its namespace and pass
the single-robot regression suite.

## 8. Testing and Acceptance Rules

Testing is incremental. The required level is determined by the change class
and affected layer.

### 8.1 Minimum test ladder

1. **Static inspection**: changed files, dependencies, parameters, and contract
   consistency.
2. **Build**: package build for the target workspace and target architecture.
3. **Unit test**: pure algorithms, transforms, parsers, limits, and state
   machines.
4. **Launch test**: startup, shutdown, lifecycle, and expected processes.
5. **Topic test**: type, QoS, frequency, timestamp, content, and timeout.
6. **TF test**: complete tree, owner uniqueness, timestamp lookup, and static
   transform delivery to late subscribers.
7. **Visualization test**: Foxglove/RViz or an equivalent recorded inspection
   for map, scan, TF, pose, plan, and costmap.
8. **Bag replay test**: deterministic replay for sensor and estimator changes.
9. **Integration test**: estimator, localization, costmap, planner, controller,
   and safety behavior together.
10. **Hardware test**: real sensors, real timing, real power, and safe-stop
    procedure.

Not every C0 document change needs every test, but every behavior change MUST
state which levels were run and why the others do not apply.

### 8.2 Required tests by change type

| Change | Minimum required evidence |
|---|---|
| Pure utility | Build, unit test, negative cases |
| Sensor driver or QoS | Build, launch, topic, timestamp, bag/replay |
| TF or frame | Build, TF tree, late-subscriber test, visualization |
| Odometry or SLAM | Unit, bag replay, trajectory metrics, resource metrics |
| Map or localization | Map integrity, TF, localization sequence, covariance/truth comparison |
| Nav2 planner/controller | Lifecycle, plan, motion, goal failure, zero-velocity failure |
| Costmap/obstacle layer | Sensor marking/clearing, footprint, inflation, obstacle scenarios |
| Velocity or safety | Timeout, limit, acceleration, node death, operator stop |
| CUDA | CPU reference comparison, correctness, determinism, profiling, fallback behavior |
| Namespace/multi-robot | Single robot regression, two robot graph isolation, simultaneous commands |
| Deployment | Clean install, target architecture, restart, rollback, log collection |

### 8.3 Metrics rules

Every estimator or navigation experiment MUST record both accuracy and system
health. At minimum:

- position and orientation error against a declared reference;
- drift over distance and time;
- timestamp age and jitter;
- dropped frames or scans;
- tracking or lifecycle health;
- CPU, GPU, memory, temperature, and callback time where relevant;
- command latency and final stop time;
- success, abort, timeout, and recovery counts.

Benchmark averages MUST NOT hide failure cases. Easy, hard, feature-poor,
occluded, dynamic, and long-duration cases must be reported separately when
they are relevant to the claim.

## 9. Refactoring Rules

### 9.1 When refactoring is allowed

Refactoring is allowed when it removes a demonstrated defect or a repeated
maintenance cost, and when behavior can be protected by tests or a migration
plan.

The proposal MUST name:

- the current pain;
- the intended invariant;
- the files and interfaces affected;
- the compatibility strategy;
- the tests that prove behavior is preserved.

### 9.2 When refactoring is prohibited

Do not refactor during an active precision experiment, safety incident, failed
deployment investigation, or acceptance run unless the refactor is the direct
fix and is separately identified.

Do not combine refactoring with algorithm tuning, map changes, namespace
migration, or dependency upgrades.

Large rename or module rewrite is prohibited without a staged compatibility
plan, intermediate build, and rollback point.

Reason: broad refactors destroy the baseline needed to explain a robotics
regression.

## 10. AI Developer Rules

AI-generated changes are subject to the same engineering gates as human changes,
with stricter scope controls because an AI can alter many related files quickly
without understanding runtime ownership.

### 10.1 Mandatory AI preamble

Before editing, the AI MUST provide:

- objective and non-goals;
- files it expects to inspect;
- change class;
- risk analysis;
- proposed tests;
- whether human design approval is required;
- explicit protected files it will not touch.

### 10.2 Scope limits

Default AI limits for one implementation turn:

- no more than 3 source/config files;
- no more than 1 ROS package;
- no more than 1 public topic/frame/parameter contract;
- no package creation, package rename, or workspace migration;
- no production algorithm and simulation workaround in the same change.

The AI MUST stop and request a renewed design when any limit is exceeded.

Exceptions require an approved contract migration plan and a file-by-file list.

### 10.3 Prohibited AI behavior

AI MUST NOT:

- modify protected assets without explicit approval;
- reset, checkout, clean, or overwrite unknown user changes;
- perform broad rename or rewrite based only on text search;
- infer that a process launch equals functional success;
- claim simulation success as real-robot success;
- install large dependencies without an approved plan;
- silently change topic, TF, QoS, parameter, or namespace contracts;
- edit production code to compensate for an unverified simulation defect;
- delete logs, bags, backups, or evidence to make a test pass.

### 10.4 AI verification obligation

The AI MUST run or explicitly report each applicable validation level. It MUST
return:

- changed files;
- behavior changed;
- tests run and exact outcome;
- tests not run and reason;
- residual risk;
- deployment status;
- documentation updated.

The AI MUST NOT say "complete" when a required test, deployment, or user
decision remains outstanding.

## 11. AI and Human Collaboration Workflow

### Phase A: Request framing

The human states the desired outcome, constraints, target environment, and what
must not change. The AI restates assumptions and identifies unknowns.

### Phase B: Investigation

The AI reads current architecture, contracts, logs, tests, and working-tree
state. It reports contradictions instead of silently choosing a convenient
interpretation.

### Phase C: Design decision

For C2-C5, the AI presents at least two options. The human selects or revises
one. The selection becomes an auditable decision record.

### Phase D: Small implementation

The AI changes only the approved scope. The human may interrupt after each
logical unit. Long-running experiments must have a stop condition and a safe
state.

### Phase E: Verification

The AI executes the test ladder, reports raw evidence locations, and separates
facts from interpretation. The human reviews safety-critical results.

### Phase F: Acceptance and record update

The human accepts the result or requests changes. Only then may the AI update
the handoff, baseline, and release record.

### 11.1 Human approval is mandatory for

- changes to safety behavior or final velocity output;
- changes to map, AMCL, production ICP, or initial localization;
- new hardware or power wiring;
- new ROS/CUDA/Gazebo dependency;
- topic, TF, QoS, namespace, or parameter contract changes;
- deployment to the real-robot workspace;
- architecture changes or new solution tracks;
- claims of precision, navigation success, or production readiness.

## 12. Documentation and Decision Records

The project MUST maintain:

- one current project handoff;
- topic/frame/QoS/data contracts;
- architecture decision records for C2-C5 decisions;
- experiment plans and result reports;
- hardware and calibration records;
- deployment and rollback instructions;
- known-risk register;
- release acceptance report.

Every decision record MUST include date, context, options, selected decision,
rejected alternatives, consequences, owner, and review trigger.

Documentation is part of the change, not a later cleanup task. Reason: in a
three-year robotics project, undocumented assumptions become accidental API.

## 13. Long-Term Maintenance Rules

For a project expected to live more than three years, the following are also
mandatory:

- record ROS 2, Ubuntu, JetPack, CUDA, compiler, GPU, camera firmware, and
  driver versions;
- maintain reproducible environment manifests and clean-install procedures;
- maintain calibration files with sensor serial number, date, method, and
  coordinate convention;
- define data retention, rosbag naming, metadata, and privacy rules;
- monitor CPU, GPU, memory, temperature, disk, DDS discovery, and dropped data;
- maintain fault injection tests for missing sensor, stale TF, delayed data,
  node death, planner failure, and command timeout;
- maintain a deprecation policy for topics, parameters, packages, and frames;
- require backward compatibility or migration notes for public interfaces;
- review licenses and third-party security advisories;
- perform periodic dependency and hardware lifecycle reviews;
- maintain an operator runbook for startup, shutdown, recovery, emergency stop,
  and evidence collection;
- maintain a reproducible benchmark suite rather than relying on one successful
  demonstration;
- require thermal and power validation for sustained Jetson/CUDA workloads;
- preserve the ability to run a CPU/reference implementation for correctness
  comparison when CUDA code is changed.

Reason: a robotics system fails over time through environment drift as often as
through source-code defects.

## 14. Exception Process

An exception MUST be written before the change and include:

- rule being bypassed;
- reason the default rule is infeasible;
- affected safety and maintenance risks;
- temporary controls;
- expiry or review date;
- responsible human approver.

"Urgent" does not mean "unrecorded". Emergency hardware actions may proceed
to a safe state first, but the exception record and post-incident review are
still mandatory.

## 15. Version 1.0 Self-Review

### 15.1 Conflict check

Potential conflicts were resolved as follows:

- Small changes may use a lightweight design note, while C2-C5 changes require
  two-option review. This preserves efficiency without weakening architecture
  control.
- The default AI limit is 3 files, but inseparable contract migrations may
  exceed it only with a file-by-file approved plan.
- Refactoring is permitted for proven maintenance problems but prohibited during
  active experiments or incidents unless it is the direct fix.
- Existing handoff documents describe current state; this document governs how
  that state may change.

### 15.2 Efficiency check

The rules are intentionally strict for safety, contracts, hardware, and
architecture, but lightweight for documentation, isolated diagnostics, and
small local fixes. The test ladder is risk-based rather than requiring a full
hardware test for every documentation or utility change.

### 15.3 Missing-rule check

The first review specifically added rules for:

- environment reproducibility;
- calibration provenance;
- data and rosbag retention;
- fault injection;
- thermal and power limits;
- CUDA reference correctness;
- deprecation and interface migration;
- operator runbooks;
- dependency and security review;
- evidence retention.

These are commonly missing in research-originated robotics projects and become
expensive after deployment.

### 15.4 Three-year commercial-project check

For a commercial project maintained for three years or more, future revisions
should add formal requirements for:

- product security threat modeling;
- SBOM and license compliance automation;
- CI on every supported architecture;
- hardware-in-the-loop and simulation-in-the-loop farms;
- requirements traceability from mission requirement to acceptance test;
- incident severity and field failure review;
- release support windows and rollback packages;
- fleet telemetry schema and privacy controls;
- controlled calibration and firmware rollout;
- change ownership and on-call escalation;
- formal safety case and hazard analysis before production deployment.

These are identified as Version 1.0 follow-up governance items, not silently
implemented as part of the current source change.

## 16. Immediate Adoption Checklist

Before the next implementation task:

- [ ] Confirm the task's change class.
- [ ] Inspect `PROJECT_HANDOFF.md`, contracts, package ownership, and working-tree state.
- [ ] Define objective, non-goals, protected files, and risk analysis.
- [ ] Present two options for C2-C5 changes.
- [ ] Define acceptance tests and rollback before editing.
- [ ] Keep the patch within the AI scope limit.
- [ ] Run the applicable build, launch, topic, TF, visualization, replay, and safety tests.
- [ ] Record evidence and residual risk.
- [ ] Update handoff, contracts, and decision records.
- [ ] Obtain human acceptance before real-robot deployment or production claims.
