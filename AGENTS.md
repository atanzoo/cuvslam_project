# cuVSLAM Project Instructions

This file is the repository-level operating contract for AI agents and human
contributors working in this project. It applies to the entire repository.

## Required reading and authority

Before analyzing, editing, launching, deploying, or deleting anything:

1. Read `ENGINEERING_GUIDELINES.md` completely. It is the mandatory engineering
   policy and the authority for architecture, safety, testing, Git, and AI work.
2. Read `PROJECT_HANDOFF.md` to understand the current architecture, milestone,
   known limitations, and next workstream.
3. Read only the relevant documents listed by `docs/README.md` for the task.

When documents disagree, follow this order:

1. Explicit user safety or hardware instruction.
2. `ENGINEERING_GUIDELINES.md`.
3. `PROJECT_HANDOFF.md` and current data contracts.
4. Package/design documents.
5. Historical handoffs, logs, and patch notes.

`ENGINEERING_GUIDELINES.md` defines how the project may change. `PROJECT_HANDOFF.md`
defines the current state. Historical records are evidence, not current policy.

## Before making a change

State briefly:

- objective and non-goals;
- files and runtime components in scope;
- change class (`C0`–`C5`);
- risks and protected assets;
- acceptance tests and evidence to collect;
- whether human approval is required.

For `C2`–`C5` changes, provide the required design options, trade-offs,
failure modes, rollback plan, and verification plan, then wait for approval
before implementation. Do not silently turn an experiment into a production,
hardware, safety, navigation, or public-interface change.

## Non-negotiable project rules

- Prefer evidence over implication. Every performance, localization,
  navigation, safety, or hardware claim must identify environment, hardware,
  ROS/middleware/parameters, procedure, metric, and evidence.
- Keep hardware/drivers, sensor normalization, estimation, mapping/localization,
  navigation, safety, and observability as separate layers.
- At runtime, keep one owner for each of `map -> odom`, `odom -> base_link`,
  final `/cmd_vel`, authoritative map, model command input, and lifecycle group.
- Keep the cuVSLAM and existing LiDAR/Nav2 solution tracks separate until an
  evidence-based integration review approves the boundary.
- Keep simulation/replay and real-robot deployment separate. Simulation results
  are never evidence of real-robot accuracy.
- Never delete logs, bags, maps, calibration, backups, screenshots, or other
  acceptance evidence to make a result pass. Preserve user changes.
- Do not put credentials, tokens, local environment files, generated runtime
  data, or build products into Git.
- Do not commit, push, deploy to Jetson, change safety behavior, or alter a
  protected map/config unless the user explicitly requests that operation.

## Verification and handoff

Use the smallest applicable verification ladder: static inspection, build or
compile, unit checks, launch checks, topic/QoS checks, TF checks, visualization,
bag replay, integration, and hardware checks as applicable. Do not claim a test
was run if it was not run.

After a material change, report:

- changed files and behavior;
- tests run and tests not run;
- evidence paths;
- remaining risk, rollback, and deployment status;
- documentation updated or intentionally not updated.

Update `PROJECT_HANDOFF.md` when architecture, milestone, limitation, or next
step changes. Put experiment results in `reports/` or `logs/` according to the
existing folder rules. Keep one current handoff and use `docs/README.md` as the
document index.

## Scope reminder for future conversations

These instructions apply automatically when work is performed from this
repository. A new conversation that is not opened in this repository may not
load this file; in that case, start by specifying:

`/Users/tsengpochien/Desktop/cuvslam_project`

and ask the agent to read `AGENTS.md`, `ENGINEERING_GUIDELINES.md`, and
`PROJECT_HANDOFF.md` before taking action.
