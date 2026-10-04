# GitHub repository guide

This project contains the cuVSLAM development source, deployment notes,
calibration contracts, experiment reports, and the local monitoring GUI. It
also contains historical simulation work and machine-generated runtime data;
those items are deliberately kept out of the public source repository.

## Commit to GitHub

- `real_robot/cuvslam/tools/`, `simulation/cuvslam/tools/`, and
  `shared/tools/` — repeatable GUI, launch, inspection, and analysis scripts.
- `real_robot/cuvslam/deployment/`, `simulation/cuvslam/deployment/`, and
  `shared/deployment/` — separated Isaac ROS deployment boundaries.
- `shared/config/` and workstream `config/` folders — shareable RViz and
  Xorg configuration, plus `shared/config/local.env.example`.
- `simulation/path_planning/tools/` and `simulation/path_planning/nav2_map3/`
  — navigation research source and offline contract tests; incomplete runtime
  integration is identified in `PROJECT_HANDOFF.md`.
- `real_robot/cuvslam/calibration/`, `docs/`, and workstream evidence folders — contracts, decisions, and curated
  evidence.
- `simulation/path_planning/models/` — readable robot description sources needed by the project.
- Root handoff and engineering documents.

## Keep local only

The root `.gitignore` excludes Python environments, ROS build products, model
checkpoints, temporary conversions, backups, third-party checkouts, downloaded
papers, raw recordings, and high-volume GUI traces. This keeps the repository
reviewable and avoids publishing data that is either machine-specific,
generated, copyrighted, or too large for normal source control.

The latest R2 correction record remains intentionally preserved at
`real_robot/cuvslam/evidence/logs/real_d435i_quality/20260825_r2_correction_log.md`. New raw traces should
remain local; summarize important findings in the owning workstream's
`evidence/reports/` or `docs/` instead.

## Local configuration

1. Copy `shared/config/local.env.example` to `shared/config/local.env` on the Mac.
2. Set the Jetson host/user and the local VNC password.
3. Create `/home/tseng/isaac_ros_ws/.local.env` on the Jetson with the same
   `CUVSLAM_VNC_PASSWORD` value.
4. Confirm that SSH key authentication is already configured before using the
   GUI.

The VNC password is intentionally not present in source code, logs, or status
messages. If the password has ever been exposed in a log or screenshot, change
it before making the repository public.

## Pre-push checklist

From the project root:

```text
git status --short --ignored
git diff --check
```

Review the staged file list and confirm that no credentials, raw bags, local
configuration, generated checkpoints, or new hard-coded workstation paths are
included. Historical evidence may retain its original acquisition paths; that
does not make those paths portable setup instructions. Application documents
and portraits under `output/` remain local and are not project source.
The repository is not a
replacement for the Jetson image or the Isaac ROS SDK; those dependencies are
installed separately according to `shared/deployment/isaac_ros/README.md`.

The workstream migration does not change this policy: moved raw traces,
downloaded recordings, binary costmap dumps, and the generated URDF archive
remain ignored at their new locations. Curated reports and small acceptance
summaries can be versioned. Do not remove local evidence to make an upload pass.
