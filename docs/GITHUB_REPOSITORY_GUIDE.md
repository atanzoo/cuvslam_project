# GitHub repository guide

This project contains the cuVSLAM development source, deployment notes,
calibration contracts, experiment reports, and the local monitoring GUI. It
also contains historical simulation work and machine-generated runtime data;
those items are deliberately kept out of the public source repository.

## Commit to GitHub

- `tools/` — repeatable GUI, launch, inspection, and analysis scripts.
- `deployment/` — Isaac ROS deployment boundary and simulation adapter source.
- `config/` — shareable RViz and Xorg configuration, plus
  `local.env.example`.
- `calibration/`, `docs/`, and `reports/` — contracts, decisions, and curated
  evidence.
- `models/` — readable robot description sources needed by the project.
- Root handoff and engineering documents.

## Keep local only

The root `.gitignore` excludes Python environments, ROS build products, model
checkpoints, temporary conversions, backups, third-party checkouts, downloaded
papers, raw recordings, and high-volume GUI traces. This keeps the repository
reviewable and avoids publishing data that is either machine-specific,
generated, copyrighted, or too large for normal source control.

The latest R2 correction record remains intentionally preserved at
`logs/real_d435i_quality/20260825_r2_correction_log.md`. New raw traces should
remain local; summarize important findings in `reports/` or `docs/` instead.

## Local configuration

1. Copy `config/local.env.example` to `config/local.env` on the Mac.
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
absolute paths, or generated checkpoints are included. The repository is not a
replacement for the Jetson image or the Isaac ROS SDK; those dependencies are
installed separately according to `deployment/isaac_ros/README.md`.

