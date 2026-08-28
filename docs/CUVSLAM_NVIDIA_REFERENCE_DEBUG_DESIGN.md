# cuVSLAM NVIDIA Reference and Debug-Dump Design

Date: 2026-07-30
Change class: C2 launch parameter interface, diagnostic only

## Objective

Separate the installed cuVSLAM runtime from the Gazebo stereo input by:

1. replaying NVIDIA's release-2.1 `small_pol_test` rosbag through the installed
   VisualSlamNode; and
2. running an exact one-metre Gazebo test with the NVIDIA debug dump enabled.

TF, camera calibration, estimator output frames, and cuVSLAM algorithm
parameters remain unchanged.

## Options

### Option A: Permanently enable the debug dump

Advantages:

- no launch argument is required during a diagnostic;
- every run produces input evidence.

Disadvantages:

- continuously writes images and metadata to Jetson storage;
- changes the normal GUI workflow and can consume storage without warning;
- makes a diagnostic side effect part of the default runtime.

### Option B: Add opt-in launch arguments

Add `enable_debug_mode` and `debug_dump_path` launch arguments. Keep debug mode
disabled by default and enable it explicitly for controlled tests.

Advantages:

- preserves existing runtime behavior;
- makes the diagnostic path reproducible;
- prevents accidental continuous disk use.

Disadvantages:

- test commands must pass two explicit launch arguments.

## Decision

Use Option B. This is a backward-compatible launch interface extension. The
normal GUI continues to launch with debug mode disabled.

## Acceptance

- Existing launch-contract tests pass.
- Default `enable_debug_mode` remains `false`.
- An explicit debug run creates the configured dump.
- NVIDIA reference playback produces odometry from every left image.
- Gazebo test uses native 6DoF truth and commands at least 1.0 m travel.
- All test processes and simulation services are stopped after evidence is
  collected.
