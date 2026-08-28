# SSH Low-Rate Simulation Monitor

Date: 2026-07-28 (Asia/Taipei)

## Scope

- Change class: C2 operator observability interface.
- Environment: macOS localhost GUI observing Jetson simulation over
  key-based SSH.
- Observable objective: show the active simulation world, service state,
  required sensor interfaces, and cuVSLAM tracking state without changing the
  estimator or simulator.
- Non-goal: no image streaming, topic-rate benchmark, trajectory evaluation,
  ROS package, Jetson daemon, mapping algorithm, IMU fusion, Nav2, or
  real-robot behavior is added.

## Design Decision

Option A, selected by the user:

- Poll through SSH every 15 seconds.
- Reuse the existing localhost GUI backend and key-based SSH configuration.
- Advantages: no Jetson dependency or open network service, small rollback,
  and low runtime load.
- Disadvantages: delayed updates, SSH startup overhead, and no live image.
- Failure behavior: show Offline while keeping all control actions independent.
- Rollback: remove the monitor panel/API/controller methods from
  `tools/jetson_connection_web.py`.

Option B, rejected for this stage:

- Add rosbridge/WebSocket streaming.
- Advantages: lower-latency continuous ROS data and easier future image/point
  cloud display.
- Disadvantages: adds a Jetson package, open port, lifecycle owner, bandwidth,
  and security/configuration work before the level-zero mapping test needs it.

## Monitor Contract

The monitor is read-only. It samples:

- Baseline units:
  `cuvslam-d435i-gazebo.service`,
  `cuvslam-d435i-bridge.service`.
- Independent test units:
  `cuvslam-mapping-simple-gazebo.service`,
  `cuvslam-mapping-simple-bridge.service`.
- Five required D435i topics.
- `/scan`, `/ground_truth/odom`, and `/clock`.
- The custom cuVSLAM launch process and `/visual_slam/status` `vo_state`.

The browser polls only the localhost snapshot API. The backend performs one
low-rate SSH sample. User Start/Stop actions and monitor SSH samples are
serialized so they do not operate on the Jetson concurrently.

Ground truth is displayed only as an evaluation-source health signal. It is
not consumed by cuVSLAM and does not correct the visual pose.

## Display States

| State | Meaning |
|---|---|
| Healthy | One world is active, D435i and clock are ready, cuVSLAM reports `vo_state=1` |
| Sensors ready | One world and sensors are ready; cuVSLAM is stopped |
| Tracking issue | cuVSLAM runs but does not report `vo_state=1` |
| Sensor issue | Active simulation lacks D435i topics or `/clock` |
| Conflict | Baseline and test-world units are active together |
| Idle | Neither world is active |
| Offline | No configured SSH endpoint responds or the sample fails |

The work log records the first sample, state changes, failures, and one
heartbeat every four successful samples. It does not log every browser refresh.

## Verification

Completed locally:

- Python syntax compilation: passed.
- Generated monitor Bash syntax check: passed.
- Healthy baseline parser fixture: passed.
- Baseline plus test conflict fixture: passed.
- Fully stopped fixture: passed.
- Monitor thread start, first update, and stop lifecycle: passed.
- Required HTML monitor element IDs: present.
- Localhost API monitor Start/Stop/Start lifecycle: passed.
- Browser visual inspection: passed; the monitor panel is directly below the
  connection section and fully visible in the initial control-pane viewport.

Completed on Jetson after SSH reconnection, 2026-07-28:

- Live SSH sample used `192.168.55.1:22`.
- World classification: `Baseline`.
- Baseline Gazebo and bridge were active; the independent test-world units
  were inactive.
- D435i readiness: 5/5 topics; `/scan` and `/ground_truth/odom` were ready.
- cuVSLAM reported `Tracking (vo_state=1)`.
- Four successful samples produced one initial update and one heartbeat after
  one minute, without duplicate per-sample work-log entries.
- Stopping and restarting the monitor did not stop or restart Gazebo, bridge,
  or cuVSLAM; the restarted monitor again reached Healthy.

## Residual Risk

- Topic presence and publisher discovery are health indicators, not frequency,
  latency, image quality, or mapping accuracy measurements.
- One sample may take several seconds while waiting for
  `/visual_slam/status`; the next sample begins 15 seconds after completion.
- SSH address changes are handled through the configured candidate list, but
  a fully disconnected Jetson remains Offline.
- The current GUI still starts only the baseline world. Test-world launch
  remains a separately controlled workflow.
