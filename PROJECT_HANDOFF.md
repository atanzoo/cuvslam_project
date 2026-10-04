# cuVSLAM Odometry and Navigation Integration Handoff

> This handoff covers the real D435i/cuVSLAM odometry development line and its
> integration boundary with the 3D LiDAR SLAM and Nav2 stack.
>
> As of 2026-08-28, cuVSLAM visual mapping/localization is no longer a required
> production objective. Historical mapping and simulation work is preserved
> below as evidence, but the active architecture uses cuVSLAM as local
> visual-inertial odometry and assigns global mapping/localization to 3D LiDAR
> SLAM.

- Last handoff update: 2026-10-02 (Asia/Taipei)
- Project root: `<repository-root>`
- Active current-state entry point: `docs/CUVSLAM_CURRENT_STATUS_20260825.md`
- Latest real-camera correction record: `real_robot/cuvslam/evidence/logs/real_d435i_quality/20260825_r2_correction_log.md`
- GitHub repository: `https://github.com/atanzoo/cuvslam_project` (private;
  unchanged `main` at `9043a71`). The October 4 research snapshot is published
  on `codex/github-workstream-update-20261004` for draft review:
  `https://github.com/atanzoo/cuvslam_project/pull/1`. This is publication, not
  new runtime acceptance; see `docs/GITHUB_UPDATE_PREPARATION_20261003.md`.
- Content classification: `docs/PROJECT_CONTENT_MAP.md`
- Physical layout: `docs/PROJECT_LAYOUT.md`
- Current milestone: real D435i stereo+IMU cuVSLAM odometry accepted for the
  tested operating envelope; the current-space emitter A/B baseline is
  cuVSLAM-only odometry, and production robot/Nav2 integration is next.
- Current target architecture: cuVSLAM owns local odometry; 3D LiDAR SLAM owns
  the global map and localization; Nav2 owns planning, costmaps, and control.
- Note: sections below preserve earlier simulation and visual-mapping history.
  The architecture decision in the current handoff supersedes older objectives
  that required cuVSLAM map save/load or visual relocalization before Nav2.
- Historical simulation status: NVIDIA reference passed; wheel odometry is isolated; the D435i
  200 Hz IMU contract is verified; stereo-only in-place 90-degree pose now
  passes two independent cold starts in the dense-turn v3 world
- Validated development baseline: NVIDIA Jetson AGX Orin with Intel RealSense
  D435i
- Intended robot deployment: AgiBot D1 Max using its onboard Orin NX 16 GB,
  built-in front/rear 96-line LiDARs, and an externally mounted D435i; this is
  an engineering-feasible target, not yet a completed real-robot validation.
- Real-camera mapping entry point: `docs/REAL_D435I_MAPPING_HANDOFF_20260817.md`
- Historical simulation implementation status: Isaac ROS Visual SLAM 2.1.0 / cuVSLAM 11.4
  runs in the isolated Jetson container. The official NVIDIA release-2.1
  stereo bag produces complete odometry output. A controlled baseline 1 m run
  proved that every in-motion stereo frame was unique while cuVSLAM still
  underestimated translation by 92.6 percent. An independent near-field
  observable-world A/B then produced 1.0232 m for 1.0128 m truth without
  estimator or calibration changes. Two additional cold-start trials produced
  1.0189 m for 1.0176 m truth and 0.9708 m for 1.0152 m truth. The sparse
  far-field Gazebo scene, not stale pixels, was the root cause of the current
  1 m simulation failure.

### D1 Max / Nav2 isolated simulation gate (2026-10-01)

This is a **separate navigation-simulation research branch**, not the real
robot Nav2/odometry integration milestone above. On Jetson, an isolated Foxy
Nav2 map server and planner in ROS domain 43 accepted a scenario-specific 3×
derived map containing the known fixed obstacle. The original candidate-map
YAML `free_thresh: 0.25` caused Foxy to treat gray-205 Unknown pixels as Free;
the separate Nav2-only export uses 0.19, with source YAML/PGM unchanged. One
real Nav2 path passed fixed-obstacle, Unknown, costmap, and boundary geometry
checks. A local MuJoCo proxy using that path, with the fixed object active and
pedestrian disabled, had no physical contact or map violation but **timed out
at 70 s**, 2.066 m from the goal. Thus the closed-loop gate failed and no new
25k PPO training has started. The planner-only Jetson session was stopped;
there was no controller, `/cmd_vel`, formal workspace deployment, or real robot
motion. See `research/path_planning/NAV2_IN_LOOP_MAP3_PPO_DESIGN_20261001.md`
and `simulation/path_planning/evidence/20261001_nav2_map3_jetson_path_and_proxy_gate.md`.

Next as of 2026-10-01 (superseded by the 2026-10-02 follow-up below): review a
coordinated costmap-aware path-following/control design, then
complete the approved isolated Nav2 control bridge and fixed-only/mirrored/
active-person closed-loop gates before considering fresh training. A bounded
in-memory test that merely added path-heading yaw in AVOID **collided with the
fixed object** at 46.288 s; it was not adopted. Do not treat a safe global path
as evidence that the current controller can safely reach the goal.

### D1 Max / Nav2 controller-in-loop follow-up (2026-10-02)

The approved true Nav2 controller-in-loop bridge is now running only in the
isolated Jetson Foxy namespace/domain and a local MuJoCo D1 Max kinematic proxy.
With the fixed box active, pedestrian disabled, known-Unknown blocked, and a
10 Hz Regulated Pure Pursuit controller capped at 0.22 m/s, the same 3x Map-3
scene passed one forward and one reverse ~10 m traversal. Both actual Jetson
`FollowPath` action results are `succeeded`; the local proxy reports zero
physical/geometric collision, zero map violation, zero timeout, and endpoint
error below 0.25 m. Minimum simulated center-to-box-edge distances were 0.897 m
forward and 0.804 m reverse, above the 0.61 m required margin. The reverse
local runner's raw JSON says `fail` only because it missed the terminal-status
transition in a race; its saved local trace has result code 4 and is paired
with the matching Jetson action result (`succeeded`) in a separate adjudication
record. The raw record is preserved unchanged. One earlier forward harness
attempt stopped at its local goal tolerance before Nav2 reported success and
was retained as a failed attempt.

This is a **one-scene, two-direction static-obstacle closed-loop pass**, not a
general fixed-obstacle/boundary safety acceptance. There is no dedicated
boundary-approach suite, mirrored placements, active pedestrian, gait model,
or real-robot test yet. No new PPO training has started. Keep the next gate
bounded to mirrored/varied fixed-object and map-boundary scenes; inspect those
before enabling pedestrians, and only consider training after those gates.
Full environment, run conditions, evidence paths, and limitations:
`simulation/path_planning/evidence/20261002_nav2_rpp_mujoco_fixed_gates.md`.

### D1 Max / Nav2 MPPI speed-headroom follow-up (2026-10-02)

The separately approved Humble Nav2 MPPI architecture now has a v3 isolated
profile with only `vx_max` reduced from 0.30 to 0.28 m/s. The bridge's hard
limits (0.30 m/s forward, 0.02 m/s lateral, 0.70 rad/s yaw), watchdog, static
clearance, and Unknown blocking remain unchanged. The preceding v2 boundary
attempts stopped on actual overspeed commands; those failed records remain.

On Jetson domain 178 with the Mac MuJoCo D1 Max kinematic proxy, v3 passed
the 10.174 m boundary-approach route in 37.568 simulated seconds and the
10.962 m original fixed-object route in 40.808 seconds. Both actual Nav2
`FollowPath` actions succeeded, with zero contact/geometric collision, map
violation, or timeout. Minimum center-to-fixed-box-edge distances were
1.178471 m and 0.932213 m (required 0.61 m). The paired command recorders
captured 378 and 411 ROS commands with no nonfinite or overspeed samples.
The runtime is stopped and preserved; no source map, production configuration,
motor/sensor runtime, or real-robot acceptance status changed.

This qualifies **two static routes in one scene**, not the mirrored/varied
fixed-placement suite, IMM pedestrian costs, PPO-to-Nav2 preference interface,
or controller compute-time deadline. No new PPO pilot has started. On
2026-10-02 the user explicitly waived additional mirrored/varied fixed-object
placement testing as a prerequisite and directed progression to IMM/PPO
interface acceptance. Those placements remain untested, not passed. Complete
the dynamic-prediction and policy-input gates before the planned 25k-step/70 s
pilot; retain unchanged collision, clearance, Unknown, command-limit, and
watchdog requirements. The user subsequently selected the custom critic
Option A and approved its 15-file, one-package/one-contract staged scope,
requesting Luna max implementation with parent review until interface
completion. G1 input contract passed parent review (33 focused unit tests,
Python 3.8 syntax and actual perceived-tracker serialization); G2 ARM64 critic
implementation is in progress. G2 build/load, G3 IMM integration,
and G4 five-preference/SB3 interface checks are not yet accepted. This
orchestration does not automatically start a 25k learning run. See
`research/path_planning/NAV2_MPPI_PPO_IMM_INTERFACE_DESIGN_20261002.md`.
The legacy local
`ObstacleMPPI` policy is not an already integrated Nav2 policy. Evidence and
rollback: `research/path_planning/NAV2_MPPI_BOUNDARY_GATE_COMMAND_HEADROOM_20261002.md`.

### Repository layout update (2026-08-31)

The active project files are now physically separated by execution workstream:

- `real_robot/cuvslam/` contains real D435i/cuVSLAM launch, calibration, tools,
  and real-camera evidence.
- `simulation/cuvslam/` contains Gazebo/replay cuVSLAM launch, adapters, tools,
  and simulation evidence.
- `simulation/path_planning/` contains D1 Edu models, maps, path-planning
  tools, and navigation simulation assets.
- `research/path_planning/` contains the dated PPO/IMM/MPPI research records.
- `shared/` contains cross-workstream connection, deployment, and display
  helpers. Root `.command` files remain compatibility entry points.

This is a repository layout change only. It does not add RPLIDAR, Nav2, a new
TF owner, or a hardware deployment. Simulation and real-robot work remain
separate, and historical evidence was moved without rewriting its contents.

### Jetson runtime organization update (2026-08-31)

The Jetson Isaac ROS workspace was reorganized in place to match the repository
workstream boundary without copying the large source, bag, or Docker layers.
Real D435i runtime files now live under
`/home/tseng/isaac_ros_ws/real_robot/cuvslam/`, simulation files under
`/home/tseng/isaac_ros_ws/simulation/cuvslam/`, and real D435i data under
`/home/tseng/isaac_ros_data/real_robot/cuvslam/`. Old top-level paths remain
compatibility symlinks. The migration and approved evidence cleanup are
recorded in `research/cuvslam/jetson_workspace_organization_20260831.md`.

This is an interim storage-constrained separation inside the Isaac ROS
workspace, not a claim that a fully independent real-robot workspace has been
deployed. No RPLIDAR package, mapping node, TF owner, Docker image, or hardware
runtime behavior was changed.

### RPLIDAR A2M12 integration (2026-08-31)

The first real 2D LiDAR integration has been staged as a separate experimental
workstream. The intended ownership is cuVSLAM odometry-only for `odom ->
base_link`, static sensor extrinsics for the D435i and A2M12, and `slam_toolbox`
as the sole `map -> odom` publisher. The existing R2 runner, RViz container,
external LiDAR/Nav2 baseline, and historical evidence remain protected.

The A2M12 is visible on the Jetson as `/dev/ttyUSB0` through a Silicon Labs
CP2102 adapter. The official `sllidar_ros2` package was added to the Jetson
workspace and compiled for the target container. The launch uses zero-valued,
explicitly temporary camera and LiDAR extrinsics, so only the stationary
wiring/data-path gate is accepted. Full details, runtime evidence, warnings,
and rollback are recorded in
`research/cuvslam/real_d435i_rplidar_integration_20260831.md`.

The stationary gate passed: A2M12 `/scan` is approximately `12.2 Hz`, D435i
images and CameraInfo are active, cuVSLAM publishes `odom -> base_link` at
approximately `27.2 Hz` with `vo_state=1`, and slam_toolbox's dynamic map query
returned a `195x117` map containing `239` occupied cells. The first start
failed at D435i `Resetting device...` because it inherited `initial_reset=true`;
the integrated launch now defaults to `initial_reset=false`, and the second
start reached the odometry/map data path. This is not motion, extrinsic, or
map-accuracy acceptance; Nav2 and `/cmd_vel` remain out of scope.

On 2026-09-03, the updated live telemetry collector and the mapping runner were
synced to the Jetson canonical real-robot tools path. The GUI now routes
`mapping` to the isolated D435i + A2M12 + slam_toolbox runtime and keeps `odom`
on the existing R2 runner. The stationary check reached `/scan` at about
12 Hz, cuVSLAM odometry at about 30 Hz with `vo_state=1`, and a `199x116`
`/map`; the collector emitted map, scan, odom, and map-frame path data. A
light rosbag smoke test was also recorded and stopped cleanly under
`real_d435i_rplidar/`. The mapping container was stopped after verification.

The runner and collector were syntax-checked remotely, and dated backups were
preserved. This remains a stationary data-path result only: the launch still
uses zero extrinsics, slam_toolbox produced a small number of queue-drop
warnings, and no motion, map-quality, Nav2, or `/cmd_vel` test was run.

The mapping stop flow was subsequently clarified and synchronized: stopping
the mapping runtime stops the in-container A2M12 `sllidar_node` together with
the D435i, cuVSLAM, and slam_toolbox. The separate `real_d435i_rviz`
observation container is intentionally left running. The deployed runner's
rollback copy is
`/home/tseng/isaac_ros_ws/real_robot/cuvslam/tools/run_real_d435i_rplidar.sh.bak_20260903_stop_motor_result`.
The stop test received the driver `/stop_motor` response and left no process
holding `/dev/ttyUSB0`.

### Stage 1 low-light diagnostic plumbing (2026-09-03)

The local repository now forwards GUI-selected mode, IR profile, lighting
profile, and diagnostics mode into both the R2 odometry runner and the separate
D435i + A2M12 mapping runner. `low_light_motion` is an experimental manual
profile (`10,000 us`, gain `24`, IR projector enabled); it is not yet an
accepted operating point. D435i IR metadata is recorded for both streams, and
full evidence bags retain metadata plus cuVSLAM observation/pose-graph topics.
The GUI assessment now exposes image-quality flags and an explicit turn
geometry gate, while the live quality panel displays measured IR exposure and
gain when metadata is available. The work-record panel also provides a
separate `複製工作記錄（過程）` action for chronological process output,
keeping it distinct from the conclusion summary. Each run records Requested
runtime values, Actual D435i metadata, quality/geometry state changes, and a
final quality/geometry block before the session is saved.

This is runtime/evidence preparation only. It does not add LiDAR odometry, EKF
fusion, Nav2, `/cmd_vel`, a new TF owner, or calibrated extrinsics. Stage 1 has
now been synchronized to Jetson after dated backups were created; remote Bash
and Python syntax checks passed, and no container was started or stopped during
the sync. No low-light motion acceptance test has been run. The existing
stationary A2M12 data-path result and all historical evidence remain valid but
are not upgraded by this change.

### Stage 1 official-profile mapping run review (2026-09-03)

The first post-plumbing R2 + A2M12 mapping run (`14:29:25`--`14:32:09`,
`official`, `640x360x30`, `motion=other`) had healthy transport metrics:
approximately 29.7 Hz stereo input, 30 Hz odometry, 0 us reported stereo-sync
P95, and 100 percent raw cuVSLAM VO-valid samples. It is not, however, an
accuracy PASS. The process log recorded transient `clipped`/`stereo:desync`
flags, turn-jitter REVIEW, and two turn-translation FAIL observations, with a
worst reported XY displacement of `0.696 m` at `89.8 deg`. The final closure
residual was approximately `0.265 m`, above the `0.20 m` review reference, but
the motion label was `other`, so this run is not treated as a formal pure-turn
or closed-loop acceptance result.

The saved session is
`real_robot/cuvslam/evidence/logs/real_d435i_quality/20260903_143209_r2_session.json`
and its full trace is
`real_robot/cuvslam/evidence/logs/real_d435i_quality/20260903_142925_r2_trace.jsonl`.
The historical JSON is preserved unchanged. The GUI assessment was corrected
after this run to retain any observed image flags and the worst geometry gate
even when the final live snapshot returns to `none`/`未偵測`; future summaries
will therefore report REVIEW/FAIL evidence instead of allowing a healthy
data-rate score to appear as an overall PASS.

### GUI process-log and scan-map observability correction (2026-09-11)

The real D435i + A2M12 GUI now has a separate `清除工作記錄（過程）` action.
It clears only the in-memory chronological display; JSONL traces, session files,
mapping summaries, and downloaded evidence are preserved. The existing copy
action remains available.

The live collector now projects each cuVSLAM odom sample into `map` using the
`map -> odom` transform at that sample's timestamp. It no longer reprojects the
complete historical path with the latest transform, which could make the blue
path move when slam_toolbox updated its correction.

Latest `/scan` telemetry also reports a current occupancy-grid alignment
diagnostic: endpoint ratios within 0.15 m and 0.30 m, plus normal/mirrored/
0/±90/180-degree orientation candidates. The distance field is rebuilt when
the map revision changes. This is observation only; it does not modify TF
ownership, odometry, slam_toolbox parameters, EKF weights, or the authoritative
map.

The collector now includes LaserScan timing telemetry: `header.stamp`,
`scan_time`, `time_increment`, estimated beam-acquisition span and end stamp,
the timestamp requested from TF, the returned TF stamp when available, and
latest-TF fallback information when the timestamped lookup fails. The GUI
records a compact timing entry in the process log and preserves a
`scan_telemetry` record without storing the full point array in the trace.

Local verification passed: Python compilation, the existing
`REAL_D435I_RPLIDAR_CONTRACT_PASS` contract test, and an in-memory process-log
clear exercise. The timing parser, process-log entry, and compact
`scan_telemetry` trace record were also exercised locally. This timing update
was synchronized to the Jetson canonical tools path
`/home/tseng/isaac_ros_ws/real_robot/cuvslam/tools/`. The collector and
contract-test source hashes matched the local files, and remote Python
compilation passed. A recoverable pre-sync backup is retained at
`/home/tseng/isaac_ros_ws/backups/scan_timing_20260911_132541/`. No live ROS
or hardware run was performed; the GUI remains a Mac-side file.

Detailed record:
`research/cuvslam/real_d435i_scan_map_observability_20260911.md`

### Scan timestamp/TF release gate (2026-09-11)

The next B+A correction is now implemented locally. The A2M12 driver output is
remapped to `/scan_raw`; `scan_tf_gate.py` waits for the exact scan timestamp
to become available in `odom -> rplidar_link` TF, then republishes the original
LaserScan unchanged as `/scan`. The gate is bounded (2 s, 32 pending scans),
does not publish TF, and does not change the cuVSLAM or slam_toolbox owners.
Timed-out or overflowed scans are explicitly dropped and reported.

The mapping bag now records both `/scan_raw` and gated `/scan`. The collector
and Mac GUI expose scan timestamp age and scan-end age so the next run can
verify whether the roughly 1.1 s visual/scan TF lag is being absorbed before
the mapper sees a scan. This addresses timestamp coverage first; it does not
deskew the 78--85 ms scan acquisition interval or claim improved map accuracy.

Local Python compilation, shell syntax checking, and
`REAL_D435I_RPLIDAR_CONTRACT_PASS` passed. Jetson synchronization and a
stationary/hardware gate remain required before interpreting the next map.
Rollback is to start the runner with `CUVSLAM_SCAN_TF_GATE=false` (or pass the
twelfth runner argument `false`) or restore the dated Jetson backup; the raw
scan remains preserved in recorded bags.

The first corrected-runtime restart exposed an implementation error: the
plain Python `scan_tf_gate` was given ROS-only `--ros-args` and exited with
argument parsing code 2. The launch command now passes only its own argparse
options; this fix is being synchronized before the next stationary check.

The corrected stationary smoke check confirmed `scan_tf_gate pending=0`,
continuous `/scan` publication at about 12 Hz, one `/scan` publisher and one
subscriber, and a valid `/map` OccupancyGrid message. It dropped 236 scans
during the approximately 19-second cuVSLAM GPU warm-up, then published with
typical waits below 35 ms. The map data path is therefore restored; mapping
quality and motion acceptance remain untested.

### GUI delayed exact-TF scan display correction (2026-09-11)

The mapping data path was healthy, but the GUI collector retained only the
newest `/scan` and required the complete `map -> rplidar_link` transform at
that scan's exact timestamp. With slam_toolbox's observed approximately
one-second `map -> odom` lag, the collector could stay at `TF_WAIT` and clear
the displayed scan even though gated `/scan` continued to publish.

The collector now keeps bounded monitor-side queues for scans and cuVSLAM
odom samples. It displays the newest sample whose complete map transform is
available at the sample timestamp, retains the last valid scan during a
temporary wait, and exposes `OK_DELAYED`, `display_delay_ms`, queue depth, and
queue drops in telemetry. Map-frame path projection uses the same delayed
exact-timestamp policy. This changes only GUI observability; it does not
compose mixed-time transforms, change `/scan`, tune slam_toolbox, publish TF,
or change cuVSLAM/LiDAR/EKF ownership.

Local syntax and contract checks passed. The collector was synchronized to
`/home/tseng/isaac_ros_ws/real_robot/cuvslam/tools/collect_real_d435i_odom_live.py`
after creating the recoverable backup
`/home/tseng/isaac_ros_ws/backups/collector_delayed_scan_display_20260911_160726/`.
The local and remote SHA-256 is
`750a3d46c9d9bcffdc4588b7aa7f2809e36cebe0a74ce1db1116bf0fafe0c5e0`, and
remote Python compilation passed. The existing mapping and RViz containers
were not restarted; the updated collector takes effect when the GUI telemetry
connection is recreated. A stationary post-sync check remains required before
calling the GUI display behavior verified.

The synchronized collector also passed a bounded 12-second read-only smoke
against the already-running mapping container: it transitioned from
`WAITING_FOR_MAP_TF` to `OK_DELAYED`, displayed 346--355 points out of 1800,
and reported roughly 80--110 ms delay with the queue returning to zero. Eleven
early samples were dropped while this separate collector waited for startup
timestamp coverage. This validates the monitor data path only; the bounded
smoke did not restart either runtime container and does not establish map
accuracy.

### Candidate map downloaded for localization follow-up (2026-09-11)

The current Jetson `/map` was saved with `nav2_map_server map_saver_cli` and
downloaded to
`real_robot/cuvslam/evidence/maps/20260911_162933_cuvslam_a2m12_emitter_off/`.
It contains a `200 x 296` occupancy grid at `0.05 m/cell`, with YAML origin
`[-6.29, -9.62, 0]`. The corresponding Jetson source is under
`/home/tseng/isaac_ros_data/real_robot/cuvslam/maps/20260911_162933_cuvslam_a2m12_emitter_off/`.
The local and remote file hashes match and are recorded in the map directory's
README.

This is a candidate static map for the next, separately reviewed localization
stage. The download did not start localization, Nav2, `/cmd_vel`, or robot
motion. Because the live capture reported only approximately `1%` scan-map
endpoint alignment within `0.30 m`, the artifact is preserved for controlled
testing and is not yet a map-accuracy or navigation acceptance result.

### Experimental localization GUI integration (2026-09-24)

The current candidate map
`real_robot/cuvslam/evidence/maps/20260924_092630_current_cuvslam_a2m12/`
is `199 x 308` at `0.05 m/cell`, with YAML origin `[-3.89, -5.34, 0]`.
The existing Mac `8765` GUI now has a visible `定位測試區` with a
`開始定位` button, candidate-map metadata, AMCL pose, scan/TF status,
`0.15/0.30 m` overlap ratios, and a read-only occupancy-map canvas. Scan
endpoints are colored by their distance to occupied map cells; missing map,
AMCL, scan, or TF data remains `WAIT`.

The GUI starts the isolated `experimental_candidate_localization` container
only after guarding against active R1.5/R2/RPLIDAR runtimes. The experimental
launch remains the sole `map -> odom` owner in that container and does not
publish `/cmd_vel`. The collector now includes `/amcl_pose` in its telemetry.
This is a monitor/integration change only; no localization runtime or robot
motion was started during this update, and the visible overlap is not an
accuracy acceptance result.

Jetson synchronization completed with a dated backup at
`/home/tseng/isaac_ros_ws/backups/localization_gui_20260924_103011/`.
The collector and control/owner-guard scripts were hash-verified after copy;
remote shell/Python syntax checks passed. The local GUI smoke request could
not complete because the macOS execution environment rejected a loopback
client connection after binding the temporary port; static DOM/API checks and
the project contract tests passed.

### Diagnostic LiDAR odometry + EKF shadow implementation (2026-09-03)

Following the `0.696 m` turn-translation observation, the local repository now
contains the first implementation of the selected Scheme A diagnostic branch:

```text
cuVSLAM odom ----------------┐
                             ├-> robot_localization EKF shadow
RPLIDAR /scan -> rf2o odom --┘
```

The branch is opt-in through the GUI estimator selector or the mapping runner
argument `ekf_shadow`. It publishes `/lidar/odom` and
`/odometry/filtered_shadow`, records both in light/full bags, and sends their
relative-pose comparison to the GUI and R2 session/trace records. The
comparison uses each source's first accepted pose as its local origin and is
diagnostic only; it does not imply that the sources are time-synchronized or
accuracy-qualified.

The ownership contract is deliberately unchanged: cuVSLAM remains the sole
`odom -> base_link` TF publisher, slam_toolbox remains the sole `map -> odom`
publisher, and the shadow LiDAR odometry and EKF both set `publish_tf=false`.
The integrated launch defaults both switches to false, so the existing
cuVSLAM + A2M12 + slam_toolbox behavior is unchanged unless the new estimator
is explicitly selected. The cuVSLAM launch also exposes
`publish_odom_to_base_tf` for a future, separately reviewed promotion rather
than changing its current default.

`rf2o_laser_odometry` and `robot_localization` are external Jetson ROS 2
dependencies. On 2026-09-10 they were added to the canonical Jetson workspace
and built successfully in the separate `isaac_ros_dev-aarch64:ekf-shadow`
image, with GeographicLib built offline from a pinned source commit because
the Jetson could not resolve external package repositories. The deployed
preflight and bounded no-sensor startup checks passed. A dated pre-deployment
backup is preserved under
`/home/tseng/isaac_ros_ws/backups/ekf_shadow_predeploy_19700101_090410`.
The Jetson clock is incorrect, so that directory name and build timestamps are
not reliable wall-clock evidence. No hardware motion, mapping-accuracy, Nav2,
`/cmd_vel`, or EKF TF-ownership acceptance has been run.

The next gate is: start the shadow branch while stationary, verify `/scan`,
`/lidar/odom`, `/odometry/filtered_shadow`, frame IDs, timestamps, and absence
of a second `odom -> base_link` TF publisher, then perform separately labelled
pure-turn and straight-motion tests. A FAIL in the shadow comparison is
evidence for review, not permission to promote EKF to the formal odom TF owner.

### EKF contribution observability (2026-09-10)

The shadow telemetry now records two deliberately separate views of EKF
influence. The GUI work record and saved session show a geometric projection
of the EKF pose between the cuVSLAM and LiDAR trajectories for XY and yaw;
this is labelled as a trajectory estimate and must not be read as a Kalman
gain percentage. The collector also subscribes to `/diagnostics` and records
the `robot_localization` fields that are actually available, including
processed, rejected, dropped, ignored, and frequency values. The absence of a
field is shown as unavailable. The standard diagnostic stream does not provide
one universal per-sensor contribution percentage.

The contribution estimate is marked REVIEW when source timestamp skew exceeds
100 ms. This observability addition does not change TF ownership, EKF
parameters, Nav2, `/cmd_vel`, or hardware acceptance status.

### Jetson EKF shadow deployment and stationary gate (2026-09-10)

The LiDAR odometry + EKF shadow branch is now deployed in the separate
`isaac_ros_dev-aarch64:ekf-shadow` image. The Jetson build, dependency
preflight, and bounded no-sensor node startup checks passed. The first
stationary start also found and corrected a stale cuVSLAM real-camera launch
copy on Jetson; the corrected launch is now synchronized to the canonical and
source-package paths.

The corrected stationary gate measured approximately `/scan` `12.0 Hz`,
`/lidar/odom` `11.3 Hz`, and `/odometry/filtered_shadow` `29.3 Hz`. Both shadow
nodes resolved `publish_tf=false`; their `/tf` publishers are defensively
remapped to `/shadow/lidar_odom_tf_unused` and `/shadow/ekf_tf_unused`. The
production `/tf` graph retained cuVSLAM and slam_toolbox ownership only. The
container was stopped through the motor-stop flow after the check, and no
process remained on `/dev/ttyUSB0`.

This advances the project to a stationary interface/ownership gate only. No
pure-turn, straight-motion, closed-loop, LiDAR accuracy, EKF correction, Nav2,
or `/cmd_vel` acceptance claim is made. The next test must be a labelled
motion run with the same estimator selection and saved evidence.

### Multi-source odom visualization and covariance guard (2026-09-10)

The local shadow observability path now keeps bounded, common-`odom` display
tracks for cuVSLAM, `/lidar/odom`, and `/odometry/filtered_shadow`. The GUI
renders all three tracks together so a smooth EKF output cannot be mistaken
for a formal odom switch. A reconnect resets only these comparison tracks;
the existing retained cuVSLAM session path and counters remain preserved.

Each odometry source now reports compact pose/twist covariance status,
including zero indices in the planar fields used by the shadow EKF. The GUI
work record, live quality assessment, saved session, and copyable summary
show covariance warnings and the policy is explicit: mark and review the
input, but do not rewrite source messages or change TF ownership. Pairwise
timestamp skew is logged as well. When the three-source skew exceeds 100 ms,
the geometric EKF contribution percentage is shown as unavailable rather
than presenting a false 100/0 result.

This change is local observability only. The Jetson files were not
resynchronized in this step, no runtime was started, and no production
`odom -> base_link`, `map -> odom`, Nav2, or `/cmd_vel` owner changed.
Local Python compilation, the static RPLIDAR/cuVSLAM/EKF contract test, and
`git diff --check` passed. The next hardware gate is to sync these two tool
changes, run stationary first, then repeat labelled pure-turn and straight
tests while preserving the trace and session files.

### Fused odom input contract correction (2026-09-10)

The first local implementation of Scheme A now separates input validation from
the diagnostic EKF. The EKF shadow consumes `/fusion/visual_odom` and
`/fusion/lidar_odom`, not the raw source topics. Pose fields and differential
pose conversion are disabled to avoid feeding the same source motion twice.
cuVSLAM contributes selected `vx`, `vy`, and `wz` twist fields; RF2O
contributes only `vx` and `wz` because the pinned RF2O source writes lateral
velocity as a fixed zero. `odom_contract_guard.py` rejects bad frame IDs,
non-monotonic timestamps, non-finite values, and zero/invalid covariance, and
publishes `/fusion/odom_contract_diagnostics` with the reason.
The live collector forwards these diagnostics to the GUI, and the copyable
process log records each source transition as accepted or isolated with its
latest reason. Raw and guarded topics are both retained in the light/full
recording topic lists so a rejected input remains auditable.

This is a local C2/C3 shadow change only. It does not invent covariance values,
does not change TF ownership, and does not make the LiDAR source acceptable
until its selected twist covariance is measured and populated. If the current
RF2O output still has zero covariance, the guard will isolate it by design.
Local Python, unit, contract, Bash, and diff checks passed; Jetson sync,
runtime startup, and motion tests remain pending.

### Scheme A input contract Jetson sync (2026-09-10)

The Phase 1 input-contract files were synchronized to the Jetson canonical
workspace after preserving the previous remote files at
`/home/tseng/isaac_ros_ws/backups/odom_contract_phase1_20260910_154300`.
The guard, EKF config, collector, runner, unit/contract tests, and integrated
launch were copied to `real_robot/cuvslam/`; the launch was also synchronized
to the ROS package source copy so both launch lookup paths contain the same
content. Local and remote SHA-256 values matched for every deployed file.

Jetson offline validation passed: Python compilation, guard unit tests (4/4),
Bash syntax, and the key launch/config/recording contract checks. No Docker
container, ROS node, sensor, motor, or formal TF owner was started or changed.
The next step is a stationary `ekf_shadow` runtime gate; it must first report
whether the current RF2O twist covariance is valid. If not, the guard should
isolate `/lidar/odom` and the result must be recorded as an input-contract
failure, not treated as successful fused odometry.

### D435i emitter A/B GUI control (2026-09-10)

The macOS GUI now exposes an independent `IR projector A/B` selector. `ON` and
`OFF` are forwarded as an explicit emitter override to both the R2 and A2M12
mapping runners; the selected lighting profile still controls exposure, gain,
auto-exposure, and denoise. The requested override is preserved in the work
log, trace, and saved session so the two runs can be compared as a controlled
single-variable experiment. The Jetson runners were backed up under
`/home/tseng/isaac_ros_ws/backups/emitter_ab_gui_20260910_162925` and synced;
  the active container was not interrupted. The motion result is recorded in
  `research/cuvslam/real_d435i_cuvslam_baseline_emitter_ab_20260910.md`.

### Current-space cuVSLAM-only navigation baseline (2026-09-10)

The controlled emitter A/B work in the current indoor space produced a usable
cuVSLAM-only odometry result with emitter OFF. The primary OFF run used
`mode=imu`, `640x360x30`, `official` lighting, `estimator=cuvslam`,
`lidar_odom=false`, and `ekf_shadow=false`. It reached approximately 30 Hz
with 100% VO-valid samples and ended near its starting position after an
approximately 14.98 m route. The detailed trace is
`real_robot/cuvslam/evidence/logs/real_d435i_quality/20260910_163200_r2_trace.jsonl`.

For the current space, cuVSLAM-only odometry with emitter OFF is now the
navigation development baseline. This is an environment-scoped engineering
decision, not a universal emitter recommendation and not a claim that every
turn transient is eliminated. The trace still contains intermediate rotation
geometry warnings, so straight, turn, closed-loop, tracking-loss, and Nav2
costmap gates remain required.

LiDAR scan-matching odometry and EKF fusion are deferred as a future
enhancement. The shadow implementation, input-contract guard, configurations,
tests, and Jetson rollback copies remain preserved. They do not publish the
formal `odom -> base_link` TF and do not participate in the current navigation
baseline. Revisit them only after cuVSLAM-only navigation shows a repeatable
failure that a second odometry source can address and the RF2O input contract,
direction, covariance, timing, and Jetson resource gates are qualified.

### Fixed A2M12 mounting calibration path (2026-09-10)

The existing RPLIDAR launch already owns the static
`base_link -> rplidar_link` transform, but the GUI and mapping runner did not
previously forward the fixed installation yaw. The local tools now accept
`CUVSLAM_LIDAR_YAW` (or the mapping runner's optional seventh `start`
argument), pass it to the launch as `lidar_yaw`, and write it into the runtime
record. The default remains `0.0` radians; the physical A2M12 is not moved and
no 180-degree correction is assumed. The runner rejects non-numeric values.

This is parameter plumbing and traceability, not extrinsic calibration or an
accuracy result. Jetson was not synchronized and no runtime was started in
this change. The next hardware step is to sync the tools, run the stationary
gate, then use a labelled straight/turn test to determine whether the current
fixed mount requires a nonzero yaw. Until that evidence exists, keep
`lidar_yaw=0.0` and retain the shadow-only TF policy.

### LiDAR translation extrinsic parameter plumbing (2026-09-11)

The local runner and GUI now carry `CUVSLAM_LIDAR_X/Y/Z` in addition to
`CUVSLAM_LIDAR_YAW`. The existing positional runner interface remains
backward-compatible: translation values are appended after the emitter
override, validated as numeric values, and forwarded to the integrated launch
as `lidar_x`, `lidar_y`, and `lidar_z`. The requested values are also written
to the runtime record and work log so that a future hardware result can be
traced to the exact static TF input.

The current Mac settings are `x=0.0`, `y=-0.10`, `z=0.0`, and `yaw=2.334` rad.
The horizontal offset is the user's approximate fixed-mount measurement, with
the LiDAR to the camera's right under the project's `base_link` convention.
It remains a hardware test candidate rather than a verified extrinsic result;
the fixed A2M12 mount was not moved. This change does not claim improved map
quality and does not change TF ownership, LiDAR odometry, EKF, Nav2, or the
production odom path.

Local Python compilation, runner Bash syntax, the real D435i + RPLIDAR
contract test, and whitespace checks passed. An initial sync attempt was
blocked while the USB network was down; after SSH recovered, the runner was
synchronized on 2026-09-11. The local and remote runner hash is
`9f65508d135522667c0d7d8b672a976817279e226ecf9b05aeed89a4baa2f86c`, and the
previous remote copy is preserved at
`/home/tseng/isaac_ros_ws/real_robot/cuvslam/tools/run_real_d435i_rplidar.sh.before_lidar_y_20260911_094344`.
Remote `bash -n` passed. No container was started and no ROS or hardware run
was performed. The next gate is to start through the GUI with the explicit
`y=-0.10 m` value, then run stationary, straight, and labelled-turn checks.
Rollback is the dated remote backup above or restoring the translation values
to zero.

### Scheme A runner deployment (2026-09-10)

The approved Scheme A deployment has now synchronized the local
`run_real_d435i_rplidar.sh` to the Jetson canonical path. The previous remote
runner was preserved at
`/home/tseng/isaac_ros_ws/backups/run_real_d435i_rplidar.sh.before_yaw_20260910`.
The Mac-side ignored setting `shared/config/local.env` now requests
`CUVSLAM_LIDAR_YAW=2.334`, based on the stable `+133.7°` trajectory alignment
estimate from the 2026-09-10 trace. The launch already contained the
`lidar_yaw` static-TF input; the runner was the missing forwarding link.

Remote Bash syntax and local contract checks passed, and the local/remote
runner SHA-256 values matched. No container was started by this deployment, so
the `2.334 rad` value remains a hardware test candidate rather than an
accuracy acceptance result. Rollback is the preserved remote runner above;
the next gate is stationary startup followed by labelled straight and
pure-turn motion.

## Current handoff (2026-09-11 — read this first)

### Milestone reached: real cuVSLAM odometry accepted

The R1, R1.5, and R2 program has established a working real-camera
visual-inertial odometry module on Jetson AGX Orin with D435i. Under the tested
good-image conditions, the accepted evidence includes:

- stable stereo input at approximately 30 Hz with synchronized left/right
  timestamps;
- working D435i IMU data and cuVSLAM IMU fusion;
- 100 percent VO-valid status in the accepted closed-loop recordings;
- approximately 10 m closed-loop trials ending at 0.011--0.036 m XY residual
  and approximately 1.0--1.4 deg yaw residual;
- stable odometry telemetry with no reconnects in the accepted final run;
- backend pose-graph growth and loop-closure evidence in the mapping diagnostic
  bag, although backend mapping is not required by the production design.

This validates cuVSLAM odometry within the tested lighting, motion, and scene
envelope. It does not claim universal robustness or replace the remaining
robot-mounted, vibration, speed, and duration tests.

| Phase | Scope | Status |
|---|---|---|
| A | Jetson/D435i environment readiness | Complete |
| B | R1/R1.5 stereo, IMU, axes, and fusion qualification | Complete |
| C | R2 real cuVSLAM odometry qualification | Complete — milestone accepted |
| D | Production odometry-only launch and robot-frame integration | In progress — current-space cuVSLAM-only baseline accepted; robot-mounted/navigation gates pending |
| E | 3D LiDAR global mapping/localization | Pending |
| F | Nav2 and real-robot qualification | Pending |

### Completed

- The repository was organized for GitHub and pushed to
  `https://github.com/atanzoo/cuvslam_project` on branch `main` at commit
  `d53d08d`.
- Real D435i R1/R1.5 transport, IMU qualification, axis checks, and R2
  stereo+IMU odometry/mapping diagnostics are operational.
- The macOS GUI provides live odometry, complete retained XY path, image
  quality diagnostics, IMU display, R2 quality scoring, process log, trace
  record, and copy/download helpers.
- The validated real-camera profile is NVIDIA-style input at `640x360x30`
  with IMU fusion, denoise, auto exposure, and emitter enabled.
- Good-image straight, turn, and closed-loop trials have passed the current
  engineering gate. The GUI and handoff keep the special-scene false-
  translation case visible instead of hiding it behind the overall score.
- The 2026-08-28 lightweight bag was audited independently: tracking odometry
  closed at 0.0356 m / -1.43 deg, SLAM path closed at 0.0657 m / -1.05 deg,
  the graph reached 32 nodes and 32 edges, and the final landmarks cloud
  contained 5,539 points.
- Jetson was reconnected after the audit; D435i enumerated as Intel USB device
  `8086:0b3a` with six video nodes, and the previous 12 MB bag remained intact.

### Active architecture decision

```text
D435i stereo + IMU
        |
        v
cuVSLAM odometry-only
        |
        +--> odometry topic / odom -> base_link
        |
        v
3D LiDAR SLAM/localization --> map -> odom
        |
        +--> 3D map and 2D/voxel navigation representation
        |
        v
Nav2 costmaps, planning, recovery, and cmd_vel
```

Production ownership is locked as follows:

| Interface | Owner |
|---|---|
| Local smooth odometry | cuVSLAM, optionally fused later with wheel data |
| `odom -> base_link` | cuVSLAM adapter or a single state-estimation node |
| Global map and localization | 3D LiDAR SLAM/localization |
| `map -> odom` | 3D LiDAR localization only |
| Static and dynamic obstacle representation | 3D LiDAR/depth to Nav2 costmaps |
| Planning and velocity commands | Nav2 and the robot base controller |

No two nodes may publish the same dynamic TF edge. In particular, cuVSLAM and
3D LiDAR localization must not both publish `map -> odom`.

### D1 Max deployment feasibility

The selected architecture is compatible with the D1 Max hardware boundary.
The robot already provides an NVIDIA Orin NX 16 GB for mapping, localization,
and navigation, two 96-line LiDARs, two USB 3.0 ports, Gigabit Ethernet, payload
mounting rails, and 5/12/24/48 V expansion power. Therefore the recommended
first deployment does not add the current AGX Orin as payload:

```text
D435i --USB 3.0--> D1 Max onboard Orin NX --> cuVSLAM odometry
front/rear LiDAR topics -------------------> LiDAR SLAM/localization
Nav2 cmd_vel --> D1 Max SDK bridge --------> SDK Move + emergency stop
```

The status is **hardware feasible / software conditional / not yet tested on
the physical D1 Max**. The remaining acceptance gates are:

- identify the robot firmware, SDK, JetPack/L4T, CUDA, ROS 2, and available
  Orin NX resources before moving the existing Isaac ROS 2.1/cuVSLAM 11.4
  container;
- verify D435i USB current under load and use regulated auxiliary power or a
  powered hub if the robot's 5 V/1 A USB port is not stable enough;
- import the official D1 Max `max_description` URDF. The repository's current
  `simulation/path_planning/models/d1_edu` model is D1 Edu and must not be used as D1 Max physical truth;
- calibrate `base_link -> camera_link` and both LiDAR extrinsics, then audit
  timestamps and the complete TF tree;
- qualify walking vibration, body roll/pitch, gait speed, occlusion, tracking
  loss, thermals, and sustained CPU/GPU/RAM load;
- implement a continuously refreshed Nav2-to-SDK `Move` bridge with command
  timeout, control-ownership checks, and `SoftEmergencyStop` behavior.

Official references: [D1 Max hardware architecture](https://agibottech.github.io/Agibot_D1_Max/1.6D1_Max%E7%A1%AC%E4%BB%B6%E6%9E%B6%E6%9E%84%E5%9B%BE.html),
[expansion interfaces](https://agibottech.github.io/Agibot_D1_Max/1.5%E7%94%B5%E6%B0%94%E6%8B%93%E5%B1%95%E6%8E%A5%E5%8F%A3.html),
[LiDAR topics](https://agibottech.github.io/Agibot_D1_Max/4.2%E6%BF%80%E5%85%89%E9%9B%B7%E8%BE%BE%E6%95%B0%E6%8D%AE.html),
[Robot SDK and D1 Max URDF](https://github.com/AgibotTech/Agibot_D1_Max), and
[Isaac ROS 2.1 requirements](https://nvidia-isaac-ros.github.io/v/release-2.1/getting_started/index.html).

### D1 Max path-planning simulation: Scheme A A1 gate (2026-09-03)

The staged Scheme A profile contract is now recorded in
`research/path_planning/D1_MAX_SCHEME_A_DESIGN_2026-09-03.md`. The runnable
simulation baseline remains `d1_edu`. A separate `d1_max_proxy` identity is
available for configuration discovery. The official D1 Max `max_description`
source has now been fetched at revision
`83bdebb04ae140f751e7c5e9e2c8be44aad62473` into ignored `external/` storage,
audited, and converted to a MuJoCo-loadable MJCF. The full D1 Max execution
profile is still intentionally rejected before episode construction because
measured planner footprint, collision calibration, and completed gait execution
are not ready. A separate, explicit simulation-only kinematic proxy is now
allowed for map/PPO/MPPI interface work and is reported as proxy evidence.
This prevents the converted geometry proxy from being reported as full D1 Max
dynamic or real-robot evidence.

The profile guard is implemented in
`simulation/path_planning/tools/d1_robot_profile.py` and wired into the D1
decision-layer viewer and SB3 environment. The existing map-driven planner
remains an independent Nav2-style in-process simulation; no real ROS 2 Nav2,
cuVSLAM, LiDAR, TF,
`/cmd_vel`, Jetson, or physical D1 Max behavior was changed. The next gate is
to validate the base box proxy against approved geometry, measure the
gait-constrained planner footprint, and establish command/gait limits before
enabling non-proxy D1 Max training.

The A1 evidence is recorded in
`research/path_planning/D1_MAX_SCHEME_A_DESIGN_2026-09-03.md` and the conversion
entry point is
`simulation/path_planning/tools/convert_d1_max_urdf.py`. The generated model
loads with `nq=23, nv=22, nu=16` and passes a 100-step finite smoke; this is a
model-load gate only, not a standing, walking, contact, Nav2, or real-robot
acceptance result.

The first A2 geometry sub-gate is also complete. The read-only audit tool
`simulation/path_planning/tools/inspect_d1_max_geometry.py` records the default
MuJoCo pose in
`research/path_planning/D1_MAX_GEOMETRY_AUDIT_2026-09-03.json`: the base
collision proxy spans approximately `0.922 x 0.237 m`, while all 29 robot
collision geoms span approximately `0.922 x 0.476 m` in XY. The measured
`0.480 m` planar radius and rounded `0.49 m` candidate are diagnostic only;
they are not yet the approved planner footprint because gait envelope and
contact calibration are still missing.

The same audit now performs a deterministic joint-limit diagnostic. Sixteen
limited hinge joints are evaluated at both limits plus 256 uniform samples
(seed 0), using MuJoCo `geom_rbound` for a conservative estimate. It produces
an approximately `1.032 m` radius (`1.04 m` rounded candidate). This is a
joint-limit envelope diagnostic, not a complete reachable-set proof or a
walking footprint; it must not be used as the planner radius. A gait-derived
envelope remains the next measurement gate.

The gait dependency is now explicitly deferred through a simulation-only
kinematic proxy. `run_d1_edu_decision_layer_viewer.py` accepts
`--robot-profile d1_max_proxy --kinematic-proxy`; this loads the imported D1 Max
MJCF, adds a floor when the source model has none, holds the model's default
hinge pose, and uses the current `0.49 m` footprint candidate for planner and
clearance calculations. A D1 Max run without the explicit flag remains blocked.
A short headless episode passed with the full LiDAR, Decision Layer, MPPI,
SimulatedD1SDK, and kinematic-base path. This is a simulation interface gate
only.

The SB3 high-level decision environment is now also profile-aware. Passing
`--robot-profile d1_max_proxy --kinematic-proxy` to
`simulation/path_planning/tools/run_d1_edu_decision_layer_sb3.py` makes both
training and evaluation load the D1 Max MJCF proxy, use the calibrated
`0.49 m` candidate planner radius, and carry the profile into the episode
config. The command writes a `<policy>.metadata.json` sidecar; loading or
resuming a D1 Max model without matching metadata is rejected, so an existing
D1 Edu policy cannot silently be reused. A 16-step training plus one-trial
evaluation smoke passed on 2026-09-03 and wrote the model/metadata to `/tmp`.
This is an SB3 data-flow gate only: no D1 Max standing/walking, physical
contact qualification, Nav2 ROS runtime, Jetson, or real-robot control was
performed.

### Current limitations

The remaining real-camera risk is scene-dependent false translation during a
turn, especially when the view contains weak, changing, or difficult stereo
observability. A high VO-validity percentage and a completed yaw change do not
prove that every intermediate XY displacement is correct. This is treated as a
known special-scene limitation, not as evidence that IMU fusion is disabled.

Additional integration work remains:

- lock the D435i-to-`base_link` extrinsic and robot-centered frame convention;
- create an odometry-only production launch with cuVSLAM mapping, map TF, and
  SLAM visualization disabled;
- test robot-mounted vibration, commanded speed, longer duration, temporary
  occlusion, and tracking-loss recovery;
- select and validate the 3D LiDAR SLAM/localization provider;
- provide Nav2 with a 2D occupancy projection or voxel/obstacle layers;
- define safe behavior when either visual odometry or LiDAR localization is
  unavailable.

### Next conversation: recommended order

1. Read `docs/CUVSLAM_CURRENT_STATUS_20260825.md` and
   `docs/PROJECT_CONTENT_MAP.md`.
2. Confirm the Jetson is reachable and the D435i is enumerated before starting
   a new session.
3. Create and verify the production cuVSLAM odometry-only launch:
   `enable_imu_fusion=true`, `enable_localization_n_mapping=false`, SLAM
   visualization off, and no cuVSLAM `map -> odom` publisher.
4. Connect the cuVSLAM odometry output to the robot `base_link` frame and audit
   the complete `map -> odom -> base_link -> sensor frames` TF tree.
5. If investigating the special turn, compare the complete trace and image
   quality timeline rather than relying only on the final score or final yaw.
6. Bring up 3D LiDAR SLAM/localization as the sole global-pose owner and define
   its 2D/voxel output for Nav2.
7. Integrate Nav2 only after TF ownership, timestamps, localization reset, and
   failure behavior pass controlled tests.
8. Keep D1/path-planning simulation evidence separate from real sensor and
   localization acceptance evidence.

### Stable entry points

- Real-camera status: `docs/CUVSLAM_CURRENT_STATUS_20260825.md`
- Real-camera procedure: `docs/REAL_D435I_MAPPING_HANDOFF_20260817.md`
- Repository classification: `docs/PROJECT_CONTENT_MAP.md`
- macOS GUI: `real_robot/cuvslam/tools/real_d435i_axis_web_gui.py`
- Real R2 launcher: `real_robot/cuvslam/tools/run_real_d435i_r2.sh`
- Mapping/backend bag auditor: `real_robot/cuvslam/tools/analyze_real_cuvslam_mapping_bag.py`
- RViz viewer helper: `real_robot/cuvslam/tools/open_real_d435i_rviz_view.sh`
- GitHub policy: `docs/GITHUB_REPOSITORY_GUIDE.md`

## Historical simulation record

The later geometry investigation refined this conclusion: scene
observability fixed forward translation, but low-disparity far features still
created bad current observations and landmarks. A controlled
`640x360 @ 30 Hz` plus `2.5 m` render-range variant improved a 0.25 m
observation test from 43.8% to 87.2% within 0.25 m of SDF surfaces. Its first
1 m out-and-back run nevertheless ended with 0.1834 m cuVSLAM closure error,
6.07 deg rotation error, and only 78.4% of final landmarks within 0.25 m.
Turning remains blocked.

A later exploratory `1.0 m -> 90 deg -> 0.5 m` route tested whether more time
and viewpoint diversity would improve the backend. All 883 status samples
remained at `vo_state=1`, but the turn produced `0.4998 m` false translation
and `24.23 deg` rotation error. `LL_MAP` quality peaked at 80.4 percent during
the first straight and fell to 65.7 percent at the end. The hypothesis that a
longer route would naturally clean the map was rejected for this
configuration. See `simulation/cuvslam/evidence/reports/cuvslam_corner_exploration_20260731.md`.

The D435i audit then proved that `/ground_truth/odom` was wheel odometry, not
physical turn truth. A nominal 91-degree wheel-odometry turn was only
79.28 degrees in Gazebo native pose, while 200 Hz IMU integration measured
79.29 degrees. The route controller now stops from timestamped IMU gyro
integration; a short validation reached 89.74 degrees by native pose and
89.76 degrees by bag IMU. A native-truth A/B found that enabling cuVSLAM IMU
fusion worsened turn rotation residual from 11.28 to 12.72 degrees and false
turn translation from 0.2738 to 0.3251 m. Keep fusion disabled by default.
See `simulation/cuvslam/evidence/reports/cuvslam_d435i_imu_corner_audit_20260731.md`.

The subsequent pure-turn investigation strictly removed wheel odometry from
the bridge, recorder, controller, cuVSLAM, and evaluator. Speed A/B, VIO,
camera-at-origin, VO-only, and render-range counterfactuals isolated the
remaining error to rotational scene observability. The accepted dense-turn v3
world supplies continuous asymmetric near-field geometry across 0--90
degrees. Two independent stereo-only 20 deg/s cold starts passed:

| Trial | Native yaw | cuVSLAM yaw | Yaw error | False translation |
|---|---:|---:|---:|---:|
| dense v3 run 1 | 90.47 deg | 89.72 deg | -0.75 deg | 0.0030 m |
| dense v3 run 2 | 90.70 deg | 90.78 deg | +0.08 deg | 0.0167 m |

Both had zero tracking loss and remained below the 5-degree / 0.05 m pose
gate. Keep IMU fusion disabled. The next gate is
`1 m -> 90 deg -> 0.5 m` in dense v3, using non-wheel distance control and
native Gazebo pose only for control/evaluation truth. See
`simulation/cuvslam/evidence/reports/cuvslam_in_place_turn_pose_fix_20260731.md`.

That initial mapping gate has now been run. Control reached 1.0023 m,
90.04 degrees by IMU, and 0.5007 m, with all 852 status samples at
`vo_state=1`. Sensor truth remained valid: native yaw was 90.48 degrees and
gyro integration was 90.50 degrees. The route nevertheless failed pose:
the first leg residual was 0.1010 m, turn yaw was under-estimated by
11.59 degrees, and final yaw error was -14.88 degrees. The dense v3 arc was
centered on the initial pose, while the route turns one metre farther forward;
the turn fix is therefore position-dependent.

Current observations were strong (99.3 percent within 0.25 m), and the final
4,243-landmark map scored 99.5 percent within 0.25 m, but snapshot P90
degraded to 0.1188 m and 22 catastrophic outliers reached 54.4 m from the
nearest SDF surface. Do not accept this map despite the high threshold pass
rate. Extend asymmetric near-field geometry around the actual corner at
approximately `(-0.8, -1.8)` and repeat the exact route. See
`simulation/cuvslam/evidence/reports/cuvslam_initial_dense_v3_mapping_20260731.md`.

## 1. Objective

Deliver a reliable visual-inertial odometry source for the navigation stack,
then integrate it with independently owned 3D LiDAR localization and Nav2:

1. Preserve the verified Jetson, D435i stereo, IMU, synchronization, and image
   quality contract.
2. Run cuVSLAM in odometry-only production mode and publish a smooth,
   robot-centered local pose/velocity interface.
3. Validate `odom -> base_link` geometry, timing, continuity, failure
   detection, and robot-mounted robustness.
4. Use 3D LiDAR SLAM/localization for the global map and `map -> odom`.
5. Convert or expose LiDAR geometry as the 2D occupancy/voxel/obstacle
   representation consumed by Nav2.
6. Integrate Nav2 after TF ownership, localization, obstacle, and safety
   interfaces pass controlled tests.
7. Evaluate optional wheel-odometry fusion only after the cuVSLAM-only baseline
   is preserved and repeatable.

The earlier goal of making cuVSLAM's own landmark map the production global
localization map is retired. It may remain an experiment or comparison, but it
is no longer on the critical path to navigation.

## 2. Non-goals for the initial project

- Do not modify the existing production LiDAR ICP implementation.
- Do not modify the existing AMCL tuning or active 2D occupancy map.
- Do not replace the existing `/Applications/slam_v2` navigation baseline.
- Do not treat a cuVSLAM 3D landmark map as a Nav2 2D occupancy map.
- Do not require cuVSLAM map save/load or visual relocalization before Nav2
  integration.
- Do not allow cuVSLAM to publish `map -> odom` when 3D LiDAR localization owns
  that edge.
- Do not claim real-robot accuracy from Gazebo truth or paper benchmarks alone.
- Do not connect cuVSLAM to the production Nav2 runtime before an approved
  integration design exists.

## 3. Planned system boundary

```text
RealSense D435i stereo + IMU
        |
        v
Jetson AGX Orin + cuVSLAM odometry-only
        |
        +--> local odometry and tracking health
        |
        v
odom -> base_link

3D LiDAR SLAM/localization
        |
        +--> global map and map -> odom
        +--> occupancy/voxel/obstacle representation
        v
Nav2 planning, costmaps, recovery, and cmd_vel
```

The cuVSLAM workstream owns camera/IMU input qualification, local odometry,
tracking health, data capture, evaluation, and its robot-frame adapter. The 3D
LiDAR workstream owns the global map and localization. Nav2 consumes both
through explicit TF, odometry, and costmap interfaces.

## 4. Spatial representation

The project must distinguish these representations:

| Representation | Meaning | Initial owner |
|---|---|---|
| Camera/IMU frames | Sensor geometry and measurements | Calibration records |
| Local visual trajectory | Smooth time-ordered pose estimate in `odom` | cuVSLAM runtime |
| Visual diagnostics | Features, landmarks, status, and optional pose graph | cuVSLAM test tools only |
| Global localization map | Persistent world reference | 3D LiDAR SLAM/localization |
| 3D obstacle geometry | LiDAR point cloud or voxel representation | 3D LiDAR pipeline |
| 2D navigation map | Occupancy/free-space representation for Nav2 | LiDAR projection/map server |

The integration must not treat a cuVSLAM landmark cloud as occupancy,
free-space, or collision geometry.

## 5. Phases and exit criteria

### Phase A: Environment readiness

Confirm the exact Orin board or carrier, JetPack, CUDA, compiler, ROS 2
compatibility, D435i firmware, image topics, IMU topics, timestamps, and
calibration availability.

Exit criteria:

- versions recorded;
- camera and IMU data recorded with timestamps;
- sensor frames and extrinsics documented;
- thermal, power, storage, and GPU monitoring procedure defined.

Status: **complete** for the current Jetson/D435i bench configuration.

### Phase B: IMU and sensor qualification

Qualify stereo timing, CameraInfo, IMU data, IMU axes, extrinsics, and the
stereo+IMU fusion path before motion acceptance.

Exit criteria:

- left/right input is synchronized and stable;
- IMU rates, direction, and static behavior are recorded;
- the fusion configuration is explicit and repeatable;
- image quality and sensor-state diagnostics are observable.

Status: **complete** as R1/R1.5.

### Phase C: Real cuVSLAM odometry qualification

Run straight, turn, special-scene, and closed-loop routes while preserving the
full trajectory and image-quality timeline.

Exit criteria:

- accepted runs maintain stable input and odometry rates;
- straight/turn/closed-loop behavior is repeatable in the tested envelope;
- known low-light and scene-dependent limitations are documented;
- telemetry reconnect and recording behavior are verified.

Status: **complete** as the R2 bench milestone. Robot-mounted robustness remains
in Phase D.

### Phase D: Production odometry and robot-frame integration

Run cuVSLAM without its global mapping/localization backend, connect the output
to the robot-centered frame tree, and validate it under real chassis motion.

Exit criteria:

- `enable_localization_n_mapping=false` and SLAM visualization are verified off;
- one and only one node owns `odom -> base_link`;
- D435i-to-`base_link` extrinsic and timestamps are verified;
- chassis vibration, speed, occlusion, duration, and restart tests pass;
- tracking loss produces a declared invalid/degraded state.

Status: **next active phase**.

### Phase E: 3D LiDAR global mapping/localization

Select the 3D LiDAR SLAM/localization implementation and make it the sole owner
of the global map and `map -> odom`.

Exit criteria:

- saved map and localization restart are repeatable;
- `map -> odom` is globally consistent and does not conflict with cuVSLAM;
- localization loss and recovery are observable;
- 2D occupancy or voxel/obstacle outputs are defined for Nav2.

Status: **pending**.

### Phase F: Nav2 integration and real-robot qualification

Connect the accepted local odometry, global localization, obstacle
representation, and base controller to Nav2 under human-approved deployment
and emergency-stop procedures.

Exit criteria:

- the full `map -> odom -> base_link -> sensor frames` tree is valid;
- real sensor and timing behavior is measured;
- odometry or localization failure stops the robot safely;
- navigation, obstacle, and recovery tests pass;
- logs and rollback procedure are complete.

Status: **pending**.

## 6. Current decisions

- cuVSLAM is accepted as the local visual-inertial odometry provider for the
  tested envelope.
- cuVSLAM visual map persistence and relocalization are not production gates.
- 3D LiDAR SLAM/localization owns the global map and `map -> odom`.
- Nav2 receives occupancy/voxel/obstacle information from the LiDAR/depth
  pipeline, not from cuVSLAM landmarks.
- The cuVSLAM production launch disables localization/mapping, map TF output,
  and SLAM visualization while retaining stereo, IMU fusion, odometry, status,
  and diagnostics required for safety.
- A single TF owner is required for every dynamic edge.
- The existing LiDAR/Nav2 implementation remains a protected baseline until an
  approved integration design and rollback path exist.
- Wheel odometry fusion is optional and must be evaluated against the preserved
  cuVSLAM-only baseline.

## 7. Open decisions

The next implementation must decide and record:

- whether cuVSLAM publishes robot-centered odometry directly or feeds a single
  `robot_localization` EKF;
- the measured D435i-to-`base_link` extrinsic and covariance policy;
- the selected 3D LiDAR SLAM/localization implementation and map format;
- whether Nav2 uses a projected 2D static map, voxel layer, obstacle layer, or
  a controlled combination;
- the mechanism that marks cuVSLAM odometry invalid after tracking loss;
- the behavior when local odometry is healthy but global LiDAR localization is
  unavailable, and vice versa;
- whether and how wheel encoders are fused without creating duplicate TF or
  feedback loops;
- the simulation-to-real test order, emergency stop, and rollback boundary.

## 8. Required evidence bundle

Every experiment must preserve:

- experiment ID and date;
- hardware serials and software versions;
- calibration identifiers;
- configuration and parameter snapshot;
- input recording or source reference;
- output odometry trajectory, TF ownership, and status identifiers;
- global map/localization identifier when the 3D LiDAR phase begins;
- tracking and resource logs;
- metrics and plots;
- operator notes and known anomalies;
- conclusion limited to what the evidence supports.

## 9. Historical simulation entry point

The environment-design gate is complete for the isolated Jetson container.
Start with the procedure in `docs/SIMULATION_HANDOFF.md`.

The first simulation target is sensor replay or declared sensor simulation:
D435i stereo/IMU for cuVSLAM, LiDAR `/scan` for Nav2 obstacle handling, and a
dedicated dynamic-obstacle decision layer. Use ROS domain `43` for simulation
so it cannot accidentally exchange traffic with the verified Isaac ROS domain
`42` or the old Foxy systems.

Do not connect motors, serial control, or production Nav2 during this phase.

## 10. Verified State

### Current real D435i odometry state (2026-08-28)

- Target hardware: Jetson AGX Orin + Intel RealSense D435i.
- Production candidate input: rectified stereo `640x360x30`, synchronized
  left/right input, denoise, automatic exposure, emitter enabled, and D435i IMU
  fusion enabled.
- Accepted output: `/visual_slam/tracking/odometry` at approximately 30 Hz with
  status and image-quality telemetry retained by the macOS GUI.
- Accepted closed-loop evidence includes approximately 10 m routes with
  0.011--0.036 m final XY residual and approximately 1.0--1.4 deg final yaw
  residual under the tested good-image conditions.
- The 2026-08-28 backend diagnostic bag contains 2,382 odometry samples,
  5,539 final landmarks, 32 graph nodes, 32 graph edges, and one non-empty
  loop-closure cloud. These are diagnostic mapping results, not a required
  production localization database.
- Known limitation: false XY translation can occur during turns in specific
  weak or difficult stereo scenes; low-light/exposure and motion robustness
  remain operating-envelope concerns.
- Current production transition: disable cuVSLAM localization/mapping and map
  TF publication, then validate robot-centered `odom -> base_link` before
  integrating 3D LiDAR localization and Nav2.

### Historical simulation verified state

#### Runtime and frame contract

- Jetson: AGX Orin, JetPack 5.1.3 / Jetson Linux R35.5.0.
- Host ROS: Foxy; isolated Isaac ROS container: Humble.
- Simulation ROS domain: `43`.
- Estimator: Isaac ROS Visual SLAM 2.1.0, cuVSLAM 11.4.
- Historical simulation mode: rectified stereo,
  `enable_imu_fusion=false`. This does not override the accepted real-camera
  stereo+IMU profile above.
- Output ownership: cuVSLAM publishes `odom -> base_link`.
- The release-2.1 physical camera-frame contract, right CameraInfo baseline,
  and static TF tree have been audited. Do not change them without new direct
  evidence and a C2 design review.
- NVIDIA debug dump is opt-in and disabled by default.

#### NVIDIA control

The SHA-256-verified NVIDIA release-2.1 `small_pol_test` bag produced:

- 725 left images;
- 725 odometry samples;
- 725 status samples;
- `vo_state=1` for all samples;
- output frame `odom -> base_link`.

The official bag has no ground truth, so this proves runtime/data-path
functionality, not absolute accuracy.

#### Current Gazebo failure

The level-chassis `cuvslam_mapping_simple.sdf` 1 m test produced:

| Metric | Native 6DoF truth | cuVSLAM |
|---|---:|---:|
| Translation | 1.0176 m | 0.1914 m |
| Rotation | 0.00 deg | 5.05 deg |
| Translation error | - | 81.2% low |
| Tracking | - | `vo_state=1` throughout |

NVIDIA's debug dump contained 4,186 synchronized stereo pairs. During native
model motion, frames 2750 through 3250 were 501 consecutive byte-identical
left images and 501 consecutive byte-identical right images while timestamps
continued to advance.

This was the 2026-07-30 leading hypothesis. At that time, the evidence did not
distinguish:

1. Gazebo Transport camera payload freezing;
2. the Gazebo-to-ROS bridge repeating stale pixels; or
3. the NVIDIA debug dump repeating stale pixels.

#### 2026-07-31 boundary rerun

A controlled isolated Gazebo/bridge run on Jetson endpoint `192.168.55.1`
used partition `cuvslam_boundary_20260731`. Native truth moved from
`x=0.5148 m` to `x=1.0308 m`; all three pixel boundaries changed throughout
the motion interval `265.15–269.55 s`. Long repeated runs began at about
`269.545 s`, after native truth had stopped, in Gazebo Transport, both ROS
image topics, and the NVIDIA debug dump.

This run therefore does not identify an in-motion stale-pixel owner. Evidence
is in `simulation/cuvslam/evidence/reports/image_freeze_boundary_20260731.md` and the Jetson bundle under
`/home/tseng/isaac_ros_ws/data/boundary_20260731/`.

The same bag still failed the trajectory gate: native truth moved `0.516 m`,
while relative cuVSLAM tracking odometry moved approximately `0.172 m`, with
`vo_state=1` for all 332 samples. This boundary result motivated the reliable
single-owner baseline below, which supersedes the stale-pixel hypothesis.

#### 2026-07-31 1 m observability fix

A reliable single-owner cold-start test reproduced the sparse-world failure:

- native truth: `1.0116 m`;
- cuVSLAM: `0.0749 m`;
- rotation: `1.56 deg`;
- 492 / 492 samples: `vo_state=1`.

During motion, all 258 bridged left images and all 258 bridged right images
were unique. NVIDIA debug left/right streams also contained no repeated frame
during motion. Stereo timestamps matched exactly. This excludes stale pixels
as the cause of the accepted baseline failure.

The independent `cuvslam_mapping_observable.sdf` variant adds visual-only,
asymmetric near-field features while preserving all sensor, calibration,
timing, frame, and estimator contracts. Its 1 m A/B produced:

- native truth: `1.0128 m`;
- cuVSLAM: `1.0232 m`;
- translation-norm error: approximately `+1.0%`;
- 3D residual: `0.0554 m`;
- rotation error: `0.98 deg`;
- 486 / 486 samples: `vo_state=1`.

This passes the 0.95–1.05 m translation and 15 deg orientation gates. The
simulation root cause is insufficient near-field stereo disparity and
parallax in the sparse far-field world. The original world remains the
negative control; use the observable world for the next repeatability and
mapping gates. See `simulation/cuvslam/evidence/reports/cuvslam_1m_observability_fix_20260731.md`.

Do not apply an odometry scale multiplier, force planar mode, TF adjustment,
CameraInfo retuning, or IMU fusion to hide this upstream defect.

#### 2026-07-31 repeatability result and next experiment

The observable world now covers every leg of the square route with 30
visual-only near-field features. Two additional cold-start 1 m trials passed:

- repeat 1: truth `1.0176 m`, cuVSLAM `1.0189 m`, rotation `0.78 deg`;
- repeat 2: truth `1.0152 m`, cuVSLAM `0.9708 m`, rotation `1.32 deg`.

All in-motion ROS and NVIDIA debug images were unique, stereo timestamps
matched exactly, and every status sample remained `vo_state=1`. See
`simulation/cuvslam/evidence/reports/cuvslam_observable_world_repeatability_20260731.md`.

The first 1 m straight mapping run completed with truth `1.0140 m`, cuVSLAM
`0.9548 m`, fixed-scale translation ATE P95 `0.0746 m`, rotation ATE P95
`1.31 deg`, and 581 / 581 samples at `vo_state=1`. The trajectory gate passed.

The final sparse cloud contained 379 landmarks, but only 50.1 percent were
within 0.25 m of any static SDF surface. Surface-distance P90 was `1.7618 m`,
and the maximum was `5.0750 m`. The landmark geometry gate therefore failed.
See `simulation/cuvslam/evidence/reports/cuvslam_first_observable_straight_mapping_20260731.md`.

The cold-start repeat produced truth `1.0104 m`, cuVSLAM `0.9321 m`,
translation ATE P95 `0.1081 m`, and 579 / 579 samples at `vo_state=1`.
Landmark geometry failed again: 47.9 percent were within 0.25 m, P90 was
`1.8473 m`, and maximum error was `6.7848 m`.

Across the two final maps, approximately 45 percent of landmarks matched
within 1 mm and 66 percent within 5 cm. The failure is predominantly
systematic. A temporary visualization-only export patch showed every
`LL_MAP` landmark weight was exactly `1.0`, including severe outliers. The
patch was reversed and the standard NVIDIA PointCloud schema was rebuilt.

The next minimum experiment is an observation-versus-map diagnostic using
`/visual_slam/vis/observations_cloud` plus stereo disparity. Do not proceed to
a 90-degree turn. See
`simulation/cuvslam/evidence/reports/cuvslam_landmark_repeatability_and_weight_20260731.md`.

Keep the sparse world as a negative control. The straight 1 m result does not
yet establish turn accuracy, loop closure, map persistence, or relocalization.

#### Power and shutdown

The user-provided power bank passed:

- an eight-worker CPU load;
- approximately one minute of headless Gazebo + bridge + cuVSLAM;
- maximum CPU temperature 45.281 C;
- maximum GPU temperature 38.843 C;
- no observed undervoltage, overcurrent, brownout, or thermal throttling.

This is a short bench-load result, not a battery-runtime qualification.
INA3221 voltage/current telemetry still requires authorized root access.

At handoff completion, all simulation workloads were stopped and the Jetson was
shut down normally. A follow-up SSH probe timed out, confirming that the Jetson
was offline.

## 11. Primary Evidence

- `real_robot/cuvslam/evidence/logs/real_d435i_quality/20260828_113407_r2_trace.jsonl`
- `real_robot/cuvslam/evidence/logs/real_d435i_quality/20260828_113607_r2_session.json`
- `real_robot/cuvslam/tools/analyze_real_cuvslam_mapping_bag.py`
- Jetson lightweight bag:
  `/home/tseng/isaac_ros_data/real_robot/cuvslam/experiments/real_d435i_r2/gui_20260828_113440_imu_mapping_scene_unknown_other_light/rosbag`
- `simulation/cuvslam/evidence/reports/nvidia_reference_and_gazebo_1m_debug_20260730.md`
- `simulation/cuvslam/evidence/reports/cuvslam_1m_observability_fix_20260731.md`
- `simulation/cuvslam/evidence/reports/cuvslam_observable_world_repeatability_20260731.md`
- `simulation/cuvslam/evidence/reports/cuvslam_first_observable_straight_mapping_20260731.md`
- `simulation/cuvslam/evidence/reports/cuvslam_landmark_repeatability_and_weight_20260731.md`
- `simulation/cuvslam/evidence/reports/jetson_powerbank_stress_test_20260730.md`
- `simulation/cuvslam/evidence/reports/cuvslam_level_chassis_straight_trials_20260730.md`
- `simulation/cuvslam/evidence/reports/cuvslam_stage4_official_input_contract_20260730.md`
- `docs/CUVSLAM_NVIDIA_REFERENCE_DEBUG_DESIGN.md`
- `docs/sensor_and_frame_contract.md`
- `docs/NEXT_THREAD_MAPPING_TEST_PROMPT.md`

## 12. Offline D1 Max proxy candidate-map scale-5 experiment (2026-09-24)

方案 A connects the preserved candidate map to the existing local D1 Max
kinematic-proxy decision environment. The map geometry is enlarged by `5x`
while the D1 proxy and pedestrian dimensions remain unchanged. The source PGM
is read-only; the 199 x 308 raster is replicated to 995 x 1540 cells while
keeping `0.05 m/cell`. The resulting simulation bounding box is 49.75 x 77.00
meters. Occupied pixels become MuJoCo wall geometry; unknown cells remain
blocked to the route planner and are not invented as observed walls. The
planner uses the current diagnostic `0.49 m` proxy radius plus `0.12 m` margin
(13 cells of inflation); this is not a gait-derived footprint.

The experiment uses
`real_robot/cuvslam/evidence/maps/20260924_092630_current_cuvslam_a2m12/map.yaml`
and a deterministic pedestrian-crossing scenario. Crossing anchors are
selected only from locations checked against the inflated known-free mask,
preventing invalid narrow-corridor samples from aborting resets. The 12-seed
reset smoke and one-trial evaluation-wiring smoke passed. A separate 24-second
fixed-seed baseline rollout reached the episode goal in 12.608 seconds, with
no collision, MuJoCo contact, or map-boundary violation; measured minimum
clearance was 0.796 m. This is one simulation-only route result, not a
statistical performance claim.

Candidate-map CLI/evaluation support and checkpoint identity checks now bind a
policy to the source YAML/PGM hashes and scene scale. Omitting the map keeps the
existing environment path; mismatched map identity is rejected. Evidence and
reproduction scope are in
`simulation/path_planning/evidence/20260924_d1_max_proxy_candidate_map_scale_5_smoke.md`.

Map, planner, D1 SB3 environment, decision-benchmark, and scenario suites
passed. The adjacent `test_d1_edu_decision_layer.py` legacy crossing test still
fails its final-mode assertion (`min_clearance=-0.056 m`); that test exercises
the separate legacy decision-layer scenario, not the new map-reset smoke, and
was not changed here. No PPO learning run, Jetson process, hardware, gait, or
deployment was started. The map feature remains opt-in and can be rolled back
by omitting `--candidate-map-yaml`; source map files are unchanged.

## 13. Offline D1 Max proxy map-3 dual-target training preflight (2026-09-24)

The local opt-in map environment was extended for a proposed 10 m PPO run at
3x map geometry, with one fixed object and one balanced/mirrored pedestrian
from five motion families. The D1 Max geometry remains a kinematic proxy, the
candidate YAML/PGM remain read-only, and existing policies are not compatible
initializations. The focused SB3 smoke test and 100 deterministic scene-seed
preflight passed; one candidate route required a deterministic retry.

An initial fresh 300k PPO attempt was stopped at 6,400 steps after measuring
about 20.6 steps/s (311 seconds elapsed), implying roughly 4 h 03 min of PPO
training plus evaluation. It stopped before the first 25k checkpoint and
produced only a small TensorBoard event log, no policy or resumable checkpoint.
The log is retained under
`output/d1_max_proxy_map3_dual_target_balanced_300k_20260924/`. No PPO result or
hardware claim is available. Do not restart until the user chooses a new
training budget; detailed settings and measurements are recorded in
`simulation/path_planning/evidence/20260924_d1_max_proxy_map3_dual_target_ppo_interrupted.md`
and `research/path_planning/D1_MAX_PROXY_MAP3_DUAL_TARGET_PPO_PLAN_20260924.md`.

## 14. Offline D1 Max proxy map-3 dual-target 25k pilot (2026-09-24)

The user selected a bounded 25k pilot. Training completed locally at 25,088
steps (PPO's 256-step rollout boundary), with checkpoints every 5k and a
training-outcome rate series logged to TensorBoard and the summary. The
scale-3 candidate map and 10 m dual-target distribution were read-only; no
Jetson, ROS runtime, gait, hardware, or deployment was involved.

Rolling mean reward improved from a low near -40.4 early in training to -27.0
at the final step, but all five 5k outcome windows had zero successes. On 10
matched fixed seeds, CRUISE and PPO both had 0% success. PPO collision fell
from 40% to 10%, while timeout rose from 10% to 40%; both policies violated
the inflated map in the same 5/10 scenario seeds. Treat this as a preliminary
failure-mode and trade-off result, not a successful navigation policy.

Do not increase the training budget yet. First diagnose why those five routes
produce the same map violation for both policies and inspect the high WAIT and
timeout rates. Full settings, 5k training trends, per-trial metrics, artifacts,
and verification are recorded in
`simulation/path_planning/evidence/20260924_d1_max_proxy_map3_dual_target_ppo_25k.md`.

## 15. Offline D1 Max proxy map-3 feasibility follow-up (2026-10-01)

The user selected Option A: repair local route following and route-center
clearance before more PPO. Luna made a scoped simulation-only change: an extra
`0.25 m` route-selection buffer and bounded CRUISE path-heading feedback. The
runtime inflated map mask remains `0.61 m` (13 cells); the source map, old
checkpoint, and Jetson/robot runtime are untouched. The focused SB3 regression
passed under the project `.venv`, but the behavioral gate did not: ten
actor-disabled 10 m fixed-seed runs at 50 s produced 0 success, 0 collision,
10 timeout, and 0 map violation. Final goal distance was `0.721–3.240 m`
(median `2.039 m`). One matched scene reached the goal at `53.472 s` when
observed under a 70 s cap; that does not validate a longer cap for the other
nine scenes or the two-actor distribution. Fixed-object and pedestrian tests
were not run because the path-only gate failed. Do not start a fresh PPO pilot
or resume the 25k checkpoint until this baseline gate and model-identity
boundary are resolved. Diagnostics, limitations, rollback scope, and next
steps are in
`simulation/path_planning/evidence/20261001_d1_max_proxy_map3_25k_diagnostic_review.md`.
