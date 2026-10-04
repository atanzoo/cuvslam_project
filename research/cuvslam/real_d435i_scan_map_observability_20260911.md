# Real D435i + A2M12 scan-map observability correction

Date: 2026-09-11 (Asia/Taipei)

## Objective

Use the most useful parts of the historical `/Applications/slam_v2` project to
separate three problems that were previously mixed together in the live GUI:

1. historical path visualization being reprojected with a new `map -> odom`;
2. the current scan not matching the current occupancy map;
3. a possible fixed LiDAR orientation or timestamp/TF problem.

This is an observability correction. It does not promote LiDAR odometry or EKF
shadow to the production odom owner.

## Scope and ownership

- Hardware context: Jetson + Intel RealSense D435i + RPLIDAR A2M12.
- Current estimator owner remains cuVSLAM for `odom -> base_link`.
- Current mapping owner remains slam_toolbox for `map -> odom`.
- The GUI and collector remain monitoring components only.
- The GUI remains on the Mac; the runtime collector and contract-test source
  were synchronized to Jetson under
  `/home/tseng/isaac_ros_ws/real_robot/cuvslam/tools/`.

## Changes

### 1. Timestamped map-frame path

The collector previously generated `map_path` by applying the latest
`map -> odom` transform to every historical cuVSLAM point. When slam_toolbox
changed its correction, the entire old path could move together and appear to
leave the occupancy geometry.

The collector now records each odometry timestamp and projects that sample with
the transform valid at that timestamp. The resulting map path is retained as a
historical visualization; it is not an accuracy or localization acceptance
metric.

### 2. Dynamic scan-map alignment diagnostic

The collector now reports, for the latest scan:

- fraction of valid endpoints within 0.15 m of an occupied map cell;
- fraction within 0.30 m;
- best match among normal, mirrored, and 0/±90/180-degree orientation
  candidates;
- the occupancy-grid revision used for the calculation.

The distance field is rebuilt when a new map revision arrives. This corrects a
limitation in the historical monitor, which built the field only once and would
otherwise compare later scans against a stale map.

The diagnostic is intentionally descriptive. A low ratio during an incomplete
live map is not by itself proof of a bad extrinsic, and a high ratio in a map
created by the same moving scan stream is not a formal accuracy pass.

### 3. LaserScan and TF timing telemetry

The collector now records `header.stamp`, `scan_time`, `time_increment`, the
estimated per-beam acquisition span, and the estimated scan end timestamp. It
also records the timestamp requested from TF, the returned TF timestamp when
available, signed timestamp deltas, and the latest-TF fallback when the
timestamped lookup returns `TF_WAIT`. The GUI writes a compact version to the
process log and stores a point-free `scan_telemetry` record in the JSONL trace.

These values diagnose timestamp coverage; they do not deskew the scan or
change the scan consumed by slam_toolbox.

### 4. Timestamp/TF release gate (B+A implementation)

The integrated launch now remaps the A2M12 driver output to `/scan_raw` and
runs `scan_tf_gate` before the mapper. The gate checks the original scan
timestamp against the `odom -> rplidar_link` TF, waits up to two seconds, and
publishes the unchanged message on `/scan` only after that timestamped TF is
available. It does not rewrite timestamps, publish TF, or change odometry
ownership. Scans that exceed the wait or overflow the bounded queue are
dropped with an explicit runtime warning.

The mapping recorder preserves both `/scan_raw` and gated `/scan` so a later
bag review can distinguish source timing from mapper input. The GUI also logs
the current scan age and estimated scan-end age. This is a synchronization
mitigation, not a LiDAR deskew implementation; the 78--85 ms beam-acquisition
distortion observed in the 2026-09-11 run remains a later C-stage item.

### 5. Delayed exact-TF display correction

The live collector previously retained only the newest `/scan`. It then asked
for the complete `map -> rplidar_link` transform at that newest scan timestamp.
Because slam_toolbox's `map -> odom` correction can arrive roughly one second
later, the lookup could remain in `TF_WAIT` indefinitely even while the gate
and `/scan` publisher were healthy. The GUI consequently showed zero scan
points shortly after startup.

The collector now uses bounded monitor-side queues for odom and scan samples.
It waits for the complete transform at each sample's own timestamp, displays
the newest scan that resolves, retains the last valid display during a gap, and
reports `OK_DELAYED`, `display_delay_ms`, queue depth, and queue drops. Old
samples are dropped only after a bounded three-second monitor wait. The map
path uses the same delayed exact-timestamp projection. No `/scan` content,
slam_toolbox setting, TF publisher, odom source, or estimator ownership is
changed, and no latest `map -> odom` transform is mixed with an older scan.

This is a GUI observability correction, not a mapping-quality correction. The
display is intentionally delayed when the full map transform is delayed; the
delay must remain visible in the work record and trace.

### 6. Process-record clearing

The GUI work-record panel now has a clear action. It clears only `STATE.logs` in
memory. It does not remove or rewrite JSONL traces, session files, mapping
summaries, motion-quality files, downloaded bags, or other evidence.

### 7. Mapping artifact downloaded for the next localization stage

After the mapping run, the current `/map` was saved from Jetson with
`nav2_map_server map_saver_cli` and downloaded to:

`real_robot/cuvslam/evidence/maps/20260911_162933_cuvslam_a2m12_emitter_off/`

The artifact contains `map.pgm` and `map.yaml`. It is a `200 x 296` grid at
`0.05 m/cell`, with YAML origin `[-6.29, -9.62, 0]`. The remote source is
`/home/tseng/isaac_ros_data/real_robot/cuvslam/maps/20260911_162933_cuvslam_a2m12_emitter_off/`.
The local and remote SHA-256 values are recorded in the artifact README.

This is a preserved candidate map for a separate localization conversation.
The save/download operation did not start localization, Nav2, `/cmd_vel`, or
robot motion. The capture's live scan-map alignment was approximately `1%`
within `0.30 m`, so map download integrity is accepted while map accuracy and
localization readiness remain open.

## Verification

Executed locally:

```text
python3 -m py_compile \
  real_robot/cuvslam/tools/scan_tf_gate.py \
  real_robot/cuvslam/tools/real_d435i_axis_web_gui.py \
  real_robot/cuvslam/tools/collect_real_d435i_odom_live.py \
  real_robot/cuvslam/tools/test_real_d435i_rplidar_contract.py

bash -n real_robot/cuvslam/tools/run_real_d435i_rplidar.sh

python3 real_robot/cuvslam/tools/test_real_d435i_rplidar_contract.py
REAL_D435I_RPLIDAR_CONTRACT_PASS
```

The clear endpoint was also exercised through the imported GUI state object;
the in-memory log became empty and no evidence path was touched.

The scan timing parser, compact trace record, and process-log entry were also
exercised with a synthetic `TF_WAIT` telemetry sample.

The delayed exact-TF collector path was then syntax-checked and covered by the
contract test. It was synchronized to Jetson at
`/home/tseng/isaac_ros_ws/real_robot/cuvslam/tools/collect_real_d435i_odom_live.py`
after creating the recoverable backup
`/home/tseng/isaac_ros_ws/backups/collector_delayed_scan_display_20260911_160726/`.
The local and remote collector SHA-256 is
`750a3d46c9d9bcffdc4588b7aa7f2809e36cebe0a74ce1db1116bf0fafe0c5e0`, and
remote Python compilation passed. The existing mapping and RViz containers
were not restarted; the new collector takes effect when the GUI telemetry
connection is recreated.

A Jetson stationary smoke test is still required to confirm that the displayed
`/scan` remains populated after the initial map-TF lag.

Before that full stationary test, a bounded 12-second read-only collector
smoke was run against the already-running mapping container. It initially
reported `WAITING_FOR_MAP_TF`; after the timestamped TF became available it
reported `OK_DELAYED` with 346--355 displayed points out of 1800 and roughly
80--110 ms display delay. The queue returned to zero. Eleven early samples
were dropped while this new collector waited for startup timestamp coverage;
this is monitor startup behavior, not a mapping-accuracy result. The bounded
smoke process was terminated by its timeout and did not restart either runtime
container.

The new gate, collector, runner, and launch were synchronized to Jetson after
creating `/home/tseng/isaac_ros_ws/backups/scan_tf_gate_20260911_142048/`.
Remote Bash/Python syntax checks passed and the synchronized source hashes
matched the local files. The GUI remains Mac-side.

The first hardware restart exposed and fixed a launch error: the gate is a
plain Python `ExecuteProcess`, so passing ROS-only `--ros-args` caused it to
exit before subscribing to `/scan_raw`. The launch now passes only the gate's
own argparse options. The failure evidence was observed in the Jetson launch
log at 14:46:35 and is retained in the conversation record; the container was
then restarted for the corrected gate.

The corrected stationary smoke check then showed `scan_tf_gate pending=0`,
continuous publication of `/scan` (about 12 Hz), one `/scan` publisher and one
subscriber, and a valid `/map` `OccupancyGrid` message. The gate dropped 236
startup scans while cuVSLAM warmed up for about 19 seconds; after TF became
available it published continuously with typical wait times below 35 ms. This
startup loss is recorded as a remaining operational limitation, not as a map
accuracy result.

The earlier timing-only backup remains at
`/home/tseng/isaac_ros_ws/backups/scan_timing_20260911_132541/`.

Not run:

- live ROS `/map`, `/scan`, and timestamped TF test on Jetson;
- the full Jetson-host contract test, because the existing checkout is missing
  `deployment/isaac_ros/isaac_ros_visual_slam_d435i_rplidar.launch.py`;
- motion, mapping-quality, or Nav2 acceptance.

## Next test

Run one stationary interval and one short labelled route, then report together:

1. timestamped map path versus occupancy geometry;
2. scan-map 0.15/0.30 m ratios;
3. best orientation candidate;
4. map revision and scan timestamp;
5. `scan_raw` versus gated `/scan` counts and the GUI `scan_age` /
   `scan_end_age` values;
6. whether the displayed path still moves when slam_toolbox updates `map -> odom`;
7. whether the GUI reports `OK_DELAYED` with a finite delay instead of a
   persistent zero-point `TF_WAIT` state.

Only after this observation boundary is stable should `minimum_time_interval`
be A/B tested at `0.5 s` versus `0.2 s`.
