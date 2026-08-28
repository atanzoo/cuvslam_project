# cuVSLAM Project Handoff

> This is the handoff for the independent cuVSLAM development line.
>
> It must not be used as the handoff for the existing LiDAR/Nav2 project. The
> two projects are compared later through a controlled evaluation process.

- Last historical update: 2026-08-17 (Asia/Taipei)
- Project root: `/Users/tsengpochien/Desktop/cuvslam_project`
- Active current-state entry point: `docs/CUVSLAM_CURRENT_STATUS_20260825.md`
- Latest real-camera correction record: `logs/real_d435i_quality/20260825_r2_correction_log.md`
- Note: sections below preserve the earlier simulation and project history. For
  the current real D435i R2 status, metrics, and next test, read the active
  current-state document first.
- Status: NVIDIA reference passed; wheel odometry is isolated; the D435i
  200 Hz IMU contract is verified; stereo-only in-place 90-degree pose now
  passes two independent cold starts in the dense-turn v3 world
- Primary target: NVIDIA Jetson AGX Orin with Intel RealSense D435i
- Real-camera mapping entry point: `docs/REAL_D435I_MAPPING_HANDOFF_20260817.md`
- Current implementation status: Isaac ROS Visual SLAM 2.1.0 / cuVSLAM 11.4
  runs in the isolated Jetson container. The official NVIDIA release-2.1
  stereo bag produces complete odometry output. A controlled baseline 1 m run
  proved that every in-motion stereo frame was unique while cuVSLAM still
  underestimated translation by 92.6 percent. An independent near-field
  observable-world A/B then produced 1.0232 m for 1.0128 m truth without
  estimator or calibration changes. Two additional cold-start trials produced
  1.0189 m for 1.0176 m truth and 0.9708 m for 1.0152 m truth. The sparse
  far-field Gazebo scene, not stale pixels, was the root cause of the current
  1 m simulation failure.

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
configuration. See `reports/cuvslam_corner_exploration_20260731.md`.

The D435i audit then proved that `/ground_truth/odom` was wheel odometry, not
physical turn truth. A nominal 91-degree wheel-odometry turn was only
79.28 degrees in Gazebo native pose, while 200 Hz IMU integration measured
79.29 degrees. The route controller now stops from timestamped IMU gyro
integration; a short validation reached 89.74 degrees by native pose and
89.76 degrees by bag IMU. A native-truth A/B found that enabling cuVSLAM IMU
fusion worsened turn rotation residual from 11.28 to 12.72 degrees and false
turn translation from 0.2738 to 0.3251 m. Keep fusion disabled by default.
See `reports/cuvslam_d435i_imu_corner_audit_20260731.md`.

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
`reports/cuvslam_in_place_turn_pose_fix_20260731.md`.

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
`reports/cuvslam_initial_dense_v3_mapping_20260731.md`.

## 1. Objective

Develop and evaluate an independent cuVSLAM solution in the following order:

1. Establish the Jetson, CUDA, JetPack, camera, and IMU environment.
2. Build a cuVSLAM visual map with D435i stereo and IMU data.
3. Validate cuVSLAM map save/load and simulated or replay localization.
4. Validate real-robot localization against the visual map.
5. Integrate cuVSLAM with simulated Nav2 only after localization has a stable
   measured interface.
6. Integrate with real-robot Nav2 only after simulation and real localization
   gates pass.
7. Compare cuVSLAM with the existing LiDAR ICP solution using the same routes,
   hardware conditions, and reference measurements.

## 2. Non-goals for the initial project

- Do not modify the existing production LiDAR ICP implementation.
- Do not modify the existing AMCL tuning or active 2D occupancy map.
- Do not replace the existing `/Applications/slam_v2` navigation baseline.
- Do not treat a cuVSLAM 3D landmark map as a Nav2 2D occupancy map.
- Do not claim real-robot accuracy from Gazebo truth or paper benchmarks alone.
- Do not connect cuVSLAM to the production Nav2 runtime before an approved
  integration design exists.

## 3. Planned system boundary

```text
RealSense D435i stereo + IMU
        |
        v
Jetson AGX Orin + cuVSLAM
        |
        +--> visual map and keyframe/landmark data
        |
        +--> visual trajectory and tracking health
        |
        v
Independent localization and evaluation interfaces
        |
        v
Later: simulation Nav2, then real-robot Nav2
```

The initial project owns visual mapping, visual localization, camera/IMU
calibration, data capture, evaluation, and deployment records. Navigation
integration is a later phase, not an implicit side effect of building the map.

## 4. Spatial representation

The project must distinguish these representations:

| Representation | Meaning | Initial owner |
|---|---|---|
| Camera/IMU frames | Sensor geometry and measurements | Calibration records |
| Local visual map | Keyframes, 3D landmarks, observations | cuVSLAM frontend |
| Global visual map | Loop-closed visual map and pose graph | cuVSLAM backend |
| Visual trajectory | Time-ordered pose estimate | cuVSLAM runtime |
| 2D navigation map | Occupancy/free-space representation for Nav2 | Later integration design |

The mapping phase must not silently claim that a visual landmark map provides
complete occupancy, free-space, or collision geometry.

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

### Phase B: Independent mapping

Run cuVSLAM without Nav2 integration. Store the visual map, trajectory,
configuration, calibration identifiers, and recording metadata as one immutable
experiment bundle.

Exit criteria:

- map save/load succeeds;
- tracking health is recorded;
- trajectory has no unexplained discontinuity;
- resource usage and dropped-frame behavior are measured;
- at least one repeatable route is recorded.

### Phase C: Simulated or replay localization

Use replay or a declared simulated sensor source to test map loading,
localization, relocalization, reset, and tracking loss behavior.

Exit criteria:

- localization can start from declared initial conditions;
- failure and recovery behavior is observable;
- timestamps and frame conventions remain valid;
- results are not described as real-camera accuracy.

### Phase D: Real-robot localization

Use the D435i and Orin on the robot. Compare repeated routes, difficult visual
conditions, and long-duration drift against a declared reference.

Exit criteria:

- repeatable startup and shutdown;
- visual map relocalization is measured;
- tracking loss behavior is safe and recorded;
- camera, IMU, and robot frame alignment is verified;
- A/B dataset plan with LiDAR ICP is complete.

### Phase E: Simulated Nav2

Create an approved integration boundary and test navigation with declared map,
pose, obstacle, and safety sources.

Exit criteria:

- navigation consumes the intended pose and map interfaces;
- planner/controller/costmap behavior is repeatable;
- pose discontinuity and tracking loss produce safe behavior;
- simulation-specific assumptions are documented.

### Phase F: Real-robot Nav2

Only after Phase E, connect to the real robot with human-approved deployment
and emergency-stop procedures.

Exit criteria:

- real sensor and timing behavior is measured;
- localization failure stops the robot safely;
- navigation, obstacle, and recovery tests pass;
- logs and rollback procedure are complete.

## 6. Current decisions

- cuVSLAM is an independent solution, not an immediate replacement provider.
- The existing LiDAR/Nav2 project remains the comparison baseline.
- The first deliverable is a standalone visual mapping and localization result.
- The first comparison must use real D435i, IMU, LiDAR, and the same physical
  routes where possible.
- Backend loop-closure corrections must not be assumed safe for a continuous
  navigation odometry interface without a separate design review.

## 7. Open decisions

Before implementation, decide and record at least two options for each:

- cuVSLAM SDK/API and supported JetPack/ROS 2 version;
- stereo versus RGB-D operating mode;
- camera/IMU synchronization method;
- visual map storage and versioning format;
- replay versus Gazebo sensor simulation for Phase C;
- reference system for localization accuracy;
- 2D navigation map source for later Nav2 integration;
- failure handoff behavior if visual tracking is lost;
- independent repository versus nested repository boundary.

## 8. Required evidence bundle

Every experiment must preserve:

- experiment ID and date;
- hardware serials and software versions;
- calibration identifiers;
- configuration and parameter snapshot;
- input recording or source reference;
- output visual map and trajectory identifiers;
- tracking and resource logs;
- metrics and plots;
- operator notes and known anomalies;
- conclusion limited to what the evidence supports.

## 9. Simulation entry point

The environment-design gate is complete for the isolated Jetson container.
Start with the procedure in `docs/SIMULATION_HANDOFF.md`.

The first simulation target is sensor replay or declared sensor simulation:
D435i stereo/IMU for cuVSLAM, LiDAR `/scan` for Nav2 obstacle handling, and a
dedicated dynamic-obstacle decision layer. Use ROS domain `43` for simulation
so it cannot accidentally exchange traffic with the verified Isaac ROS domain
`42` or the old Foxy systems.

Do not connect motors, serial control, or production Nav2 during this phase.

## 10. Current Verified State

### Runtime and frame contract

- Jetson: AGX Orin, JetPack 5.1.3 / Jetson Linux R35.5.0.
- Host ROS: Foxy; isolated Isaac ROS container: Humble.
- Simulation ROS domain: `43`.
- Estimator: Isaac ROS Visual SLAM 2.1.0, cuVSLAM 11.4.
- Normal mode: rectified stereo, `enable_imu_fusion=false`.
- Output ownership: cuVSLAM publishes `odom -> base_link`.
- The release-2.1 physical camera-frame contract, right CameraInfo baseline,
  and static TF tree have been audited. Do not change them without new direct
  evidence and a C2 design review.
- NVIDIA debug dump is opt-in and disabled by default.

### NVIDIA control

The SHA-256-verified NVIDIA release-2.1 `small_pol_test` bag produced:

- 725 left images;
- 725 odometry samples;
- 725 status samples;
- `vo_state=1` for all samples;
- output frame `odom -> base_link`.

The official bag has no ground truth, so this proves runtime/data-path
functionality, not absolute accuracy.

### Current Gazebo failure

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

### 2026-07-31 boundary rerun

A controlled isolated Gazebo/bridge run on Jetson endpoint `192.168.55.1`
used partition `cuvslam_boundary_20260731`. Native truth moved from
`x=0.5148 m` to `x=1.0308 m`; all three pixel boundaries changed throughout
the motion interval `265.15–269.55 s`. Long repeated runs began at about
`269.545 s`, after native truth had stopped, in Gazebo Transport, both ROS
image topics, and the NVIDIA debug dump.

This run therefore does not identify an in-motion stale-pixel owner. Evidence
is in `reports/image_freeze_boundary_20260731.md` and the Jetson bundle under
`/home/tseng/isaac_ros_ws/data/boundary_20260731/`.

The same bag still failed the trajectory gate: native truth moved `0.516 m`,
while relative cuVSLAM tracking odometry moved approximately `0.172 m`, with
`vo_state=1` for all 332 samples. This boundary result motivated the reliable
single-owner baseline below, which supersedes the stale-pixel hypothesis.

### 2026-07-31 1 m observability fix

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
mapping gates. See `reports/cuvslam_1m_observability_fix_20260731.md`.

Do not apply an odometry scale multiplier, force planar mode, TF adjustment,
CameraInfo retuning, or IMU fusion to hide this upstream defect.

### 2026-07-31 repeatability result and next experiment

The observable world now covers every leg of the square route with 30
visual-only near-field features. Two additional cold-start 1 m trials passed:

- repeat 1: truth `1.0176 m`, cuVSLAM `1.0189 m`, rotation `0.78 deg`;
- repeat 2: truth `1.0152 m`, cuVSLAM `0.9708 m`, rotation `1.32 deg`.

All in-motion ROS and NVIDIA debug images were unique, stereo timestamps
matched exactly, and every status sample remained `vo_state=1`. See
`reports/cuvslam_observable_world_repeatability_20260731.md`.

The first 1 m straight mapping run completed with truth `1.0140 m`, cuVSLAM
`0.9548 m`, fixed-scale translation ATE P95 `0.0746 m`, rotation ATE P95
`1.31 deg`, and 581 / 581 samples at `vo_state=1`. The trajectory gate passed.

The final sparse cloud contained 379 landmarks, but only 50.1 percent were
within 0.25 m of any static SDF surface. Surface-distance P90 was `1.7618 m`,
and the maximum was `5.0750 m`. The landmark geometry gate therefore failed.
See `reports/cuvslam_first_observable_straight_mapping_20260731.md`.

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
`reports/cuvslam_landmark_repeatability_and_weight_20260731.md`.

Keep the sparse world as a negative control. The straight 1 m result does not
yet establish turn accuracy, loop closure, map persistence, or relocalization.

### Power and shutdown

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

- `reports/nvidia_reference_and_gazebo_1m_debug_20260730.md`
- `reports/cuvslam_1m_observability_fix_20260731.md`
- `reports/cuvslam_observable_world_repeatability_20260731.md`
- `reports/cuvslam_first_observable_straight_mapping_20260731.md`
- `reports/cuvslam_landmark_repeatability_and_weight_20260731.md`
- `reports/jetson_powerbank_stress_test_20260730.md`
- `reports/cuvslam_level_chassis_straight_trials_20260730.md`
- `reports/cuvslam_stage4_official_input_contract_20260730.md`
- `docs/CUVSLAM_NVIDIA_REFERENCE_DEBUG_DESIGN.md`
- `docs/sensor_and_frame_contract.md`
- `docs/NEXT_THREAD_MAPPING_TEST_PROMPT.md`
