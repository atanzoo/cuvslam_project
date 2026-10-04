# cuVSLAM research records

Use this directory for cuVSLAM hypotheses, experiment plans, parameter
decisions, and cross-run interpretations. Keep raw runtime output and dated
acceptance reports under `simulation/cuvslam/evidence/` or
`real_robot/cuvslam/evidence/` according to the execution environment.

Each record should identify the environment, hardware, ROS/middleware and
software versions, parameters, procedure, metrics, evidence paths, conclusion,
and next step. Simulation results must remain explicitly separate from
real-camera claims.

Current deployment record:

- `jetson_workspace_organization_20260831.md` — Jetson filesystem migration,
  approved evidence cleanup, compatibility paths, verification, and rollback.
- `real_d435i_lidar_odom_ekf_shadow_20260903.md` — diagnostic LiDAR
  scan-matching odometry + EKF shadow architecture, ownership contract,
  verification status, and hardware follow-up gate.
- `real_d435i_cuvslam_baseline_emitter_ab_20260910.md` — current-space
  cuVSLAM-only navigation baseline, emitter A/B result, evidence boundaries,
  and the deferred LiDAR odometry/EKF enhancement.
- `real_d435i_scan_map_observability_20260911.md` — timestamped map-frame path,
  dynamic scan-map alignment diagnostics, and GUI process-record clearing.
