# Nav2 map-3 fixed-object export: local preflight only

Date: 2026-10-01 (Asia/Taipei)  
Status: **offline map preparation passed; real Nav2 and closed-loop gates not run**.  
Environment: local Mac project `.venv`, candidate map scale 3, D1 Max kinematic proxy scene reset seed 10042 / curriculum 0 / map-scene seed 1369567760. No ROS, Jetson, hardware, training, or checkpoint use.

Later update: this is the historical offline preflight, not the current gate status. The initial derived YAML's inherited `free_thresh: 0.25` caused real Nav2 Foxy to classify gray 205 as Free. The corrected, separate Nav2-only export and Jetson readback are documented in [the follow-up evidence](20261001_nav2_map3_jetson_path_and_proxy_gate.md). The original failed artifact is retained for diagnosis.

## Protected inputs and generated output

- Source YAML SHA-256: `be42f16b9730106c54f0bb75e83b04ec48b7c2c69f49e12c4fb4183ca21feb5d`.
- Source PGM SHA-256: `41e913c872bd118fc971b313024339f78e23247aec94fde00a18e51aa62efd74`.
- The exact reset fixed box is center `(-3.145, 2.3179942314911184)` m, half-length/half-width `0.25/0.25` m. The pedestrian is **not** rasterized into the static map.
- Generated-only directory: `output/nav2_map3_static_seed10042_20261001/`; original YAML/PGM were not written.
- Export: 597 × 924 cells, 0.05 m/cell, origin `(-11.67, -16.02, 0)` m; 121 occupied cells added for the fixed box.
- Output YAML SHA-256: `9b508f2a2fc976f4b15f152ef789dea6f52cb8c00620b371133ae88bb3acaa17`.
- Output PGM SHA-256: `4acb7b47b6c0315da74e18e7c0347bd0603627cc5389790b8668ecdce2aab921`.

The new exporter `tools/d1_nav2_map_export.py` uses the same 3× nearest-neighbor cell replication as the existing in-memory candidate-map loader. It refuses a fixed box touching occupied/unknown/outside cells and refuses to overwrite an output directory. New exports reload their own PGM/YAML and compare **all pixels and map metadata** before writing a success manifest. The earlier seed-10042 artifact, created before that read-back check was added, was separately reloaded and compared against all 551,628 expected pixels, resolution, origin, and manifest hashes: PASS.

## Checks actually run

- `PYTHONPATH=simulation/path_planning/tools .venv/bin/python -m unittest test_d1_nav2_map_export -v`: 4 tests PASS (scale/orientation, unknown preservation, invalid-box rejection, no overwrite).
- Python compile of exporter and test: PASS.
- Existing experimental localization contract test: 7 tests PASS; notably this verifies the launch **excludes** `planner_server`, `controller_server`, and `/cmd_vel`. It is not a Nav2 navigation test.
- Offline `CandidateMapPlanner` on the derived map found a 10.14056 m / 186-point route for the reset start/goal. Dense sampling at 697 points (≤0.025 m spacing) stayed traversable, with minimum robot-center distance to the physical fixed-box edge of 0.6450 m versus the local 0.61 m radius-plus-margin requirement. **This is only an offline A* geometry preflight, not a Nav2 path or contact result.**

## Gate still open

The Mac has no `ros2`, `/opt/ros`, Docker/Podman, or detected VM runtime. The expected USB link to Jetson `192.168.55.1` was not present in local interfaces when checked; sandboxed SSH returned `Operation not permitted`. No Jetson connection, remote backup, sync, Nav2 lifecycle, costmap, planner action, TF, simulated command ownership, or MuJoCo contact replay has been validated. Do not start PPO or claim the fixed object/boundary gate passed. Resume with read-only Jetson connectivity/runtime inventory, then only the approved isolated simulation workspace and the design gates in `research/path_planning/NAV2_IN_LOOP_MAP3_PPO_DESIGN_20261001.md`.
