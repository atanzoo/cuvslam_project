# D1 Max proxy candidate-map scale-5 smoke

- Date: 2026-09-24 (Asia/Taipei)
- Change class: C3 simulation algorithm/integration experiment
- Scope: local MuJoCo D1 Max kinematic proxy, candidate map, scripted crossing
- Approval: 方案 A was approved before implementation

## Map and safety contract

Source map: `real_robot/cuvslam/evidence/maps/20260924_092630_current_cuvslam_a2m12/map.yaml`.

- Source raster: 199 x 308; source cell size: 0.05 m.
- Scene scale: 5x; in-memory raster: 995 x 1540; cell size stays 0.05 m.
- Simulation bounding box: 49.75 x 77.00 m; scaled origin: (-19.45, -26.70) m.
- Source cells: 28,405 known-free; 4,385 occupied; 28,502 unknown.
- Unknown remains non-traversable. Only observed occupied pixels become the
  1,082 static MuJoCo wall rectangles.
- D1 Max proxy radius candidate remains 0.49 m; obstacle margin 0.12 m;
  inflation 13 cells. Robot geometry is not enlarged with the map.
- Dynamic actor: one 0.85 m-high cylindrical pedestrian proxy, 0.45 m/s,
  crossing a corridor validated against the inflated known-free mask. The map
  route yielded 126 validated crossing anchors for deterministic sampling.

## Fixed-seed rollout

Configuration: seed 23, 24.0 s episode cap, 3.20 m map path, D1 Max
kinematic proxy, LiDAR + existing MPPI + Decision Layer + SimulatedD1SDK.
The baseline passed `CRUISE` as the high-level action; the existing safety
layer entered `AVOID` when risk was detected. The episode reached its goal at
12.608 s after 2.824 m of measured travel. Collision, physical contact,
map-boundary violation, and timeout were all false; minimum reported
clearance was 0.796 m. Episode seed: 77,115,417.

This is one scripted simulation run, not PPO training, a statistical
benchmark, a gait/contact result, or real-world safety acceptance. An earlier
16 s preview cap timed out close to the goal; extending only the preview cap to
24 s allowed the same deterministic episode to finish.

## Verification

Passed:

- `test_d1_candidate_map.py`
- `test_d1_map_planner.py`
- `test_d1_edu_decision_layer_sb3.py` (including map integration, 12 reset
  seeds, deterministic reset, and map-hash/scale checkpoint guard)
- `test_d1_edu_scenario.py`
- `test_d1_edu_decision_benchmark.py`
- SB3 runner `--help`
- one-trial map-aware baseline/policy evaluation-wiring smoke

Known adjacent failure: `test_d1_edu_decision_layer.py` fails its legacy
crossing case's final-mode assertion; the recorded final minimum clearance is
-0.056 m. This is separate from the candidate-map tests and was not modified
by this experiment.

## Boundary and rollback

No long PPO training, checkpoint production, Jetson access, robot motion, gait,
deployment, or map-file modification was performed. Candidate maps are
opt-in (`--candidate-map-yaml`); omitting that option retains the prior
environment behavior. The source map can be restored simply by leaving it
unselected; no raster rewrite or destructive cleanup occurred.
