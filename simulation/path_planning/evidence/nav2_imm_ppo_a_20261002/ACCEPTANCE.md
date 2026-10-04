# Option A staged interface acceptance

Date: 2026-10-02, Asia/Taipei. Scope: isolated Jetson Humble MPPI 1.1.12
and Mac MuJoCo D1 Max kinematic proxy, not a real robot.

User approved the custom-critic Option A, the listed 15 source/config files,
and Luna max implementation with parent review between checkpoints. Additional
mirrored/varied fixed-object layouts are waived and remain untested. Dynamic
interaction gates and existing stop/clearance requirements are not waived.

## Resume preflight

- Jetson `tseng@192.168.55.1` reachable. Disk free 7.8 GB; available RAM
  61024 MB. No running Docker container and no listener on bridge port 8987.
- The earlier network-none read-only dependency container is exited and
  preserved. Dependency results and exact installed headers are in `preflight/`.
- Immutable image:
  `sha256:934771b0dd2ad60b7bad76e97c2dcf25fc71a0b0a2db09ee009951f5123adbe6`.
- Remote isolated area:
  `/home/tseng/jetson_slam_ws/simulation/path_planning/nav2_imm_ppo_a_20261002`.
- Parent reran 6 bridge-guard tests and 5 command-evidence tests: all 11 passed.
  These are unit tests, not newly run Nav2 movement gates.
- Offline reset/query inspection (seed 10042, pedestrian curriculum 0):
  52-element observation, five perceived tracks, each with 36 finite forecast
  samples covering 0.0–3.5 s at 0.1 s spacing. CV/CA/CTRV weights summed to
  1.0; tracker-configured radius was 0.3535533905932738 m. No movement or
  ground-truth serialization was performed by this inspection. It confirms
  that the existing predictor can supply the required horizon without edits.
- Parent rechecked the preserved 437-point / 10.961558 m Nav2 route against
  exported geometry: 654 dense samples traversable; minimum fixed-box edge
  distance 0.869006 m (required 0.61 m). The preserved real Nav2 costmap
  snapshot also passed 1089 dense samples with original Occupied/Unknown
  blocked. These are geometry/snapshot checks, not new closed-loop runs.

Protected SHA-256 snapshots:

| Asset | SHA-256 |
|---|---|
| accepted export map.yaml | `ff5cd429ccb86783e6e39ce6017e2c40f1ec71a4e01c364e8b9799ad2f63a33e` |
| accepted export map.pgm | `4acb7b47b6c0315da74e18e7c0347bd0603627cc5389790b8668ecdce2aab921` |
| accepted export manifest.json | `f76335177619beb59f1de63d8442fb3b427a6c29d0c4889df5e5e77b63b5fb63` |
| stock v3 headroom YAML | `efdf0cad6717f8d9df09e44795e4f79a095b47c0d5aa5b8c364d447a571a07fa` |
| original candidate map.yaml | `be42f16b9730106c54f0bb75e83b04ec48b7c2c69f49e12c4fb4183ca21feb5d` |

The accepted export is under `../nav2_mppi_map3_seed10042_20261002/`;
the protected original is under
`real_robot/cuvslam/evidence/maps/20260924_092630_current_cuvslam_a2m12/`.

## Checkpoints

G2 build outputs will be separated under remote `g2_01/{src,build,install,log}`.
Build containers use the immutable image, no network or hardware mounts,
read-only root/source, bounded CPU/RAM/PIDs and separate writable output mounts.
  Runtime lifecycle checks, if build/test passes, remain in isolated ROS domain
178 with localhost-only discovery and a disarmed bridge. No image pull or
package installation is part of this scope.

Jetson host UTC readback during preparation was `1970-01-01T06:47:31Z`;
it is not synchronized to the Mac wall clock. No clock/NTP configuration is
changed. Use local experiment labels for chronology; retain simulation elapsed
time separately from pose-bound Jetson ROS receipt/query stamps and steady-clock
age. G3 must verify that mapping and reject stale/future/unverified input; do not
pretend the two machines share an absolute clock.

| Gate | Current acceptance |
|---|---|
| G1 pure contract / serialization | accepted: parent repeated 33/33 focused tests; Python 3.8 AST pass; actual tracker serialization pass |
| G2 typed message / ARM64 plugin / timing / disarmed load | Luna max implementation in progress; ARM64 build/load not run |
| G3 IMM bridge / dynamic scenarios / expiry | not run |
| G4 five preferences / SB3 adapter / brief interface smoke | not run |

Parent additionally executed G1 inside the pinned ARM64 container with the
installed `/usr/bin/python3` (target Python 3.8): all 33 tests passed in
0.017 s, process exit 0. Original output: `g1_python38_tests.log`.
The network-none/read-only container `nav2_imm_ppo_a_20261002_g1_python38`
is exited and preserved; it launched no ROS node or motion interface.

No full 25k training run, production workspace update, sensor runtime, motor
runtime, commit, push, or real-robot deployment is authorized by this resume.
Finite MPPI costs and post-step collision checks do not prove collision freedom.
Retain all failed contexts, trials, build logs, commands, and action results.

G1 freezes one versioned JSON contract with explicit epoch/observation
sequences, transactional input acceptance, matching pose sequence, 0.30 s
TTL, 3.5 s prediction coverage, 8 actors and 64 samples per actor. The serializer
reads perceived tracker fields only. Fresh empty sensing is valid. Unavailable
or exhausted WAIT keeps its requested action/reason and falls back to neutral
CRUISE, without reducing collision costs. The runtime producer and critic must
maintain a consecutive-WAIT ledger; the pure single-packet resolver alone does
not prove that a sender cannot restart its elapsed counter. This remains a
G2–G4 requirement. A parent exploratory check for a non-existent ledger API
failed with AttributeError; no runtime was active, and no such API is claimed.

## Dynamic-gate coverage concern identified before G3

The unchanged legacy crossing actor travels 0.70 m at 0.18 m/s and disappears
at approximately 3.99 s. The preserved v3 static-only trace approaches its
crossing y=-3.2678 m around 10–12 s. A successful route with the original early
crossing is therefore not evidence of avoiding a simultaneous crossing.
G3 must record whether a real predicted/ground-truth encounter occurs rather
than award dynamic-avoidance coverage for actor presence alone.

The user subsequently directed: "先不要管這個 這反而更能當作隨機事件".
Keep the existing appearance/disappearance schedule; do not retime it to force
an encounter or make encounter occurrence an added acceptance prerequisite.
Record actual actor exposure, perceived risk and encounter occurrence. Runs
without an encounter may validate transport/path following but do not establish
dynamic avoidance. The seeded schedule is still reproducible, not newly
randomized by this instruction. The protected source generator is unchanged.
