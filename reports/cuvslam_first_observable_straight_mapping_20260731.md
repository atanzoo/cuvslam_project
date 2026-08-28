# First Observable-World Straight Mapping Geometry Report

Date: 2026-07-31 (Asia/Taipei)  
Environment: Jetson AGX Orin, ROS 2 Humble container, ROS domain 43  
Estimator: Isaac ROS Visual SLAM 2.1.0 / cuVSLAM 11.4  
IMU fusion: disabled  
Camera: simulated D435i stereo, 424 x 240, nominal 30 Hz

## Decision

The first 1 m observable-world straight mapping run completed successfully.
The trajectory gate passed, but the sparse landmark geometry gate failed.
Do not proceed to the 90-degree turn gate yet.

Jetson bundle:

```text
/home/tseng/isaac_ros_ws/data/straight_mapping_observable_20260731_35497
```

## Runtime result

- native truth distance: `1.0140 m`;
- cuVSLAM inferred base distance: `0.9548 m`;
- endpoint residual: `0.0753 m`;
- false rotation: `1.46 deg`;
- status: 581 / 581 samples with `vo_state=1`;
- left/right images: 571 / 571;
- exact stereo timestamp skew: `0.000 ms`;
- landmark clouds: 3 non-empty messages;
- final finite landmarks: 379.

The run stopped normally, published zero velocity, and removed all Gazebo,
bridge, recorder, cuVSLAM, and container workloads.

## Trajectory geometry

The trajectory comparison uses the first synchronized pose for rigid SE(3)
alignment. Scale remains fixed; no Sim(3) or truth-based correction is used.

| Metric | RMSE | Median | P95 | Maximum |
|---|---:|---:|---:|---:|
| ATE translation | 0.0495 m | 0.0380 m | 0.0746 m | 0.0753 m |
| ATE rotation | 0.8522 deg | 0.7023 deg | 1.3095 deg | 1.4553 deg |
| 1 s RPE translation | 0.0101 m | 0.0060 m | 0.0210 m | 0.0316 m |
| 1 s RPE rotation | 0.1705 deg | 0.1236 deg | 0.3122 deg | 0.4289 deg |

This is a good straight trajectory result and is consistent with the earlier
1 m repeatability gate.

## Landmark-to-SDF geometry

The final `map` landmark cloud was aligned to the simulation world using the
first synchronized truth and cuVSLAM base poses. The resulting
`world <- map` translation was effectively zero. Each point was compared with
the nearest surface among 50 static SDF box surfaces, including the floor,
walls, feature boxes, near-field boards, and floor markers.

| Metric | Result |
|---|---:|
| Surface-distance median | 0.2430 m |
| Surface-distance P90 | 1.7618 m |
| Surface-distance P95 | 2.1708 m |
| Maximum surface distance | 5.0750 m |
| Within 0.10 m | 136 / 379, 35.9% |
| Within 0.25 m | 190 / 379, 50.1% |
| Within 0.50 m | 239 / 379, 63.1% |
| Over 0.25 m | 189 / 379, 49.9% |

Aligned world bounds:

```text
X [0.7254, 8.0953] m
Y [-2.3319, 2.2789] m
Z [-0.7449, 1.8953] m
```

The nearest-surface comparison is generous because every known static surface
is eligible. Nearly half the points still remain more than 25 cm from any
surface, and some extend several metres beyond the arena. The failure is
therefore not explained by a small map/world alignment offset.

## Interpretation

The near-field scene fixed metric camera motion, but accurate tracking does
not guarantee accurate exported sparse landmark depth. The trajectory is
dominated by robust feature tracks, while the final landmark cloud retains a
large population of weak or incorrectly triangulated points.

The result is also inconsistent with a single global scale error:

- trajectory scale is approximately `0.9548 / 1.0140 = 0.942`;
- landmark errors have a broad, heavy-tailed distribution;
- rigid alignment is already effectively identity;
- no fixed scale multiplier can remove the 5.075 m maximum surface error.

## Startup defect found and fixed

Initial attempts exposed a false readiness condition. The bridge advertised
camera topic names even when Gazebo had exited because the observable world
was absent from the Jetson install space. The mapping workflow now:

- captures Gazebo and bridge logs in every bundle;
- requires an actual image message, not only a topic name;
- uses a unique Gazebo partition per run;
- installs the observable world through `slam_gazebo`.

The installed world SHA-256 is:

```text
9a6e937c027779c199618ea3a7d39883439c53d636cc2fc58c9643acb794d66f
```

The Jetson system clock was `1970-01-01` and NTP was not synchronized. This
does not change the simulation-time geometry metrics, but it should be fixed
before evidence requiring trustworthy wall-clock timestamps.

## Next gate

Repeat the same 1 m straight mapping once from cold start. Require:

- trajectory ATE P95 at most 0.10 m;
- at least 80% of final landmarks within 0.25 m of an SDF surface;
- landmark surface-distance P90 at most 0.25 m;
- outliers over 0.25 m at most 20%;
- exact stereo pairing and `vo_state=1` throughout.

If the landmark failure repeats, inspect landmark export confidence and
triangulation depth versus stereo disparity before attempting a turn. Do not
use Sim(3), landmark clipping, or truth-assisted correction to pass the gate.

