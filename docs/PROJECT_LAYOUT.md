# Project layout

The repository is separated by execution workstream. Root governance files
remain stable so every task has one policy and handoff entry point.

```text
real_robot/cuvslam/          RealSense D435i and real-camera cuVSLAM
simulation/cuvslam/          Gazebo/replay cuVSLAM experiments
simulation/path_planning/    D1 Edu, RPLIDAR/Nav2, and path-planning simulation
research/                    Hypotheses, experiment notes, and papers
shared/                      Common Jetson, connection, and display resources
docs/                        Current contracts, handoffs, and design index
```

Executable source, deployment files, and tools belong to their workstream.
Runtime logs and reports are stored under that workstream's `evidence/`
directory. Historical evidence is preserved; it is not rewritten to make an
experiment pass.

The current entry points are:

- Root `Real D435i R1.5 Axis Test.command`, which delegates to the real-camera GUI.
- `shared/tools/jetson_connection_web.py`, the shared connection GUI.
- `simulation/cuvslam/Foxglove Simulation.command`, the simulation viewer launcher.

The old root `Jetson Connection.command` and `Foxglove Simulation.command`
are no longer present. Use the owning workstream's entry point instead.

Do not source `/home/tseng/slam_ws` or deploy to it during simulation work.
Simulation deployment remains `/home/tseng/jetson_slam_ws`; real-robot
deployment remains `/home/tseng/slam_ws` and requires explicit review.

## Jetson runtime mirror

The storage-constrained Jetson migration on 2026-08-31 keeps the existing
`/home/tseng/isaac_ros_ws` workspace and separates its runtime files by
workstream without duplicating source, bags, or Docker layers:

```text
/home/tseng/isaac_ros_ws/real_robot/cuvslam/
/home/tseng/isaac_ros_ws/simulation/cuvslam/
/home/tseng/isaac_ros_data/real_robot/cuvslam/
```

The old top-level `tools/`, `config/`, and `data/` paths, plus
`/home/tseng/isaac_ros_data/experiments`, are compatibility symlinks. The
complete migration record is
`research/cuvslam/jetson_workspace_organization_20260831.md`.
