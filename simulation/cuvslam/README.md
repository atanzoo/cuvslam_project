# cuVSLAM simulation

This directory contains the simulated/replay cuVSLAM workstream: Gazebo
worlds, Isaac ROS simulation adapters, launch files, experiment runners,
analysis tools, and simulation evidence.

Use ROS domain `43`. Simulation truth is evaluation-only and must not be fed
back into cuVSLAM, localization, Nav2, or command control.

This workstream must not source or mount the real-robot workspace.
