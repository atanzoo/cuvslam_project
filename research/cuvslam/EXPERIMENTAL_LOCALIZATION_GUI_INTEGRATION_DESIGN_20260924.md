# Experimental localization test area in the main GUI

## Objective

Integrate a read-only localization test area into the existing `8765` GUI.
The area provides a `開始定位` action, candidate-map metadata, AMCL pose,
timestamped scan/map overlap ratios, and a map canvas with explicit WAIT
states.

## Non-goals and protected boundaries

- This is not Nav2 navigation and does not add a planner, controller, or
  `/cmd_vel` publisher.
- The experimental launch remains the only owner of `map -> odom` during this
  stage. The GUI refuses to start it while an existing R2/SLAM runtime is
  active.
- The candidate map remains experimental; a visible pose or overlap ratio is
  not localization-accuracy acceptance.
- No Jetson runtime is started by implementation or static verification.

## Selected design

Extend the existing GUI telemetry path (option A). The collector adds an
optional AMCL pose payload while retaining the existing map and timestamped
scan payloads. The main GUI renders the localization pose and scan endpoint
colors over the occupancy grid. A small Jetson control script starts and stops
the isolated experimental localization container and exposes the same
collector stream for the GUI.

## Failure modes and safeguards

- Missing `/map`, `/amcl_pose`, `/scan`, or TF remains `WAIT`; no values are
  synthesized.
- An active R2/RPLIDAR runtime blocks experimental localization startup to
  prevent competing `map -> odom` and `/map` owners.
- The control script uses a dedicated container and dated map path; it does
  not change the protected map or safety behavior.

## Rollback and verification

Rollback is limited to removing the new GUI controls/state, collector fields,
and the experimental control script; the existing R2 path is unchanged.

Verification: Python syntax, existing contract/unit tests, HTML/JavaScript
static checks, remote script syntax, SHA-256 comparison after synchronization,
and a no-runtime GUI smoke page. ROS topic/TF and hardware acceptance remain
not run until explicitly requested.
