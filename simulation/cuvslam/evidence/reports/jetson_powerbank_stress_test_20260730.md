# Jetson Power-Bank Stress Test

Date: 2026-07-30
Target: Jetson AGX Orin Developer Kit
JetPack: 5.1.3 / Jetson Linux R35.5.0
Power mode: `MODE_30W`
Power source: User-provided power bank
Final runtime state: All test workloads stopped

## Scope

This test checked whether the power bank could keep the Jetson stable under:

1. idle system load;
2. an eight-worker CPU cryptographic workload; and
3. the representative Gazebo, ROS bridge, and cuVSLAM GPU workload.

The test did not move the simulated robot and did not change power mode,
cuVSLAM parameters, TF, or simulation files.

## Initial State

- SSH on `192.168.55.1:22`: reachable
- Failed system units: none
- Failed user units: none
- Root storage: 75% used, 14 GB available
- Memory: 61 GiB total, approximately 59 GiB available
- Initial CPU temperature: approximately 43 C
- Board temperature: approximately 33 C

The system had no external network connection and had not synchronized its
clock. Runtime timestamps therefore started at 1970 and are not suitable for
cross-machine timing evidence.

## Container Exit 137

The Isaac ROS container initially showed exit code 137. Docker reported:

```text
OOMKilled=false
RestartCount=0
RestartPolicy=no
```

Its finish time matched the previous controlled Jetson shutdown. During final
cleanup, `docker stop -t 10` also ended with 137 and `OOMKilled=false`. This
indicates that the container's main process did not exit within Docker's
10-second grace period and was killed. It is not evidence of power loss or
memory exhaustion.

## CPU Stage

Workload: OpenSSL SHA-256, eight workers.

Observed:

- all eight online CPU cores reached 100% utilization;
- CPU frequency reached 1728 MHz;
- maximum CPU temperature: approximately 44.3 C;
- SSH remained connected;
- no swap use;
- no new undervoltage, overcurrent, brownout, or thermal-throttle event.

Result: pass.

## Gazebo and cuVSLAM Stage

Workload:

- `cuvslam_mapping_simple.sdf`;
- Gazebo headless rendering;
- Gazebo-to-ROS bridge;
- Isaac ROS Visual SLAM 2.1.0 / cuVSLAM 11.4;
- stereo-only tracking;
- no robot motion.

Observed:

- Gazebo and bridge remained active;
- cuVSLAM completed GPU warm-up in approximately 6.85 seconds;
- `/visual_slam/status` remained available;
- final `vo_state=1`;
- maximum CPU temperature: 45.281 C;
- maximum GPU temperature: 38.843 C;
- sampled GPU load: approximately 17% to 31%;
- system load average peaked above 7 during the representative workload;
- SSH remained connected;
- no new undervoltage, overcurrent, brownout, or thermal-throttle event.

Result: pass for the tested workload and duration.

## Limitations

- INA3221 voltage/current channels require root and could not be sampled without
  interactive sudo authentication.
- This test proves short-duration stability, not battery runtime or remaining
  capacity.
- The test did not apply simultaneous maximum CPU and maximum GPU synthetic
  load because the representative project workload is the relevant acceptance
  case.
- USB networking provided connectivity but no internet/NTP synchronization.

## Final Shutdown Evidence

```text
cuvslam-powerbank-bridge.service: inactive
cuvslam-powerbank-gazebo.service: inactive
Isaac ROS container: exited
Container OOMKilled: false
/clock: no message during a 3-second sample
```

## Decision

The power bank passes the short CPU and representative Gazebo + cuVSLAM stress
test in Jetson `MODE_30W`. It is suitable for continued bench testing.

Before unattended or mobile operation, separately verify:

- battery runtime at the same workload;
- the power bank's rated continuous output and cable rating;
- graceful low-battery shutdown behavior;
- INA3221 voltage/current telemetry with authorized root access.
