#!/usr/bin/env python3
"""Drive straight out and reverse along the same line using truth stop gates."""

from __future__ import annotations

import argparse
import time

import rclpy

from simulation_truth_distance_profile import TruthDistanceProfile


class TruthRoundTripProfile(TruthDistanceProfile):
    def check_health(self) -> None:
        if self.vo_state != 1:
            raise RuntimeError(f"cuVSLAM tracking lost: vo_state={self.vo_state}")
        if self.last_scan_at is None or time.monotonic() - self.last_scan_at > 1.0:
            raise RuntimeError("LaserScan is stale for more than 1.0s")
        if self.minimum_range is not None and self.minimum_range < self.stop_distance:
            raise RuntimeError(
                f"obstacle clearance {self.minimum_range:.3f}m is below "
                f"{self.stop_distance:.3f}m"
            )

    def drive_leg(
        self,
        phase: str,
        speed: float,
        complete,
        timeout: float,
    ) -> None:
        started = time.monotonic()
        next_report = started
        while rclpy.ok() and not complete():
            rclpy.spin_once(self, timeout_sec=0.01)
            self.check_health()
            if time.monotonic() - started > timeout:
                raise RuntimeError(
                    f"{phase} timeout at truth distance "
                    f"{self.distance():.4f}m"
                )
            self.publish_velocity(speed)
            now = time.monotonic()
            if now >= next_report:
                print(
                    f"phase={phase}, truth_distance={self.distance():.4f}m, "
                    f"min_scan={self.minimum_range:.3f}m, "
                    f"vo_state={self.vo_state}",
                    flush=True,
                )
                next_report = now + 2.0
            time.sleep(0.02)
        self.stop()

    def run_roundtrip(
        self,
        timeout_per_leg: float,
        return_tolerance: float,
    ) -> None:
        self.wait_until_ready()
        self.drive_leg(
            "outbound",
            self.speed,
            lambda: self.distance() >= self.target_distance,
            timeout_per_leg,
        )
        outbound_distance = self.distance()
        print(
            f"turnaround: truth_distance={outbound_distance:.4f}m, "
            f"vo_state={self.vo_state}",
            flush=True,
        )
        self.drive_leg(
            "return",
            -self.speed,
            lambda: self.distance() <= return_tolerance,
            timeout_per_leg,
        )
        print(
            f"complete: outbound_truth_distance={outbound_distance:.4f}m, "
            f"closure_truth_distance={self.distance():.4f}m, "
            f"vo_state={self.vo_state}",
            flush=True,
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-distance", type=float, default=1.0)
    parser.add_argument("--speed", type=float, default=0.12)
    parser.add_argument("--stop-distance", type=float, default=0.35)
    parser.add_argument("--return-tolerance", type=float, default=0.025)
    parser.add_argument("--timeout-per-leg", type=float, default=35.0)
    args = parser.parse_args()
    if min(
        args.target_distance,
        args.speed,
        args.stop_distance,
        args.return_tolerance,
        args.timeout_per_leg,
    ) <= 0:
        parser.error("all numeric arguments must be positive")
    if args.return_tolerance >= args.target_distance:
        parser.error("return tolerance must be less than target distance")

    rclpy.init()
    node = TruthRoundTripProfile(
        args.target_distance,
        args.speed,
        args.stop_distance,
    )
    try:
        node.run_roundtrip(args.timeout_per_leg, args.return_tolerance)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
