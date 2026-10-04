#!/usr/bin/env python3
"""Refresh saved failure replays with traces for the trajectory report.

The benchmark can skip trace recording to keep a Monte Carlo run light.  This
small utility replays the already-saved failure configurations with the exact
same seeds and writes the trace back into each replay JSON.  It does not
change the benchmark CSV or regenerate scenarios.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path

from run_d1_edu_decision_layer_viewer import (
    EpisodeConfig,
    run_episode,
)


def refresh_failure_traces(failure_dir: Path, maximum_trial: int | None = None) -> int:
    refreshed = 0
    for path in sorted(failure_dir.glob("trial_*.json")):
        payload = json.loads(path.read_text(encoding="utf-8"))
        trial = int(payload["trial"])
        if maximum_trial is not None and trial >= maximum_trial:
            continue

        config = EpisodeConfig(**payload["config"])
        seed = int(payload["seed"])
        result = run_episode(config, seed=seed, record_trace=True)
        payload["result"] = asdict(result)
        path.write_text(
            json.dumps(payload, indent=2, allow_nan=True),
            encoding="utf-8",
        )
        refreshed += 1
        print(f"refreshed trial={trial} trace_points={len(result.trace)}")
    return refreshed


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--failure-dir", type=Path, default=Path("output/failures"))
    parser.add_argument("--maximum-trial", type=int, default=None)
    args = parser.parse_args()
    count = refresh_failure_traces(args.failure_dir, args.maximum_trial)
    print(f"refreshed_failures={count}")


if __name__ == "__main__":
    main()
