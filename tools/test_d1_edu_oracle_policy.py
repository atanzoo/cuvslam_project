#!/usr/bin/env python3
"""Smoke tests for the ground-truth high-level oracle."""

from __future__ import annotations

import numpy as np

from d1_edu_oracle_policy import GroundTruthOraclePolicy


def main() -> None:
    policy = GroundTruthOraclePolicy()
    try:
        policy.act(np.zeros(1, dtype=np.float32))
    except RuntimeError:
        pass
    else:
        raise AssertionError("oracle must require ground-truth context")
    policy.reset_episode()
    print("PASS ground-truth oracle interface")


if __name__ == "__main__":
    main()
