#!/usr/bin/env python3
"""Smoke tests for the standalone IMM dataset/training/evaluation pipeline."""

from pathlib import Path
import tempfile
import numpy as np

from generate_d1_edu_imm_dataset import generate_dataset
from train_d1_edu_imm import train
from evaluate_d1_edu_imm import _metrics


def main() -> None:
    data, manifest = generate_dataset(episodes=12, seed=11, duration=5.0)
    assert manifest["samples"] == len(data["episode_seed"])
    assert set(np.unique(data["split"])) == {"train", "val", "test"}
    assert len(set(data["episode_seed"][data["split"] == "train"]) & set(data["episode_seed"][data["split"] == "test"])) == 0
    with tempfile.TemporaryDirectory() as td:
        root = Path(td); dataset = root / "data.npz"; model_path = root / "model.json"
        np.savez_compressed(dataset, **data)
        model = train(dataset, model_path, epochs=30)
        assert model_path.exists() and len(model["mode_weights"]) == 9
        raw = np.load(dataset, allow_pickle=True); loaded = {k: raw[k] for k in raw.files}
        metrics = _metrics(loaded, model)
        assert metrics["samples"] > 0
        assert np.isfinite(metrics["ADE_m"])
    print("PASS IMM training pipeline")


if __name__ == "__main__":
    main()
