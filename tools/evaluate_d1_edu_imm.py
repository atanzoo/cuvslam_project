#!/usr/bin/env python3
"""Evaluate heuristic versus calibrated IMM predictions on held-out seeds."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import numpy as np

from d1_edu_tracker_models import IMMObstaclePredictor
from train_d1_edu_imm import _summary, calibrated_prediction, KINDS


def _metrics(data, model=None, *, split="test", indices=None):
    idx = np.flatnonzero(data["split"] == split) if indices is None else np.asarray(indices)
    predictor = IMMObstaclePredictor()
    errors, final_errors, turn_errors, risks, covered, leads = [], [], [], [], [], []
    warning_events = []
    turn_true = data["kind"][idx] == "turning"
    turn_pred = []
    for n in idx:
        v, a = data["velocity"][n], data["acceleration"][n]
        if model is None:
            pred = predictor.predict(data["position"][n], v, a, np.array([0.5, 1, 2, 3]))
            weights = pred.model_weights
            turn = pred.turn_rate
            trajectory, tube = pred.trajectory, pred.uncertainty_radius
            turn_probability = float(abs(turn) >= 0.15)
        else:
            feat = _summary({k: data[k][n:n+1] for k in ("velocity", "acceleration", "noise", "dropout", "confidence", "miss_count")})
            trajectory, family, turn = calibrated_prediction(data["position"][n], v, a, feat[0], model)
            tube = np.asarray(model.get("tube_radius_m", [0.08] * 4), dtype=float)
            turn_probability = float(feat[0, 3] >= model.get("turn_curvature_threshold", 0.15))
        truth = data["future"][n]
        e = np.linalg.norm(trajectory - truth, axis=1)
        errors.extend(e); final_errors.append(e[-1]); turn_errors.append(abs(turn - data["turn_rate"][n]))
        covered.extend(e <= tube)
        is_risk = data["risk"][n] > 0.5
        # Risk uses the finite calibrated tube, while ADE/FDE remain center
        # trajectory metrics. This makes uncertainty actionable without
        # silently inflating the tube during evaluation.
        tube_scale = 0.0 if model is None else float(model.get("risk_tube_scale", 0.0))
        predicted_clearance = np.abs(trajectory[:, 1]) - 0.25 - tube_scale * tube
        predicted_risk = float(np.min(predicted_clearance)) <= 0.0
        warning_clearance = predicted_clearance - (0.0 if model is None else float(model.get("warning_margin_m", 0.0)))
        predicted_warning = float(np.min(warning_clearance)) <= 0.0
        risks.append((is_risk, predicted_risk))
        warning_events.append((is_risk, predicted_warning))
        actual_hits = np.flatnonzero(data["future_clearance"][n] <= 0.0)
        predicted_hits = np.flatnonzero(warning_clearance <= 0.0)
        if len(actual_hits) and len(predicted_hits):
            leads.append(float(actual_hits[0] - predicted_hits[0]) * 0.5)
        threshold = 0.5
        turn_pred.append(turn_probability >= threshold)
    risk_tp = sum(t and p for t, p in risks); risk_fn = sum(t and not p for t, p in risks)
    turn_tp = sum(t and p for t, p in zip(turn_true, turn_pred)); turn_n = sum(turn_true)
    false_alarm = sum((not t) and p for t, p in risks)
    safe_n = sum(not t for t, _ in risks)
    warning_tp = sum(t and p for t, p in warning_events)
    warning_fn = sum(t and not p for t, p in warning_events)
    warning_false_alarm = sum((not t) and p for t, p in warning_events)
    return {"ADE_m": float(np.mean(errors)), "FDE_m": float(np.mean(final_errors)),
            "turn_rate_MAE_rad_s": float(np.mean(turn_errors)),
            "uncertainty_coverage": float(np.mean(covered)),
            "collision_risk_recall": float(risk_tp / max(1, risk_tp + risk_fn)),
            "false_alarm_rate": float(false_alarm / max(1, safe_n)),
            "warning_recall": float(warning_tp / max(1, warning_tp + warning_fn)),
            "warning_false_alarm_rate": float(warning_false_alarm / max(1, safe_n)),
            "warning_lead_time_s": float(np.mean(leads)) if leads else None,
            "turn_recall": float(turn_tp / max(1, turn_n)), "samples": len(idx)}


def _episode_rows(data, model, episode_seeds: np.ndarray) -> list[dict]:
    rows = []
    for seed in episode_seeds:
        indices = np.flatnonzero(data["episode_seed"] == seed)
        metrics = _metrics(data, model, indices=indices)
        rows.append({"episode_seed": int(seed), "motion_kind": str(data["kind"][indices[0]]), **metrics})
    return rows


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", type=Path, default=Path("output/d1_imm_dataset.npz"))
    ap.add_argument("--model", type=Path, default=Path("output/d1_imm_model.json"))
    ap.add_argument("--output", type=Path, default=Path("output/d1_imm_training_summary.json"))
    args = ap.parse_args()
    raw = np.load(args.dataset, allow_pickle=True); data = {k: raw[k] for k in raw.files}
    model = json.loads(args.model.read_text())
    validation_seeds = np.unique(data["episode_seed"][data["split"] == "val"])
    if len(validation_seeds) < 30:
        raise ValueError(f"validation split has {len(validation_seeds)} episodes; regenerate with --episodes 200 for 30")
    validation_seeds = validation_seeds[:30]
    result = {
        "validation_episode_count": int(len(validation_seeds)),
        "heuristic": _metrics(data, split="val", indices=np.flatnonzero(np.isin(data["episode_seed"], validation_seeds))),
        "calibrated": _metrics(data, model, split="val", indices=np.flatnonzero(np.isin(data["episode_seed"], validation_seeds))),
        "validation_30": {
            "heuristic": _episode_rows(data, None, validation_seeds),
            "calibrated": _episode_rows(data, model, validation_seeds),
        },
    }
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
