#!/usr/bin/env python3
"""Fit lightweight, interpretable IMM mode and turn-rate calibration."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import numpy as np


KINDS = ("static", "crossing", "same_direction", "accelerating", "turning", "irregular")


def _summary(data: dict[str, np.ndarray]) -> np.ndarray:
    v, a = data["velocity"], data["acceleration"]
    speed = np.linalg.norm(v, axis=1)
    accel = np.linalg.norm(a, axis=1)
    cross = v[:, 0] * a[:, 1] - v[:, 1] * a[:, 0]
    curvature = cross / np.maximum(speed * speed, 1e-4)
    confidence = np.asarray(data.get("confidence", np.zeros(len(v))), dtype=float)
    miss_count = np.asarray(data.get("miss_count", np.zeros(len(v))), dtype=float)
    return np.column_stack((np.ones(len(v)), speed, accel, curvature, np.abs(curvature),
                            confidence, miss_count, data["noise"], data["dropout"]))


def _softmax(z: np.ndarray) -> np.ndarray:
    z = z - np.max(z, axis=1, keepdims=True)
    e = np.exp(z)
    return e / np.sum(e, axis=1, keepdims=True)


def _residual_features(summary: np.ndarray, velocity: np.ndarray,
                       acceleration: np.ndarray) -> np.ndarray:
    return np.column_stack((summary, velocity, acceleration))


def calibrated_prediction(position: np.ndarray, velocity: np.ndarray, acceleration: np.ndarray,
                          feature_row: np.ndarray, model: dict) -> tuple[np.ndarray, dict[str, float], float]:
    """Return calibrated CV/CA/CTRV mixture and learned motion-family scores."""
    family_values = _softmax(feature_row.reshape(1, -1) @ np.asarray(model["mode_weights"]))[0]
    family = dict(zip(KINDS, family_values))
    weights = {"CV": family["static"] + family["crossing"] + family["same_direction"],
               "CA": family["accelerating"], "CTRV": family["turning"] + family["irregular"]}
    curvature = float(feature_row[3])
    turn = float(np.array([1.0, curvature]) @ np.asarray(model["turn_rate_weights"]))
    times = np.array([0.5, 1.0, 2.0, 3.0])
    p, v, a = np.asarray(position), np.asarray(velocity), np.asarray(acceleration)
    cv = p + times[:, None] * v
    ca = cv + 0.5 * times[:, None] ** 2 * a
    speed = float(np.linalg.norm(v))
    if speed < 1e-6 or abs(turn) < 1e-4:
        ctrv = cv
    else:
        heading = float(np.arctan2(v[1], v[0]))
        angles = heading + turn * times
        ctrv = p + speed / turn * np.column_stack((np.sin(angles) - np.sin(heading),
                                                     -np.cos(angles) + np.cos(heading)))
    trajectory = np.sum(np.array([weights["CV"], weights["CA"], weights["CTRV"]])[:, None, None]
                        * np.stack((cv, ca, ctrv)), axis=0)
    residual_weights = model.get("residual_weights")
    if residual_weights is not None:
        residual_input = np.concatenate((feature_row, v, a))
        mean = np.asarray(model["residual_feature_mean"], dtype=float)
        scale = np.asarray(model["residual_feature_scale"], dtype=float)
        correction = ((residual_input - mean) / scale) @ np.asarray(residual_weights)
        trajectory = trajectory + float(model.get("residual_scale", 1.0)) * correction.reshape(4, 2)
    return trajectory, family, turn


def train(dataset: Path, output: Path, epochs: int = 500, lr: float = 0.08) -> dict:
    raw = np.load(dataset, allow_pickle=True)
    data = {k: raw[k] for k in raw.files}
    mask = data["split"] == "train"
    summary_keys = ("velocity", "acceleration", "noise", "dropout", "confidence", "miss_count")
    x = _summary({k: data[k][mask] for k in summary_keys})
    y = np.array([KINDS.index(k) for k in data["kind"][mask]])
    w = np.zeros((x.shape[1], len(KINDS)))
    for _ in range(epochs):
        p = _softmax(x @ w)
        grad = x.T @ (p - np.eye(len(KINDS))[y]) / len(x)
        w -= lr * grad
    turn_x = np.column_stack((np.ones(len(x)), x[:, 3]))
    turn_y = data["turn_rate"][mask]
    turn_w = np.linalg.lstsq(turn_x, turn_y, rcond=None)[0]
    model = {"version": 3, "features": ["bias", "speed", "accel_norm", "signed_curvature", "abs_curvature", "confidence", "miss_count", "noise", "dropout"],
             "motion_kinds": list(KINDS), "mode_weights": w.tolist(),
             "turn_rate_weights": turn_w.tolist(),
             "training_samples": int(len(x)),
             "epochs": epochs, "learning_rate": lr}
    train_v, train_a = data["velocity"][mask], data["acceleration"][mask]
    residual_x = _residual_features(x, train_v, train_a)
    residual_mean = np.mean(residual_x, axis=0)
    residual_scale = np.std(residual_x, axis=0)
    residual_scale = np.maximum(residual_scale, 1e-6)
    residual_z = (residual_x - residual_mean) / residual_scale
    train_positions, train_futures = data["position"][mask], data["future"][mask]
    base_residuals = []
    for i in range(len(x)):
        trajectory, _, _ = calibrated_prediction(train_positions[i], train_v[i], train_a[i], x[i], model)
        base_residuals.append((train_futures[i] - trajectory).reshape(-1))
    base_residuals = np.asarray(base_residuals)
    ridge = 0.10 * np.eye(residual_z.shape[1])
    ridge[0, 0] = 1e-6
    residual_weights = np.linalg.solve(
        residual_z.T @ residual_z + ridge,
        residual_z.T @ base_residuals,
    )
    model["residual_feature_mean"] = residual_mean.tolist()
    model["residual_feature_scale"] = residual_scale.tolist()
    model["residual_weights"] = residual_weights.tolist()
    val_mask = data["split"] == "val"
    val_x = _summary({k: data[k][val_mask] for k in summary_keys})
    val_v, val_a = data["velocity"][val_mask], data["acceleration"][val_mask]
    val_positions, val_futures = data["position"][val_mask], data["future"][val_mask]
    val_z = (_residual_features(val_x, val_v, val_a) - residual_mean) / residual_scale
    val_correction = (val_z @ residual_weights).reshape(-1, 4, 2)
    val_base = []
    for i in range(len(val_x)):
        trajectory, _, _ = calibrated_prediction(
            val_positions[i], val_v[i], val_a[i], val_x[i],
            {key: value for key, value in model.items() if key not in {
                "residual_weights", "residual_feature_mean", "residual_feature_scale", "residual_scale",
            }},
        )
        val_base.append(trajectory)
    val_base = np.asarray(val_base)
    residual_scales = np.linspace(0.0, 1.25, 26)
    residual_scores = []
    for scale_value in residual_scales:
        prediction = val_base + scale_value * val_correction
        point_error = np.linalg.norm(prediction - val_futures, axis=2)
        residual_scores.append(0.5 * float(np.mean(point_error))
                             + 0.5 * float(np.mean(point_error[:, -1])))
    model["residual_scale"] = float(residual_scales[int(np.argmin(residual_scores))])
    residuals = []
    for i in range(len(x)):
        trajectory, _, _ = calibrated_prediction(train_positions[i], train_v[i], train_a[i], x[i], model)
        residuals.append(np.linalg.norm(trajectory - train_futures[i], axis=1))
    model["tube_radius_m"] = (np.quantile(np.asarray(residuals), 0.90, axis=0) + 0.02).tolist()
    val_family = _softmax(val_x @ w)
    val_turn = np.array([kind == "turning" for kind in data["kind"][val_mask]])
    val_speed = np.linalg.norm(data["velocity"][val_mask], axis=1)
    baseline_turn_rate = np.where(val_speed > 0.08, val_x[:, 3], 0.0)
    baseline_turn = np.abs(baseline_turn_rate) >= 0.15
    baseline_recall = float(np.sum(baseline_turn & val_turn) / max(1, np.sum(val_turn)))
    thresholds = np.linspace(-0.20, 0.60, 81)
    scores = []
    for threshold in thresholds:
        predicted = val_x[:, 3] >= threshold
        tp = np.sum(predicted & val_turn); fp = np.sum(predicted & ~val_turn); fn = np.sum(~predicted & val_turn)
        precision = tp / max(1, tp + fp); recall = tp / max(1, tp + fn)
        f1 = 2.0 * precision * recall / max(1e-9, precision + recall)
        scores.append(f1 if recall >= baseline_recall else -1.0)
    selected_threshold = thresholds[int(np.argmax(scores))]
    model["turn_curvature_threshold"] = float(selected_threshold)
    model["baseline_val_turn_recall"] = baseline_recall
    # Select a bounded uncertainty contribution for collision risk. The
    # center trajectory remains unchanged; false alarms are capped on val.
    risk_scale_candidates = np.linspace(0.0, 1.0, 21)
    risk_scores = []
    val_positions = data["position"][val_mask]
    val_velocities = data["velocity"][val_mask]
    val_accelerations = data["acceleration"][val_mask]
    val_tube = np.asarray(model["tube_radius_m"])
    for scale in risk_scale_candidates:
        predicted_risk = []
        for i in range(len(val_x)):
            trajectory, _, _ = calibrated_prediction(val_positions[i], val_velocities[i],
                                                     val_accelerations[i], val_x[i], model)
            clearance = np.abs(trajectory[:, 1]) - 0.25 - scale * val_tube
            predicted_risk.append(np.min(clearance) <= 0.0)
        predicted_risk = np.asarray(predicted_risk)
        true_risk = data["risk"][val_mask] > 0.5
        tp = np.sum(predicted_risk & true_risk); fp = np.sum(predicted_risk & ~true_risk)
        fn = np.sum(~predicted_risk & true_risk); safe = max(1, np.sum(~true_risk))
        false_alarm = fp / safe
        precision = tp / max(1, tp + fp); recall = tp / max(1, tp + fn)
        risk_scores.append(2.0 * precision * recall / max(1e-9, precision + recall)
                           if false_alarm <= 0.15 else -1.0)
    model["risk_tube_scale"] = float(risk_scale_candidates[int(np.argmax(risk_scores))])
    # Calibrate a separate, finite early-warning margin. It is not applied to
    # the collision-risk metric; it only lets the warning fire before the
    # predicted clearance reaches zero. Keep validation false alarms <= 15%.
    warning_margins = np.linspace(0.0, 0.30, 31)
    warning_scores = []
    selected_risk_scale = model["risk_tube_scale"]
    val_tube = np.asarray(model["tube_radius_m"])
    for margin in warning_margins:
        warning_leads = []
        warning_flags = []
        for i in range(len(val_x)):
            trajectory, _, _ = calibrated_prediction(val_positions[i], val_velocities[i],
                                                     val_accelerations[i], val_x[i], model)
            base_clearance = np.abs(trajectory[:, 1]) - 0.25 - selected_risk_scale * val_tube
            warning_clearance = base_clearance - margin
            warning_flags.append(np.min(warning_clearance) <= 0.0)
            actual_hits = np.flatnonzero(data["future_clearance"][val_mask][i] <= 0.0)
            predicted_hits = np.flatnonzero(warning_clearance <= 0.0)
            if len(actual_hits) and len(predicted_hits):
                warning_leads.append(float(actual_hits[0] - predicted_hits[0]) * 0.5)
        warning_flags = np.asarray(warning_flags)
        true_risk = data["risk"][val_mask] > 0.5
        false_alarm = np.sum(warning_flags & ~true_risk) / max(1, np.sum(~true_risk))
        recall = np.sum(warning_flags & true_risk) / max(1, np.sum(true_risk))
        lead = float(np.mean(warning_leads)) if warning_leads else -1.0
        warning_scores.append((lead + 0.02 * recall) if false_alarm <= 0.15 else -1e9)
    model["warning_margin_m"] = float(warning_margins[int(np.argmax(warning_scores))])
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(model, indent=2) + "\n")
    return model


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", type=Path, default=Path("output/d1_imm_dataset.npz"))
    ap.add_argument("--output", type=Path, default=Path("output/d1_imm_model.json"))
    args = ap.parse_args()
    model = train(args.dataset, args.output)
    print(f"wrote {args.output} train_samples={model['training_samples']}")


if __name__ == "__main__":
    main()
