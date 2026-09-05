"""Evaluation: metrics for detection & prediction quality.

Implements standard metrics that work on the lightweight dataclasses so
they can be unit-tested without datasets:
- detection: precision / recall / F1 (IoU-free, label+position matching)
- prediction: ADE / FDE (average & final displacement error)
"""
from __future__ import annotations

from typing import Dict, List, Sequence, Tuple

from python.perception.detector import DetectedObject, PerceptionFrame

POS_TOL_M = 1.5  # meters; two detections "match" if closer than this


def match_detections(pred: Sequence[DetectedObject],
                     truth: Sequence[DetectedObject],
                     tol_m: float = POS_TOL_M) -> Tuple[int, int, int]:
    """Greedy nearest matching -> (tp, fp, fn)."""
    used_truth = set()
    tp = 0
    for p in pred:
        best, best_d = None, tol_m
        for i, t in enumerate(truth):
            if i in used_truth or p.label != t.label:
                continue
            d = ((p.x - t.x) ** 2 + (p.y - t.y) ** 2) ** 0.5
            if d <= best_d:
                best, best_d = i, d
        if best is not None:
            used_truth.add(best)
            tp += 1
    fp = len(pred) - tp
    fn = len(truth) - tp
    return tp, fp, fn


def detection_metrics(pred: PerceptionFrame, truth: PerceptionFrame) -> Dict[str, float]:
    tp, fp, fn = match_detections(pred.objects, truth.objects)
    precision = tp / (tp + fp) if (tp + fp) else 0.0
    recall = tp / (tp + fn) if (tp + fn) else 0.0
    f1 = 2 * precision * recall / (precision + recall) if (precision + recall) else 0.0
    return {"precision": precision, "recall": recall, "f1": f1}


def ade_fde(predicted: Sequence[Tuple[float, float, float]],
            ground_truth: Sequence[Tuple[float, float, float]]
            ) -> Dict[str, float]:
    """Average / Final Displacement Error between two trajectories
    [(t, x, y), ...] of equal length."""
    if not predicted or len(predicted) != len(ground_truth):
        raise ValueError("trajectories must be non-empty and equal length")
    errs = [((px - gx) ** 2 + (py - gy) ** 2) ** 0.5
            for (_, px, py), (_, gx, gy) in zip(predicted, ground_truth)]
    return {"ade": sum(errs) / len(errs), "fde": errs[-1]}
