"""Prediction: constant-velocity trajectory prediction.

Predicts future positions of detected objects assuming constant velocity —
a standard baseline before learned predictors (LSTM/Transformer) are added.
"""
from __future__ import annotations

from typing import List, Tuple

from python.perception.detector import DetectedObject, PerceptionFrame


def predict_constant_velocity(obj: DetectedObject,
                              horizon_s: float = 3.0,
                              dt: float = 0.5) -> List[Tuple[float, float, float]]:
    """Predict (t, x, y) positions for the next `horizon_s` seconds.

    Assumes the object keeps its current heading (estimated from position
    along the road, i.e. motion along +x) and constant speed.
    """
    traj = []
    steps = max(1, int(round(horizon_s / dt)))
    for i in range(1, steps + 1):
        t = i * dt
        traj.append((round(t, 3), obj.x + obj.speed * t, obj.y))
    return traj


def predict_frame(frame: PerceptionFrame,
                  horizon_s: float = 3.0,
                  dt: float = 0.5) -> dict:
    """Predict trajectories for every object in a perception frame."""
    return {
        str(o.obj_id): predict_constant_velocity(o, horizon_s, dt)
        for o in frame.objects
    }


def time_to_collision(obj: DetectedObject, ego_x: float = 0.0,
                      ego_speed: float = 0.0) -> float | None:
    """Rough TTC with an ego vehicle at ego_x moving at ego_speed.

    Returns None when the closing speed is ~0 (no collision course).
    """
    gap = obj.x - ego_x
    closing = ego_speed - obj.speed  # positive: object approaches ego
    if closing <= 1e-6 or gap <= 0:
        return None
    return gap / closing
