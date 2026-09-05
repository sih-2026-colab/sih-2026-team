"""End-to-end pipeline: perception -> prediction -> evaluation -> results.

Generates a synthetic urban-intersection scene, runs it through the Python
pipeline, and writes a JSON report into results/. No MATLAB required.

Usage:
    python run_pipeline.py
"""
from __future__ import annotations

import json
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from python.perception import DetectedObject, PerceptionFrame, filter_by_confidence
from python.prediction import predict_frame, time_to_collision
from python.evaluation import detection_metrics, ade_fde


def build_urban_intersection_scene() -> PerceptionFrame:
    """Synthetic urban intersection scene (matches the demo)."""
    return PerceptionFrame(frame_id=0, objects=[
        DetectedObject(1, "car",        x=20.0, y=2.0,  speed=8.0),
        DetectedObject(2, "pedestrian", x=14.0, y=-1.0, speed=1.2),
        DetectedObject(3, "cow",        x=25.0, y=1.0,  speed=0.8),
    ])


def build_ground_truth() -> PerceptionFrame:
    """'Perfect' ground truth: same objects; one detection missed by the
    sensor and one phantom detection added, to exercise the metrics."""
    scene = build_urban_intersection_scene()
    truth = PerceptionFrame(frame_id=0, objects=list(scene.objects) + [
        DetectedObject(4, "cyclist", x=10.0, y=0.5, speed=3.0),
    ])
    return truth


def main() -> None:
    scene = build_urban_intersection_scene()
    truth = build_ground_truth()

    # --- perception stage ---
    detections = filter_by_confidence(scene, min_conf=0.0)

    # --- prediction stage ---
    trajectories = predict_frame(detections, horizon_s=3.0, dt=0.5)
    ttcs = {str(o.obj_id): time_to_collision(o, ego_x=0.0, ego_speed=10.0)
            for o in detections.objects}

    # --- evaluation stage ---
    det_scores = detection_metrics(detections, truth)

    # ADE/FDE: constant-velocity prediction vs. ground truth where the car
    # actually decelerates (speed drops to 6 m/s).
    car = detections.by_label("car")[0]
    predicted = predict_constant_velocity_traj(car, horizon_s=3.0, dt=0.5)
    actual = [(t, car.x + 6.0 * t, car.y) for t, _, _ in predicted]  # real: 6 m/s
    traj_err = ade_fde(predicted, actual)

    report = {
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "scene": "urban_intersection",
        "perception": {
            "num_detections": len(detections.objects),
            "labels": sorted({o.label for o in detections.objects}),
            "metrics_vs_truth": det_scores,
        },
        "prediction": {
            "model": "constant_velocity",
            "horizon_s": 3.0,
            "dt_s": 0.5,
            "time_to_collision_s": ttcs,
        },
        "evaluation": {
            "car_trajectory_error": traj_err,
        },
    }

    out = ROOT / "results" / f"pipeline_report_{datetime.now():%Y%m%d_%H%M%S}.json"
    out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    print(f"\nReport saved to: {out}")


def predict_constant_velocity_traj(obj: DetectedObject, horizon_s: float, dt: float):
    from python.prediction import predict_constant_velocity
    return predict_constant_velocity(obj, horizon_s, dt)


if __name__ == "__main__":
    main()
