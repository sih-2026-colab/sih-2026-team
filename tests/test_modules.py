"""Unit tests for perception, prediction and evaluation modules."""
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from python.perception import DetectedObject, PerceptionFrame, filter_by_confidence
from python.prediction import predict_constant_velocity, predict_frame, time_to_collision
from python.evaluation import detection_metrics, ade_fde


def make_frame():
    return PerceptionFrame(frame_id=1, objects=[
        DetectedObject(1, "car", 20.0, 2.0, speed=8.0),
        DetectedObject(2, "pedestrian", 14.0, -1.0, speed=1.2),
        DetectedObject(3, "cow", 25.0, 1.0, speed=0.8, confidence=0.3),
    ])


# ---------- perception ----------

def test_by_label():
    frame = make_frame()
    assert len(frame.by_label("car")) == 1
    assert frame.by_label("cow")[0].obj_id == 3


def test_nearest():
    frame = make_frame()
    assert frame.nearest(21, 2).label == "car"
    assert frame.nearest(13.5, -1).label == "pedestrian"
    assert PerceptionFrame(0).nearest(0, 0) is None


def test_filter_by_confidence():
    kept = filter_by_confidence(make_frame(), 0.5)
    assert len(kept.objects) == 2
    assert all(o.confidence >= 0.5 for o in kept.objects)


# ---------- prediction ----------

def test_constant_velocity():
    car = DetectedObject(1, "car", 20.0, 2.0, speed=8.0)
    traj = predict_constant_velocity(car, horizon_s=2.0, dt=0.5)
    assert len(traj) == 4
    t3, x3, y3 = traj[2]          # t = 1.5 s (dt=0.5 → index 2)
    assert t3 == pytest.approx(1.5)
    assert x3 == pytest.approx(32.0)   # 20 + 8*1.5
    assert y3 == pytest.approx(2.0)


def test_predict_frame_covers_all_objects():
    preds = predict_frame(make_frame(), horizon_s=1.0, dt=0.5)
    assert set(preds.keys()) == {"1", "2", "3"}


def test_ttc():
    car = DetectedObject(1, "car", 40.0, 0.0, speed=-10.0)  # heading toward ego
    assert time_to_collision(car, ego_x=0, ego_speed=0) == pytest.approx(4.0)
    assert time_to_collision(DetectedObject(2, "car", 40, 0, speed=10), 0, 0) is None


# ---------- evaluation ----------

def test_detection_metrics_perfect():
    truth = make_frame()
    score = detection_metrics(truth, truth)
    assert score["precision"] == 1.0 and score["recall"] == 1.0 and score["f1"] == 1.0


def test_detection_metrics_missed_and_false():
    truth = make_frame()
    pred = PerceptionFrame(1, objects=[truth.objects[0],        # correct
                                       DetectedObject(9, "car", 100, 100)])  # false positive
    score = detection_metrics(pred, truth)
    assert score["recall"] == pytest.approx(1 / 3)
    assert score["precision"] == pytest.approx(0.5)


def test_ade_fde():
    pred = [(0, 0, 0), (1, 10, 0)]
    truth = [(0, 0, 0), (1, 12, 0)]
    m = ade_fde(pred, truth)
    assert m["ade"] == pytest.approx(1.0) and m["fde"] == pytest.approx(2.0)


def test_ade_fde_length_mismatch():
    with pytest.raises(ValueError):
        ade_fde([(0, 0, 0)], [(0, 0, 0), (1, 1, 1)])
