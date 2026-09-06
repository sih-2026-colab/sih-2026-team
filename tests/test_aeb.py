"""AEB prototype: collision without braking, stop with braking."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from python.prototype.aeb_sim import simulate, threat_ttc, SCENE


def test_aeb_off_hits_pedestrian():
    out = simulate(False)
    assert out["collision"] is True
    assert out["collision_with"] == "pedestrian"


def test_aeb_on_avoids_collision():
    out = simulate(True)
    assert out["collision"] is False
    assert out["final_speed"] == 0.0
    assert out["final_x"] < 16.0
    assert out["final_mode"] == "BRAKE"


def test_initial_threat_is_pedestrian():
    ttc, label = threat_ttc(SCENE["ego"], SCENE["objects"], hit_dist_m=SCENE["hit_dist_m"])
    assert label == "pedestrian"
    assert ttc is not None
    assert ttc < SCENE["ttc_brake_s"]
