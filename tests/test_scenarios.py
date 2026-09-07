"""Tests for the 8 deterministic software scenarios (no hardware needed)."""
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from python.prototype.scenarios import SCENARIOS, run_scenario


EXPECTED = [
    "urban_intersection", "highway", "village_road", "pedestrian_crossing",
    "cut_in", "stationary_obstacle", "tracking_loss", "degraded_sensing",
]


def test_all_eight_scenarios_exist():
    assert set(SCENARIOS.keys()) == set(EXPECTED)


@pytest.mark.parametrize("name", EXPECTED)
def test_scenario_no_collision_with_aeb(name):
    result = run_scenario(aeb_enabled=True, cfg=SCENARIOS[name])
    assert result["passed"], f"{name}: collided with {result['collision_with']}"
    assert result["collision"] is False


@pytest.mark.parametrize("name", EXPECTED)
def test_scenario_metrics_present(name):
    result = run_scenario(aeb_enabled=True, cfg=SCENARIOS[name])
    for key in ("min_clearance_m", "ttc_first_s", "stopping_distance_m",
                "response_time_s", "emergency_steps", "false_braking"):
        assert key in result
    assert result["min_clearance_m"] > 0
    assert result["false_braking"] is False


def test_aeb_improves_or_equals_clearance():
    for name, cfg in SCENARIOS.items():
        on = run_scenario(True, cfg)
        off = run_scenario(False, cfg)
        assert on["min_clearance_m"] >= off["min_clearance_m"], name


def test_no_false_braking_anywhere():
    for name, cfg in SCENARIOS.items():
        assert run_scenario(True, cfg)["false_braking"] is False, name


def test_deterministic_runs():
    """Same inputs twice -> identical metrics (no RNG in the pipeline)."""
    a = run_scenario(True, SCENARIOS["urban_intersection"])
    b = run_scenario(True, SCENARIOS["urban_intersection"])
    assert a == b


def test_tracking_loss_scenario_uses_dropout():
    obj = SCENARIOS["tracking_loss"]["objects"][0]
    assert obj["drop_after_s"] == 1.0


def test_degraded_sensing_limited_range():
    assert SCENARIOS["degraded_sensing"]["sense_range_m"] == 30.0
