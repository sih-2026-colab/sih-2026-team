"""Deterministic software scenarios for the AutoNex driving simulator.

Eight scenarios, no hardware required — all data is synthetic and every
run is reproducible (fixed scenes, no randomness). Uses the shared AEB
physics from python/prototype/aeb_sim.py and adds two sensor effects:

- ``drop_after_s``  — tracking loss: the object stops being reported
  after that time (tracker must hold its last-known state).
- ``sense_range_m`` — degraded sensing: objects beyond this range are
  invisible until the ego closes in.

Measured metrics per scenario:
collision, min_clearance_m, ttc_first_s, stopping_distance_m,
response_time_s, false_braking, emergency_steps.
"""
from __future__ import annotations

from copy import deepcopy
from math import hypot
from typing import Any, Dict, List, Optional

from python.prototype.aeb_sim import threat_ttc

HIT_DIST_M = 2.4
TTC_BRAKE_S = 2.0
A_BRAKE = 6.0
DT = 0.05

BASE = {
    "dt": DT,
    "ttc_brake_s": TTC_BRAKE_S,
    "a_brake": A_BRAKE,
    "hit_dist_m": HIT_DIST_M,
    "ego": {"x": 0.0, "y": 0.0, "vx": 10.0, "vy": 0.0, "length": 4.5, "width": 1.8},
}


def _scene(name, t_end, objects, **ego_overrides) -> Dict[str, Any]:
    cfg = deepcopy(BASE)
    cfg.update({"name": name, "t_end": t_end, "objects": objects})
    cfg["ego"].update(ego_overrides)
    return cfg


SCENARIOS: Dict[str, Dict[str, Any]] = {
    # 1. Urban intersection: car, pedestrian crossing, cow on the road.
    "urban_intersection": _scene("urban_intersection", 6.0, [
        {"id": 1, "label": "car", "x": 20.0, "y": 3.6, "vx": 8.0, "vy": 0.0},
        {"id": 2, "label": "pedestrian", "x": 16.0, "y": -2.4, "vx": 0.0, "vy": 1.5},
        {"id": 3, "label": "cow", "x": 30.0, "y": 0.15, "vx": 0.4, "vy": 0.0},
    ]),
    # 2. Highway: fast lead car closing slowly, ego at 30 m/s.
    "highway": _scene("highway", 8.0, [
        {"id": 1, "label": "car", "x": 22.0, "y": 0.0, "vx": 20.0, "vy": 0.0},
    ], vx=30.0),
    # 3. Village road: animal steps onto a narrow rural road.
    "village_road": _scene("village_road", 6.0, [
        {"id": 1, "label": "cow", "x": 15.0, "y": 1.2, "vx": 0.2, "vy": -0.6},
    ], vx=9.0),
    # 4. Pedestrian crossing: pedestrian enters the lane ahead.
    "pedestrian_crossing": _scene("pedestrian_crossing", 6.0, [
        {"id": 1, "label": "pedestrian", "x": 14.0, "y": -3.0, "vx": 0.0, "vy": 1.2},
    ]),
    # 5. Cut-in: vehicle merges into the ego lane from the left.
    "cut_in": _scene("cut_in", 6.0, [
        {"id": 1, "label": "car", "x": 18.0, "y": 3.2, "vx": 11.0, "vy": -0.9},
    ], vx=12.0),
    # 6. Stationary obstacle: stalled vehicle in the lane.
    "stationary_obstacle": _scene("stationary_obstacle", 6.0, [
        {"id": 1, "label": "car", "x": 18.0, "y": 0.0, "vx": 0.0, "vy": 0.0},
    ]),
    # 7. Tracking loss: pedestrian is dropped from tracking after 1.0 s.
    "tracking_loss": _scene("tracking_loss", 6.0, [
        {"id": 1, "label": "pedestrian", "x": 14.0, "y": 0.0, "vx": 0.0, "vy": 0.0,
         "drop_after_s": 1.0},
    ]),
    # 8. Degraded sensing: sensor range limited to 30 m.
    "degraded_sensing": _scene("degraded_sensing", 6.0, [
        {"id": 1, "label": "cow", "x": 18.0, "y": 0.1, "vx": 0.0, "vy": 0.0},
    ], vx=10.0),
}
SCENARIOS["degraded_sensing"]["sense_range_m"] = 30.0


def run_scenario(aeb_enabled: bool, cfg: Optional[Dict[str, Any]] = None
                 ) -> Dict[str, Any]:
    """Run one deterministic scenario and return measured metrics."""
    scene = deepcopy(cfg or SCENARIOS["urban_intersection"])
    dt = float(scene["dt"])
    ego = dict(scene["ego"])
    objects = [dict(o) for o in scene["objects"]]
    hit = float(scene["hit_dist_m"])
    ttc_lim = float(scene["ttc_brake_s"])
    sense_range = scene.get("sense_range_m")

    mode = "CRUISE"
    collision_with: Optional[str] = None
    brake_onset_t: Optional[float] = None
    first_threat_t: Optional[float] = None
    min_clearance = float("inf")
    ttc_first: Optional[float] = None
    t = 0.0
    emergency_steps = 0
    x_at_brake: Optional[float] = None

    while t <= float(scene["t_end"]) + 1e-9:
        # Sensor effects
        visible = []
        for o in objects:
            if o.get("drop_after_s") is not None and t > float(o["drop_after_s"]):
                continue  # tracking lost
            if sense_range is not None and (o["x"] - ego["x"]) > sense_range:
                continue  # out of degraded range
            visible.append(o)

        # Ground-truth clearance (safety metric uses real positions)
        for o in objects:
            min_clearance = min(min_clearance, hypot(ego["x"] - o["x"],
                                                     ego["y"] - o["y"]))

        ttc, threat = threat_ttc(ego, visible, dt=dt, hit_dist_m=hit)
        if ttc is not None and (ttc_first is None or ttc < ttc_first):
            ttc_first = ttc
        if ttc is not None and ttc < ttc_lim * 2.5 and first_threat_t is None:
            first_threat_t = t

        if aeb_enabled and ttc is not None and ttc < ttc_lim:
            if mode != "BRAKE":
                mode = "BRAKE"
                brake_onset_t = t
                x_at_brake = ego["x"]
            emergency_steps += 1
        ax = -float(scene["a_brake"]) if mode == "BRAKE" else 0.0

        for o in objects:
            if hypot(ego["x"] - o["x"], ego["y"] - o["y"]) <= hit:
                collision_with = str(o.get("label", o.get("id")))
        if collision_with:
            break

        ego["vx"] = max(0.0, ego["vx"] + ax * dt)
        ego["x"] += ego["vx"] * dt
        ego["y"] += ego["vy"] * dt
        for o in objects:
            o["x"] += float(o.get("vx", 0.0)) * dt
            o["y"] += float(o.get("vy", 0.0)) * dt
        t += dt

    stopping_distance = (ego["x"] - x_at_brake) if x_at_brake is not None else None
    response_time = ((brake_onset_t - first_threat_t)
                     if brake_onset_t is not None and first_threat_t is not None else None)
    # False braking: braked although the object never actually threatened —
    # i.e. closest approach exceeded what braking distance plus a healthy
    # margin would require. Braking early is correct AEB, not a false brake.
    comfortable_gap = (stopping_distance + 2.0 * hit) if stopping_distance is not None else 4.0 * hit
    false_braking = bool(brake_onset_t is not None
                         and min_clearance > comfortable_gap)
    return {
        "scenario": scene["name"],
        "aeb_enabled": aeb_enabled,
        "passed": collision_with is None,
        "collision": collision_with is not None,
        "collision_with": collision_with,
        "min_clearance_m": round(min_clearance, 3) if min_clearance != float("inf") else None,
        "ttc_first_s": round(ttc_first, 3) if ttc_first is not None else None,
        "stopping_distance_m": round(stopping_distance, 3) if stopping_distance is not None else None,
        "response_time_s": round(response_time, 3) if response_time is not None else None,
        "emergency_steps": emergency_steps,
        "false_braking": false_braking,
        "final_speed_mps": round(ego["vx"], 3),
    }
