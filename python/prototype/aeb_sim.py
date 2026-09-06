"""Urban-intersection AEB prototype (shared physics for preview + tests).

This is the live-loop logic that MATLAB/Simulink should run each timestep.
RoadRunner will later supply actor poses; until then the same scene is used.
"""
from __future__ import annotations

from copy import deepcopy
from math import hypot
from typing import Any, Dict, List, Optional, Tuple

# Keep these numbers identical in preview/index.html and matlab/scenarios/urban_intersection.m
SCENE: Dict[str, Any] = {
    "name": "urban_intersection",
    "dt": 0.05,
    "t_end": 4.0,
    "ttc_brake_s": 2.0,
    "a_brake": 6.0,
    "hit_dist_m": 2.4,
    "ego": {
        "x": 0.0,
        "y": 0.0,
        "vx": 10.0,
        "vy": 0.0,
        "length": 4.5,
        "width": 1.8,
    },
    "objects": [
        {"id": 1, "label": "car", "x": 20.0, "y": 3.6, "vx": 8.0, "vy": 0.0},
        {"id": 2, "label": "pedestrian", "x": 16.0, "y": -2.4, "vx": 0.0, "vy": 1.5},
        {"id": 3, "label": "cow", "x": 30.0, "y": 0.15, "vx": 0.4, "vy": 0.0},
    ],
}


def threat_ttc(
    ego: Dict[str, float],
    objects: List[Dict[str, float]],
    horizon_s: float = 3.0,
    dt: float = 0.05,
    hit_dist_m: float = 2.4,
) -> Tuple[Optional[float], Optional[str]]:
    """Time of first predicted close approach, assuming constant velocity."""
    best_t: Optional[float] = None
    best_label: Optional[str] = None
    steps = max(1, int(round(horizon_s / dt)))
    for obj in objects:
        for i in range(1, steps + 1):
            t = i * dt
            ex = ego["x"] + ego["vx"] * t
            ey = ego["y"] + ego["vy"] * t
            ox = obj["x"] + obj["vx"] * t
            oy = obj["y"] + obj["vy"] * t
            if hypot(ex - ox, ey - oy) <= hit_dist_m:
                if best_t is None or t < best_t:
                    best_t = t
                    best_label = str(obj.get("label", obj.get("id")))
                break
    return best_t, best_label


def _collided(ego: Dict[str, float], objects: List[Dict[str, float]], hit_dist_m: float) -> Optional[str]:
    for obj in objects:
        if hypot(ego["x"] - obj["x"], ego["y"] - obj["y"]) <= hit_dist_m:
            return str(obj.get("label", obj.get("id")))
    return None


def simulate(aeb_enabled: bool, scene: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    cfg = deepcopy(scene or SCENE)
    dt = float(cfg["dt"])
    ego = dict(cfg["ego"])
    objects = [dict(o) for o in cfg["objects"]]
    ttc_lim = float(cfg["ttc_brake_s"])
    a_brake = float(cfg["a_brake"])
    hit = float(cfg["hit_dist_m"])

    log: List[Dict[str, Any]] = []
    mode = "CRUISE"
    collision_with: Optional[str] = None
    t = 0.0

    while t <= cfg["t_end"] + 1e-9:
        ttc, threat = threat_ttc(ego, objects, dt=dt, hit_dist_m=hit)
        if aeb_enabled and ttc is not None and ttc < ttc_lim:
            mode = "BRAKE"
        ax = -a_brake if mode == "BRAKE" else 0.0
        collision_with = _collided(ego, objects, hit)
        log.append({
            "t": round(t, 4),
            "ego_x": ego["x"],
            "ego_y": ego["y"],
            "ego_vx": ego["vx"],
            "ttc": ttc,
            "threat": threat,
            "mode": mode,
            "objects": deepcopy(objects),
            "collision": collision_with,
        })
        if collision_with:
            break

        ego["vx"] = max(0.0, ego["vx"] + ax * dt)
        ego["x"] += ego["vx"] * dt
        ego["y"] += ego["vy"] * dt
        for obj in objects:
            obj["x"] += obj["vx"] * dt
            obj["y"] += obj["vy"] * dt
        t += dt

    return {
        "scene": cfg["name"],
        "aeb_enabled": aeb_enabled,
        "collision": collision_with is not None,
        "collision_with": collision_with,
        "final_mode": mode,
        "final_speed": ego["vx"],
        "final_x": ego["x"],
        "frames": log,
    }


def compare_demo() -> Dict[str, Any]:
    off = simulate(False)
    on = simulate(True)
    return {
        "aeb_off": {k: v for k, v in off.items() if k != "frames"},
        "aeb_on": {k: v for k, v in on.items() if k != "frames"},
        "off_frames": off["frames"],
        "on_frames": on["frames"],
    }


if __name__ == "__main__":
    demo = compare_demo()
    print("AEB OFF:", demo["aeb_off"])
    print("AEB ON :", demo["aeb_on"])
