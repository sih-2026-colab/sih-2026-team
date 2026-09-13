"""Generate visual output for the 8 validation scenarios (matplotlib).

Produces, in results/:
- scenario_trajectories.png   8 panels: ego/actor paths, AEB on vs off
- scenario_dashboard.png      metrics bar-chart dashboard
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")  # no display needed
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from python.prototype.scenarios import SCENARIOS, run_scenario

OUT = ROOT / "results"
OUT.mkdir(exist_ok=True)

COLORS = {"car": "#1f77b4", "pedestrian": "#d62728", "cow": "#8c564b",
          "animal": "#8c564b", "bike": "#ff7f0e"}


def trace(cfg, aeb):
    """Re-run a scenario capturing full x-vs-time trajectories (mirrors run_scenario physics)."""
    from copy import deepcopy
    scene = deepcopy(cfg)
    dt = float(scene["dt"])
    ego = dict(scene["ego"])
    objects = [dict(o) for o in scene["objects"]]
    hit = float(scene["hit_dist_m"]); ttc_lim = float(scene["ttc_brake_s"])
    sense_range = scene.get("sense_range_m")
    mode = "CRUISE"; t = 0.0
    ex, et, ov, modes = [], [], {o["id"]: ([], []) for o in objects}, []
    while t <= float(scene["t_end"]) + 1e-9:
        vis = [o for o in objects
               if not (o.get("drop_after_s") and t > o["drop_after_s"])
               and not (sense_range and o["x"] - ego["x"] > sense_range)]
        ttc, _ = __import__("python.prototype.aeb_sim", fromlist=["threat_ttc"]) \
            .threat_ttc(ego, vis, dt=dt, hit_dist_m=hit)
        if aeb and ttc is not None and ttc < ttc_lim:
            mode = "BRAKE"
        ax = -float(scene["a_brake"]) if mode == "BRAKE" else 0.0
        hitobj = next((o for o in objects
                       if (ego["x"]-o["x"])**2 + (ego["y"]-o["y"])**2 <= hit*hit), None)
        ex.append(ego["x"]); et.append(t); modes.append(mode)
        for o in objects:
            ov[o["id"]][0].append(o["x"]); ov[o["id"]][1].append(t)
        if hitobj:
            break
        ego["vx"] = max(0.0, ego["vx"] + ax * dt)
        ego["x"] += ego["vx"] * dt; ego["y"] += ego["vy"] * dt
        for o in objects:
            o["x"] += float(o.get("vx", 0)) * dt; o["y"] += float(o.get("vy", 0)) * dt
        t += dt
    return ex, et, ov, objects


# ---------- Figure 1: trajectories ----------
fig, axes = plt.subplots(2, 4, figsize=(20, 9))
fig.suptitle("AutoNex — 8 Deterministic Scenarios: AEB ON vs OFF (software-only simulation)",
             fontsize=15, fontweight="bold")
for ax, (name, cfg) in zip(axes.flat, SCENARIOS.items()):
    on_x, on_t, ov, objs = trace(cfg, aeb=True)
    off_x, off_t, _, _ = trace(cfg, aeb=False)
    ax.plot(off_t, off_x, "--", color="gray", lw=1.5, label="ego AEB OFF")
    ax.plot(on_t, on_x, "-", color="green", lw=2.2, label="ego AEB ON")
    for o in objs:
        c = COLORS.get(o.get("label", ""), "black")
        ax.plot(ov[o["id"]][1], ov[o["id"]][0], "-", color=c, lw=1.3, alpha=0.85,
                label=o.get("label", ""))
    ax.set_title(name, fontsize=11, fontweight="bold")
    ax.set_xlabel("time (s)"); ax.set_ylabel("x position (m)")
    ax.legend(fontsize=7, loc="best"); ax.grid(alpha=0.3)
plt.tight_layout(rect=[0, 0, 1, 0.95])
fig.savefig(OUT / "scenario_trajectories.png", dpi=110)
plt.close(fig)

# ---------- Figure 2: metrics dashboard ----------
names = list(SCENARIOS.keys())
on = [run_scenario(True, SCENARIOS[n]) for n in names]
off = [run_scenario(False, SCENARIOS[n]) for n in names]

fig, axes = plt.subplots(2, 2, figsize=(16, 9))
fig.suptitle("AutoNex Validation Dashboard — AEB ON results per scenario",
             fontsize=15, fontweight="bold")

def bars(ax, vals_on, vals_off, title, ylabel):
    x = range(len(names))
    ax.bar([i - 0.2 for i in x], vals_off, width=0.4, label="AEB OFF", color="#c44e52")
    ax.bar([i + 0.2 for i in x], vals_on, width=0.4, label="AEB ON", color="#55a868")
    ax.set_xticks(list(x)); ax.set_xticklabels(names, rotation=30, ha="right", fontsize=8)
    ax.set_title(title); ax.set_ylabel(ylabel); ax.legend(fontsize=8); ax.grid(alpha=0.3, axis="y")

bars(axes[0][0], [r["min_clearance_m"] for r in on],
     [r["min_clearance_m"] for r in off], "Minimum clearance", "metres")
axes[0][0].axhline(2.4, color="red", ls="--", lw=1, label="collision (2.4 m)")
axes[0][0].legend(fontsize=8)

bars(axes[0][1], [r["final_speed_mps"] for r in on],
     [r["final_speed_mps"] for r in off], "Final ego speed", "m/s")

bars(axes[1][0], [r["stopping_distance_m"] or 0 for r in on],
     [0] * len(names), "Stopping distance (AEB ON)", "metres")

coll_on = [1 if r["collision"] else 0 for r in on]
coll_off = [1 if r["collision"] else 0 for r in off]
bars(axes[1][1], coll_on, coll_off, "Collision (0 = safe, 1 = crash)", "collision")
axes[1][1].set_yticks([0, 1]); axes[1][1].set_yticklabels(["safe", "collision"])

plt.tight_layout(rect=[0, 0, 1, 0.94])
fig.savefig(OUT / "scenario_dashboard.png", dpi=110)
plt.close(fig)

print("Saved:", OUT / "scenario_trajectories.png")
print("Saved:", OUT / "scenario_dashboard.png")
