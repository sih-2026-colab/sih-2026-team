"""Run all 8 deterministic scenarios and write results/validation_summary.json.

Software-only: no sensors, no hardware, no datasets. Deterministic output.

Usage:
    python run_validation.py
"""
from __future__ import annotations

import json
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from python.prototype.scenarios import SCENARIOS, run_scenario


def main() -> None:
    summary = []
    for name, cfg in SCENARIOS.items():
        on = run_scenario(aeb_enabled=True, cfg=cfg)
        off = run_scenario(aeb_enabled=False, cfg=cfg)
        # AEB must not cause collisions, must not false-brake, and must help
        # (or at least not hurt) the closest approach vs AEB-off.
        passed = (on["passed"] and not on["false_braking"]
                  and on["min_clearance_m"] >= off["min_clearance_m"])
        summary.append({
            "scenario": name,
            "passed": passed,
            "aeb_on": on,
            "aeb_off": off,
        })

    report = {
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "software_only": True,
        "hardware_used": False,
        "deterministic": True,
        "num_scenarios": len(summary),
        "all_passed": all(s["passed"] for s in summary),
        "results": summary,
    }

    out = ROOT / "results" / "validation_summary.json"
    out.write_text(json.dumps(report, indent=2), encoding="utf-8")

    for s in summary:
        status = "PASS" if s["passed"] else "FAIL"
        on = s["aeb_on"]
        print(f"[{status}] {s['scenario']:<22} collision={on['collision']} "
              f"min_clearance={on['min_clearance_m']} m  "
              f"ttc={on['ttc_first_s']} s  stop={on['stopping_distance_m']} m  "
              f"resp={on['response_time_s']} s  false_brake={on['false_braking']}")
    print(f"\nAll passed: {report['all_passed']}")
    print(f"Report saved to: {out}")


if __name__ == "__main__":
    main()
