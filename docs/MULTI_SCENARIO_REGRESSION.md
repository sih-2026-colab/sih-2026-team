# Multi-scenario closed-loop regression

Run `validate_autonex_scenarios` from the project root in MATLAB. It runs the full shared pipeline and saves per-scenario samples plus `results/multi_scenario_regression.json`. An optional cell array reruns named cases and replaces their rows in that summary. `multi_scenario_before_fusion_fix.json` preserves the initial sweep.

## Scope and identifiers

Shared pipeline: `missing_lane`, `cut_in`, `night_pedestrian`, `animal`, `degraded_sensing`, `pothole`, `tracking_loss`, `stationary_obstacle`, `clear_road`, `close_rear_cut_in`, `original_close_range`.

Legacy `missing_lanes` is accepted through the existing default highway setup. It is tested but is not the missing-lane/blocker scenario. Identical identifiers in Python do not imply identical scenes.

The separate Python AEB registry contains `urban_intersection`, `highway`, `village_road`, `pedestrian_crossing`, `cut_in`, `stationary_obstacle`, `tracking_loss`, `degraded_sensing`. `matlab/scenarios/urban_intersection.m` is an AEB scene fixture. `scenario_pedestrian(t)` is a standalone scripted trajectory helper. None of these constitutes integration with the shared steering/corridor/bicycle pipeline. RoadRunner JSON descriptions are not native executable RoadRunner scenes.

## SIH coverage

- Unmarked road: `missing_lane`, including both staggered blockers.
- Highway cut-in: `cut_in`; close-rear variants remain explicit stress cases.
- Pedestrian: `night_pedestrian` provides a scripted crossing, not a sudden occluded-emergence trigger.
- Animal: `animal` is a slow animal ahead with zero lateral velocity. Crossing exists only in the separate `village_road` AEB fixture; full-pipeline crossing is missing.
- Degraded sensing: `degraded_sensing`; stream failure: `tracking_loss`.
- Unsignalized intersection: full-pipeline topology, crossing traffic and right-of-way interaction are missing; an AEB fixture exists.
- Dense market/mixed traffic: full-pipeline scenario missing.

## Metric definitions and gates

`averageSpeed` is forward distance divided by actual configured simulation duration, in m/s. `finalSpeedKmh` and `maximumSpeedKmh` are km/h. Steering is degrees, steering rate degrees/s, clearance and lateral error metres. Missing-lane duration is 25 s; other cases run 10 s to expose delayed interactions.

Physical consistency checks enforce the existing 75 km/h highest speed candidate, +1.2/-6 m/s^2 acceleration bounds, no reverse speed, the exact speed update and midpoint bicycle position integration. These are verification bounds, not newly tuned control settings.

Collision count is the number of sampled overlap frames, not the number of separate crashes. Boundary violations are also sampled counts. Collision geometry uses conservative bounds of the yawed ego footprint. Replan count is selected-name changes; it is not the number of 20 Hz planner executions. Cat Reflex activations count entries into braking/emergency/guardian-intervention episodes, not routine CRUISE calls.

Steering-limit sample counts include endpoints; saturation time uses actual intervals, so the final endpoint contributes zero duration. Context is classified from braking/guardian state and lateral target offset. Saturation alone is not a failure. Tracking error is measured against the previous commanded path, not a fixed global reference. The reported NaN/Inf flag checks vehicle/control state; unavailable-corridor NaN sentinels are expected and are not vehicle-state failures.

Failure gates include collisions, road departures, nonfinite states, inconsistent physical motion, lost emergency authority, insufficient progress (unless emergency stopping), sustained steering reversals, RMS tracking error above 0.10 m, and lateral acceleration exceeding the existing guardian's 3.5 m/s^2 bound. Every scenario also has a capability-specific response gate. Minimum clearances remain visible; a noncollision pass is not real-world safety certification.

## Targeted correction and remaining failures

The initial sweep exposed unchanged uncertainty and shrinking safety margins when sensor quality fell. `adaptive_multimodal_fusion` now accounts for fractional modality-health loss using the existing missing-modality uncertainty penalty and caps degraded confidence against the matched nominal reference. Nominal-health behavior, controller tuning, Cat Reflex and corridor stabilization are unchanged. `test_degraded_fusion_safety` covers partial/missing modalities and loss of health.

The three rear-impact failures remain unmodified and visible in the summary, with first collision time and actor ID. Scripted rear traffic keeps its constant speed as ego brakes/stops. The current fallback does not prevent those rear impacts. Shortening simulation or moving rear traffic would hide the failure and is not an accepted fix.
