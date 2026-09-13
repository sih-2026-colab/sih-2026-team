# Steering actuator feasibility acceptance envelope

Proposed before counterfactual testing: use `autonex_options.maxSteeringAngle` (30 degrees) and `maxSteeringRate` (45 degrees/s) as the physical bounds for every mode. At dt=0.05 s the maximum steering change is 2.25 degrees per step. No higher emergency rate is justified by a specified actuator. Guardian actions may bypass comfort shaping but cannot bypass physical slew. Emergency STOP requests neutral steering through the same actuator; no angle reset is instantaneous.

Observed cause: `autonex_step` passes safetyOverride for emergency OR guardian mode other than APPROVED. `path_following_controller` sets step=inf and uses the entire angle change immediately. The reported rate is (applied angle - previous applied angle)/dt and feeds the bicycle trajectory, so the spike is an actual simulated actuation jump. OVERRIDE_SAFE is sufficient; emergency braking need not be active.

Counterfactual protocol: keep the default behavior unchanged initially; opt into physical slew for full deterministic intersection and dense-market replays, including perception, tracking, prediction, guardian and replanning. Compare against saved full-run samples. Capture current scene/tracks and path at the original events by deterministic baseline replay. Adopt the physical constraint only after bounded collision/road/clearance checks pass. Retain raw requests and actuator saturation telemetry. Requested rates may exceed the envelope; applied rates may not.

Requested steering means the Pure Pursuit request after magnitude/comfort shaping and before actuator slew. Raw Pure Pursuit angle remains separately logged. Time at steering-rate limit counts actual integration intervals, excluding the terminal observation. These bounds are prototype configuration, not measured hardware certification.

## Counterfactual evidence and adoption

Both full 10-second counterfactual runs passed with applied slew <=45 deg/s, no collision/departure and finite states. Intersection clearance was 8.1445 m (104 emergency steps, 19.422 m distance); dense-market clearance was 1.4608 m (zero emergency steps, 18.733 m distance). The existing longitudinal response accommodated the constrained actuator without planner or Cat Reflex tuning.

Physical slew is now the default. `enforcePhysicalSteeringRate=false` is retained only to reproduce the archived unphysical baseline in the diagnostic replay; it is not an accepted safety configuration. Production and acceptance tests use true. Tests requiring the old guardian teleport were replaced with assertions of physical guardian slew. The six-scenario after-run and existing regressions are required before completion.

## Diagnostic units and event details

`guardian_steering_event_reconstruction.json` stores angles in radians, rates in rad/s, curvature in 1/m, distances in metres and time in seconds. Summary/comparison files report steering in degrees and degrees/s. Front TTC is an instantaneous longitudinal closing-time diagnostic for laterally overlapping actors; it is not a complete intersection collision predictor. Scene truth is used only for these diagnostics.

At intersection t=3.50 s, speed was 2.547 m/s, previous steering 21.205 degrees, request/applied steering 0.05473 degrees, and mode OVERRIDE_SAFE/CRUISE. Initial selected-path curvature was 0.143695 1/m; front actor was CROSSING-CAR, longitudinal clearance 10.545 m and approximate TTC 4.155 s.

At dense-market t=0.05 s, speed was 5.825 m/s, steering changed from -2.25 to -22.269 degrees under OVERRIDE_SAFE/CONTROLLED_BRAKE. At t=0.10 s, STOP requested neutral from -22.269 degrees under OVERRIDE_MINIMUM_RISK/EMERGENCY_BRAKE, producing the 445.379 deg/s peak. There was no selected path in that STOP step; curvature is explicitly unavailable.

## Final acceptance and regression evidence

All six requested scenarios passed the final configured actuator envelope with zero sampled collisions, zero road departures and finite states. The missing-lane 25 s baseline retained 291.684 m distance and 1.342 m clearance. All four new SIH smoke runs passed again. Prototype, actuator, trajectory-continuity and path-following component tests passed; corridor regression retained variation 4.5549 -> 3.2431 m with no collisions/departures.

Before/after metrics are in `results/steering_actuator_comparison.json`. Before metrics are immutable archived evidence; refreshing the smoke results must not relabel corrected runs as the old baseline. The original-event replay uses the archived reconstruction snapshots and explicitly enables legacy behavior for diagnostics only. Production acceptance rejects that behavior whenever it exceeds the physical envelope.

The original collision-free intersection/market runs used impossible steering jumps. Collision-free outcomes did not require those jumps in the tested physical replays: intersection progress was more conservative and longitudinal braking increased. These observations do not establish hardware or general-road safety. Known rear-impact cases and sensor occlusion remain outside this change.
