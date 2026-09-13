# AutoNex closed-loop ego motion

## Architecture and reuse

The existing loop is retained:

`autonex_step` observes the current actors, updates tracking/uncertainty and
safe-space/corridor estimates, generates and scores candidates, applies
selection hysteresis and Cat Reflex/rear-risk gates, and passes the selected
trajectory to the existing Pure Pursuit `path_following_controller`.
`speed_tracking_controller` supplies longitudinal acceleration. The existing
`update_ego_bicycle` advances ego while `update_highway_actors` advances the
environment. The returned actors, yaw and steering are inputs to the next
`autonex_step`. An empty approved path commands bounded braking, not a
fabricated escape path. Existing rear mitigation can supply a path only through
its existing safety gates.

The planner, Pure Pursuit algorithm, dynamic safety bubble, uncertainty,
prediction, Cat Reflex, candidate scoring and corridor logic were not replaced.
Existing root and nested MATLAB entry points receive matching connection fixes.

## Targeted changes

- Preserve applied acceleration and heading when writing the integrated ego
  actor back. Previously the pre-command actor overwrote acceleration fields.
- Return `state.ego` and `out.nextEgo`, with x/y in metres, yaw/steering in radians,
  speed in m/s and acceleration in m/s^2. `state.actors(1)` remains the planner's
  authoritative actor input; `state.ego` is its explicit motion-state view,
  not an independently writable second input.
- Existing `out.speed` remains km/h for compatibility. Samples describe the
  pre-integration instant; `nextEgo` describes the end of that interval.
- `run_autonex_simulation` additionally returns `finalEgo` and
  `integratedThroughTime`. Its legacy inclusive time loop is unchanged: a final
  sample at duration advances the state to duration + dt.
- Centralize the existing longitudinal limits in `autonex_options`:
  maxAcceleration 1.2, maxDeceleration 6, controlledDeceleration 3.5,
  microDeceleration 1.5 (positive magnitudes, m/s^2). Both ordinary control and
  rear-mitigation rollouts receive the same options; the plant also clips inputs.
- Preserve existing 30-degree steering and 45-degree/s slew limits. Pure
  Pursuit enforces slew (including safety overrides with the default physical
  mode); the plant additionally enforces steering magnitude.
- Braking uses scalar speed rather than the sign of world-frame vx. If braking
  reaches zero within a timestep, integrate only the moving portion and do not
  reverse or drift after stopping. Scenario physical-consistency assertions
  were updated to independently account for this fractional stop interval.
- Add motionMode telemetry: CRUISE, REPLAN on a selection switch, BRAKE for
  negative acceleration, STOP when an emergency has stopped ego. This is
  reporting, not a replacement decision manager; each cycle still evaluates
  regenerated candidates through existing safety/selection gates.

## Reproduce in MATLAB R2026a

From the AutoNex project root:

```matlab
startup
test_closed_loop_motion
evidence = run_closed_loop_demo();
regressionResults = run_motion_regressions();
assert(all([regressionResults.passed]));
checks = validate_rear_mitigation_regression();
assert(all([checks.passed]));
validate_closed_loop_path_following();
```

The new test covers straight/curved tracking, angle and slew saturation,
configurable acceleration/braking limits, partial-step stopping, braking with
negative world-frame vx, moving-ego trajectory changes, no-safe-path braking,
and exact state feedback/path origins on subsequent planning cycles.

The headless demo uses the real planner for clear_road and missing_lane. Its
unavailable_corridor case injects unavailable observed space after 0.5 seconds
to exercise the real empty-candidate braking branch. That injection is a test
fixture, not a new planner or a claim of complete perception-failure coverage.
Each case executes 161 steps at dt=0.05 seconds.
The unavailable-corridor fixture checks braking and state feedback, not
collision avoidance: its other scripted vehicles do not react to a stopped
ego. Collision/boundary observations are printed separately, never treated as
proof of safety by the motion-pass marker.

`run_motion_regressions` executes all root test_*.m and tests/test*.m entries and
records errors without abandoning later tests. Some legacy entries are visual
smoke scripts rather than assertion-based safety tests. Full-scenario safety
results must be read separately; a script that executes is not proof of safety.

## Limitations

This remains a kinematic simulation, not real-vehicle validation: no tyre slip,
actuator lag, braking jerk or road-friction model has been added. The direct
bicycle function assumes slew has already been enforced by its caller; the
shared loop does so. Disabling the existing enforcePhysicalSteeringRate option
retains its legacy counterfactual behavior and is not the validated mode.
Candidate prediction/risk thresholds retain their existing nominal assumptions;
changing actuator configuration substantially needs scenario revalidation.
The Simulink adapter is unchanged; MATLAB multi-step evidence does not itself
certify Simulink deployment or real-time execution. This task does not add or
retrain any AI models, or add perception/UI integrations.

Test-run results are recorded in the accompanying console logs and final report.

## Source-file inventory

Modified (existing user changes preserved):

- autonex_options.m
- speed_tracking_controller.m
- update_ego_bicycle.m
- autonex_step.m and matlab/core/autonex_step.m
- evaluate_minimum_risk_action.m and matlab/planning/evaluate_minimum_risk_action.m
- run_autonex_simulation.m
- validate_autonex_scenarios.m (independent fractional-stop physics assertion)

Created:

- run_closed_loop_demo.m
- tests/test_closed_loop_motion.m
- tests/run_motion_regressions.m
- docs/CLOSED_LOOP_MOTION.md

Inspected/reused without redesign: path_following_controller.m,
update_highway_actors.m, create_highway_scenario.m,
configure_autonex_scenario.m, generate_corridor_candidates.m,
stabilize_trajectory_selection.m, cat_reflex_guardian.m,
autonex_simulink_step.m, autonex_sfun.m, existing component tests and
validate_closed_loop_path_following/validate_corridor_stability/
validate_rear_mitigation_regression/validate_steering_actuator entry points.

## Verified motion and component results

MATLAB R2026a executed the new suite and all 36 pre-existing test entry points:
`REGRESSION_TOTAL passed=37 failed=0 total=37` (includes the new test).

- Straight: 100 steps, 20.000 m travel.
- Radius-20 m curve: 140 steps, maximum radial error 0.0190 m, yaw 1.400 rad.
- Steering angle/slew saturation, custom acceleration/deceleration limits,
  exact fractional-step stopping and negative-world-vx braking: passed.
- Real-planner clear_road: 161 steps, x=167.708 m, speed=20.833 m/s.
- Real-planner missing_lane: 161 steps, x=32.429 m, y=3.195 m,
  speed=5.872 m/s, 39 moving trajectory-name changes.
- Unavailable corridor: braking at t=0.50 s; STOP by t=4.00 s;
  x=46.586 m, speed=0.000 m/s. No selected trajectory after injection.
- All three demo runs: exact next-cycle state feedback passed, zero observed
  collision steps and zero road-boundary violations over their test windows.

Console evidence: motion_regression_console.log and closed_loop_final_console.log.
Full-scenario regression results are recorded separately in motion_scenario_console.log.
