# AutoNex judge console — P2

From the repository root in MATLAB R2026a:

```matlab
startup
addpath('prototype')
app = run_autonex_judge_prototype('occluded_pedestrian');
```

START runs the existing pipeline; PAUSE toggles RESUME. RESET and scene changes
restart simulation time and the bounded event log. Closing deletes the timer.
0.5x changes display pacing only. Computation may make playback slower than
wall-clock real time. Default duration is 10 simulated seconds.

The P1 scene, world view, controls and large `motionMode` remain. The right
panel has Live, Paths, and Sensors + log tabs. Paths lists the best six planner
scores (lower is better), retaining the selected candidate if it falls outside
that set. Scroll horizontally for conflict counts and rejection reasons.
The selected path can be a guardian alternative or separate rear mitigation;
a minimum-risk selection does not imply that every planner gate passed.

## Sources and semantics

- Optional `explainabilityTelemetry` exposes already computed planner results,
  guardian results, fused records, ego state, environment and sensor settings.
  The runner enables this and existing `sensorTelemetry`. No model is rerun.
- Yellow dashed predictions use `State = [x vx y vy]` and the exact constant
  velocity mean used in `evaluate_2d_trajectories`, at 0/.5/1/1.5/2 seconds.
  This is the planner's forecast, even when its input tracker is IMM.
- Amber patches are one-standard-deviation positional covariance ellipses:
  `Pxy(h) = F(h) * StateCovariance * F(h)'`. No invented process noise is added;
  these are propagated filter covariance, not calibrated probability bounds
  or the planner's scalar future-uncertainty growth.
- Outlined safety ellipses call `calculate_contextual_safety_envelope` using
  actual fused uncertainty/confidence and relative motion at the current
  instant. Labels give longitudinal and lateral semi-axes, not a radius.
  Teal means outside the current envelope; red means inside it. Candidate
  evaluation still uses the original time-varying envelopes internally.
- The relevant risk track has smallest current normalized envelope separation.
  Its displayed TTC uses the guardian's h=0 longitudinal rule: lateral gap
  <=2.2 m, positive closing speed >.10 m/s, TTC = abs(dx)/closing speed.
  N/A means no eligible track/TTC; Inf in candidate guardian results means no
  finite predicted TTC. This is not a new crossing-pedestrian TTC algorithm.
- Physical minimum clearance, collision and boundary are `out` simulation-truth
  evaluation fields, visibly labelled separately from tracked risk.
- Cyan paths satisfy both `plannerResults.safe` and `guardianResults.safe`.
  Red paths fail at least one. Green is the actual final `out.selected` path.
  An emergency stop may have no selected trajectory. No missing candidate is
  invented; upstream occupancy-filtered paths are not exposed by core output.
- Planner score, proximityCost, intentCost, front/rear/side conflict counts,
  boundaryViolation and comfortSafe are copied from the evaluator. Guardian
  risk/TTC values are a separately labelled score family. Rear mitigation uses
  its own actual totalScore/rearRiskCost/comfortCost/progressCost. N/A is shown
  when the final action has no matching scored candidate.
- Sensor counts come from actual sensorFrame arrays. Class detection and fused
  class evidence remain distinct. Camera/depth/thermal LOS annotations call
  `autonex_actor_visibility` and are explicitly simulation-truth geometry,
  not sensor returns. Thermal is inactive in camera/radar/LiDAR mode. The LOS
  focus is the first pedestrian actor, labelled by name; fused evidence is
  class-level and does not claim that a truth actor ID is a track ID.
- The process strip summarizes actual outputs, not execution timing. Events
  are timestamped changes in LOS, detection, fused class, track count, risk,
  rejection count, guardian and motion mode. Only the last ten are retained.

Graphics are persistent lines and pooled actor/track patches, hidden and reused
when counts shrink. The high-water mark can grow when new tracks appear; it
does not grow per frame. No rendering calls consume random numbers or advance
simulation. The top-down axes use unequal display scaling to fill the panel;
coordinates remain in world metres.

## Verification

```matlab
test_autonex_judge_p2
```

This checks both requested scenarios, every-step output equivalence against
telemetry-disabled core, real score/status bindings, covariance/forecast
geometry, RNG preservation, handle reuse and event bounds. It then invokes
unchanged actor visibility, occluded pedestrian, intersection, steering,
trajectory continuity and closed-loop motion tests. Evidence is saved under
`results/judge_p2*`, with the run log in `judge_p2_validation.log`.

This is synthetic simulation explainability, not a photoreal camera or a
claim of road deployment readiness. No video, planner tuning, or sensor/control
behavior changes are part of P2.
