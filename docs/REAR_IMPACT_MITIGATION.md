# Rear-impact-aware minimum-risk planning

## Root cause

The shared loop discarded `OVERRIDE_MINIMUM_RISK` candidates and commanded an emergency stop. Rear TTC could reject a moving candidate, but the stop itself was not compared with the rear consequences of reduced braking or lateral escape. In the three original failing scenes, actor 2 continued at its scripted speed and reached the braking/stopped ego. The old truth-based longitudinal rear evaluator did not participate in this shared loop.

## Implementation

`estimate_rear_risk` uses confirmed track state and lateral covariance. It considers only tracks behind ego, predicts lateral overlap and positive closing speed under a braking forecast, and reports bumper gap, lateral offset, closing speed, instantaneous TTC and minimum projected gap. A six-second maximum-braking forecast gives an early alert even when a rear vehicle is not yet closing at the current ego speed. This is a deterministic risk classification, not a calibrated collision probability. `rearThreatActorId` is a tracker ID; `rearThreatIdSource=TRACK_ID` makes this explicit.

`evaluate_minimum_risk_action` reuses corridor targets, candidate speeds and quintic generation, adding a 5 km/h creep target. Every candidate passes the existing front trajectory evaluator and 2-D guardian, then a rollout using the existing longitudinal controller, physically limited Pure Pursuit and bicycle integration. The rollout checks front hard distance, contextual safety ellipse, front TTC, stopping distance, yawed footprint road limits, lateral acceleration and observed free-space occupancy. Only surviving actions receive a rear-gap cost, followed by lateral effort and forward progress. A currently safe plan is retained when its rolled rear cost is no worse.

The action horizon is the shared planner's three seconds; the alert horizon is six seconds. Applying six seconds of highway-speed motion to the 70 m observed map excluded useful alternatives. Unknown cells are still rejected; the shorter action horizon does not authorize any travel through them. This remains a receding-horizon controller, not a guarantee beyond its verified horizon.

Rear overlap scoring uses the rotated ego footprint plus the existing vehicle-size assumption and lateral track uncertainty. The rollout's front hard-overlap check also expands when the rotated footprint requires more lateral space than the existing fixed threshold. These checks tighten geometric rejection without relaxing any existing gate.

The shared loop invokes this selector during braking/fallback when rear risk is active. Existing immediate `BRAKING_DISTANCE` and `SENSOR_TIMEOUT` decisions remain authoritative. If no alternative survives, the existing stop remains. No scenario geometry, physical steering bound, safety bubble threshold, corridor stabilization or Pure Pursuit tuning is changed.

## Diagnostics and interpretation

Rear threat activation is binary; candidate conflict scoring is continuous. The current selector measures `baselineRearConflict` by rolling the action the loop would otherwise execute (selected path, or the physical emergency stop). The baseline is only a comparator: an unsafe baseline can never become an admissible candidate. Each hard-safe candidate reports `candidateRearConflict` and `rearConflictReduction = baselineRearConflict - candidateRearConflict`.

`score_rear_conflict_reduction` treats a reduction above max(0.001 conflict units, 1% of the baseline conflict) as meaningful, and a reduction of at least 50% as large. These are explicit prototype decision bands, not calibrated collision probabilities. Meaningful/large gains occupy separate safety-priority score bands. A bounded comfort/progress contribution cannot cancel the gain between bands. Inside a band, relative reduction and secondary costs rank candidates. Negligible reduction supplies no safety bonus; worsening conflict receives a penalty. Lateral displacement itself is never rewarded. The existing continuous overlap exponent and conflict calculation are unchanged by this relative-improvement correction.

Every evaluated admissible candidate has `baselineRearConflict`, `candidateRearConflict`, `rearConflictReduction`, `rearRiskCost`, `comfortCost`, `progressCost` and `totalScore` in `rearActionDiagnostics.candidateScores`. The best evaluated candidate's values are also exposed directly in per-step telemetry. NaN/null means no candidate was evaluated. The selected-path retention check also uses relative-improvement scoring.

`test_rear_conflict_reduction` covers partial improvement, displacement with no improvement, hard front/occupancy rejection, negligible improvement favoring centered motion, and large improvement. The actual rollout evaluator is exercised by `test_rear_risk`, not only the scalar scoring helper.

- `minimumRiskMode`: NORMAL, FRONT_BRAKE, REAR_AWARE_BRAKE, FORWARD_CREEP, LATERAL_ESCAPE or EMERGENCY_STOP. Creep describes the selected target, not a claim that current speed is already 5 km/h.
- `rearActionDiagnostics`: screened rollout count, feasible rollout count and best rear cost. This is a finite candidate search, not an exhaustive feasibility proof.
- `rearImpactUnavoidable` remains false. The scripted rear motion does not establish that the real actor lacks braking/avoidance capability, and sampled alternatives cannot prove every physically feasible action fails.
- Focused collision counts are sampled overlap frames; collision time is the first frame and partners are ground-truth actor IDs used only for evaluation. Front/rear minimum clearance classifies other actors by their position at each sample. Steering summaries use degrees and degrees/s; speed uses km/h, distances metres and TTC seconds.
- No rear collision, including a hypothetically unavoidable one, is relabelled PASS.

## Reproduction

Run `test_rear_risk; validate_rear_impact_mitigation; validate_rear_mitigation_regression;` in the project root. Focused runs preserve seed, timestep, original geometry and full 10-second durations. The regression includes all 12 existing cases (25 seconds for missing_lane), four SIH smoke scenes, actuator, continuity, path following, prototype components, degraded-fusion and corridor checks.

Baseline results are archived as `results/rear_mitigation_baseline_regression.json` and `results/rear_mitigation_baseline_smoke.json`. The rejected six-second action-horizon draft is retained in `results/rear_mitigation_initial_draft.json`. Final focused metrics and per-step telemetry are written to `results/rear_mitigation_summary.json` and `results/rear_mitigation_<scenario>.json`; regression execution results go to `results/rear_mitigation_regression_checks.json`.
