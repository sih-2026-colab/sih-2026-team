# AutoNex animation telemetry (schema 1.0)

This directory observes the existing simulator. It does not render animation,
invoke RoadRunner/Blender, or alter AutoNex decisions. Run from the repository:

```matlab
addpath('animation');
telemetry = export_autonex_video_telemetry('occluded_pedestrian');
reports = test_autonex_video_telemetry; % all six, 10 seconds each + baseline
reports = test_autonex_video_telemetry('saved'); % event regression only; NO simulation
```

Supported export names: `occluded_pedestrian`, `cut_in`, `animal_crossing`,
`dense_market`, `missing_lane`, `intersection`. The last name maps explicitly to
the existing `unsignalized_intersection` configuration. No new scenario is made.
Default settings match the judge runner: camera/radar/LiDAR perception, 10 s,
with other defaults from `autonex_options` (including seed and dt). Only the
existing `explainabilityTelemetry` and `sensorTelemetry` flags are enabled.
Options and output directory can be passed as arguments 2 and 3. A rerun
overwrites only the six exporter-owned files for that scenario.

## Outputs and conventions

`results/animation/<scenario>/` contains:

| File | Content |
| --- | --- |
| telemetry.mat | Plain-data `telemetry` structure, compressed v7; no live tracker handles |
| telemetry.json | Same logical schema for non-MATLAB consumers |
| ego.csv | One row per simulation step; world pose, controls and actual decision |
| tracks.csv | One row per confirmed track per step; no fabricated actor identity |
| candidates.csv | One row per returned candidate; score/gates and trajectory reference |
| events.json | Actual state transitions, times, IDs where available, evidence |

Top level: `schemaVersion`, `scenario`, `coreScenario`, `options`,
`coordinateFrame`, `sampleConvention`, `nonfinitePolicy`, `frames`, `events`,
`optionNonfinite`. Each frame contains `time`, `ego`, `decision`, `guardian`,
`collision`, `boundaryViolation`, `minimumFootprintClearance`, `tracks`,
`candidates`, `selected`, availability fields, `source`, and `nonfinite`.
`source` preserves all fields returned by `autonex_step`, with tracks converted
to plain snapshots of TrackID, State, StateCovariance, ObjectAttributes.

Coordinates are the existing world XY in metres; no Blender transformation is
applied. Heading/steering are radians, velocities m/s, acceleration m/s^2.
`ego.speedMps` converts the original `out.speed` km/h; both units are in JSON/MAT.
`time` identifies **pre-integration** pose. Controls apply over `[t,t+dt]`.
`source.nextEgo` is at `t+dt`, not the pose at the frame timestamp.
Candidate `trajectory.time` is a relative horizon; its absolute time is frame
time plus that horizon. IDs of candidates are frame-local, not persistent IDs.
MATLAB uses one-based indexing; CSV trajectory references use MAT notation.
Consumers should handle empty arrays and MATLAB JSON singleton objects as well
as multi-element arrays. CSV numbers use 17 significant digits.

## Audited source mapping

| Requested telemetry | Real source / interpretation |
| --- | --- |
| Time, position, heading, speed | out.time/x/y/egoYaw/speed |
| Acceleration, steering | out.ax/ay/steeringAngle; raw/requested/applied and saturation diagnostics retained in source |
| Detected objects | out.sensorFrame.camera/radar/lidar, distinct from source.actors simulation truth |
| Track IDs and confirmation | out.tracks.TrackID; first trackerGNN output contains confirmed tracks |
| Class, sensor sources, confidence | out.worldModel.class/sensorSources/confidence; fused per-sensor confidence in explainability.fused |
| Range | Raw sensor range when supplied; tracks.rangeM is explicitly derived Euclidean centre distance from current track and ego |
| Relative velocity | Track vx/vy minus ego vx/vy; raw radar radialVelocity remains separate |
| Uncertainty | Track StateCovariance, worldModel.uncertainty, fusedUncertainty; raw sensor covariances |
| Object predictions | UNAVAILABLE: evaluator-local predictions are not captured in out |
| Safety bubble geometry | UNAVAILABLE: evaluator-local envelopes are not captured in out |
| TTC | Candidate guardianResults.minFrontTTC/minRearTTC (minimum over candidate horizon); source.rearTTC is separate rear estimator output |
| Clearance | out.minClearance = current truth footprint clearance; planner minClearance = predicted centre distance, not footprint clearance |
| Candidate paths/scores | out.candidates and explainability.plannerResults, copied exactly |
| Rejection reasons | Labels from recorded planner conflicts/boundary/comfort and guardian.safe; detailed Guardian numeric evidence retained |
| Selected path | out.selected copied exactly, including rear mitigation selection; empty when core returns no selected trajectory |
| Rear risk | source.rearRiskActive/Level, rearTTC/Distance/ClosingSpeed/PredictedGap, rearActionDiagnostics and score fields |
| Guardian/Cat Reflex | out.guardianMode, longitudinalCommand, guardianOverrideActive, steering diagnostics |
| Final decision | out.motionMode verbatim, including STOP; never inferred from animation needs |
| Collision / boundary | out.collision/boundaryViolation: existing simulation-truth evaluations |

## Missing data and nonfinite policy

- Do not mistake ground-truth actor IDs/classes for perceived track identity.
  No reliable detection-to-truth actor association is exported. Camera pedestrian
  events therefore have no actor/track ID.
- The judge display reconstructs CV curves, uncertainty ellipses, and envelopes
  in `autonex_judge_explain`. This exporter deliberately does not invoke that
  calculation or present reconstructed geometry as captured core telemetry.
  Object prediction and safety-bubble fields are empty and marked unavailable.
- Paths eliminated inside `generate_corridor_candidates` by the occupancy gate
  never reach `out`; they and their rejection histories cannot be exported.
- Rear mitigation candidates/scores already exposed in `rearActionDiagnostics`
  remain in `source`; they are not fabricated into the main candidate list.
  A selected mitigation need not match a main candidate.
- Thermal is inactive in the validated P3 perception mode. Missing per-sensor
  confidence is not zero confidence. Unknown class remains `unknown`.
- Existing Inf sentinels mean such things as no eligible TTC, no distance, or
  unbounded road geometry; they are not finite measurements. NaN can mean an
  inapplicable diagnostic. Each occurrence is replaced by `[]` and listed in
  `frame.nonfinite` with original `NaN`, `+Inf`, or `-Inf` and exact field path
  (MATLAB linear indices). Mixed numeric arrays become same-sized cell arrays
  with empty elements. CSV unavailable values are blank; consult JSON/MAT's
  nonfinite log to distinguish sentinels. Core nonfinite ego/control samples
  fail export instead of being accepted as an animation pose.

## Events

Events occur on rising edges; the first frame records already-active states.
No fixed timestamps, sampling interpolation, or smoothing is used. Disappearance
then reappearance is a new event; genuine decision switching is not suppressed.

- PEDESTRIAN_DETECTED: camera reports pedestrian class.
- TRACK_CONFIRMED: track ID appears in confirmed GNN output.
- TTC_CRITICAL: a returned candidate's minimum front TTC <2 s or rear TTC <1.5 s,
  matching existing Guardian gates. This describes a candidate, not necessarily
  the selected/ego path. Candidate ID is recorded.
- CANDIDATE_REJECTED: planner or Guardian gate rejects a returned candidate.
- SAFE_PATH_SELECTED: exact selected candidate passes both recorded safe flags.
- GUARDIAN_INTERVENTION: guardian mode other than APPROVED/NO_ACTION appears;
  its actual mode is evidence, including stop/no-safe-space states.
- CRUISE / REPLAN / BRAKE / STOP: exact out.motionMode transitions.
- PREDICTION_ACTIVE is **not emitted** because active prediction output is not
  captured. A confirmed track alone does not prove a retained prediction.

Candidate event continuity uses name + target speed + target Y, since IDs are
frame-local. Changing target geometry may produce many genuine candidate events;
the exporter does not turn them into a scripted video narrative.

## Validation

`test_autonex_video_telemetry` exports all six scenarios, then reruns each with
telemetry flags absent and the same seed/options. It compares **every existing
out field at every sample**, using plain track snapshots and explicit nonfinite
normalization. It checks ego/decisions/paths, monotonic time, caller RNG,
MAT/JSON/CSV round trips, repeated-event idempotence, and nonfinite handling.
Observed collisions/boundary violations are reported, not retuned or hidden.
Results are in `results/animation/validation.json`.

After MATLAB validation completes, run the independent read-only file checks
from PowerShell (not while exports are still being written):

```powershell
./animation/validate_autonex_video_exports.ps1
# Resume only the two remaining file checks, without regenerating data:
./animation/validate_autonex_video_exports.ps1 -Scenarios missing_lane,intersection
```

These check all six files per scenario, every CSV field against JSON/source,
track covariance/class/source attribution, candidate paths and scores, selected
path, unavailable geometry, and the actual source evidence for every event.
MAT v7 is intended for these bounded scenario runs; very long runs may exceed
its per-variable size limit and are not validated by this milestone.

Inspected files: autonex_step.m, autonex_options.m, startup.m,
create_highway_scenario.m, configure_autonex_scenario.m,
configure_new_sih_scenario.m, update_autonex_world_model.m,
create_autonex_gnn_tracker.m, autonex_sensor_frame.m, simulate_rgb_camera.m,
fuse_autonex_detections.m, generate_corridor_candidates.m,
generate_2d_trajectory.m, evaluate_2d_trajectories.m,
cat_reflex_2d_guardian.m, estimate_rear_risk.m,
prototype/run_autonex_judge_prototype.m, prototype/autonex_judge_explain.m,
prototype/autonex_judge_overlay.m, prototype/autonex_demo_catalog.m,
prototype/autonex_demo_metrics.m, prototype/test_autonex_judge_p3.m.
