# AutoNex P3 judge demonstration

Run `startup; addpath('prototype'); app=run_autonex_judge_prototype;`.
The Scenario tab introduces the selected challenge without prescribing a
command. START shows live path explainability. Readable dropdown entries map
to the existing six scenario IDs. NEXT SCENARIO cycles through the catalog.
RESET creates a fresh core state and tracker, clears event and metric history,
and updates/hides pooled graphics from the new time-zero output.

DEMO MODE controls whether completion opens the Scenario/result tab. The
existing timer stops at the configured duration either way. 0.5x changes timer
pacing only. Pausing/resuming and the pipeline timestep remain unchanged.

Metrics use actual output only:
- collision and boundary: accumulated boolean OR;
- clearance: minimum observed `out.minClearance`;
- distance: sum of consecutive ego x/y Euclidean displacements;
- steering: maximum absolute steering, converted from radians to degrees;
- braking events: transitions into `out.ax < 0`, including an initially braking sample;
- replans: changes in selected maneuver name, matching the existing regression
  report's definition (not a count of planner invocations).

The live result is REVIEW until the complete existing acceptance suite has
qualified that run. Collision, boundary departure, or nonfinite state yields
FAILED. Absence of these alone never produces a fabricated PASS. Existing
acceptance results are reported separately by `test_autonex_judge_p3`; no stale
cached test result is attached to a new live run.

`test_autonex_judge_p3` verifies exact headless equivalence for 61 frames of all
six scenarios, repeated-render handles/RNG/events, reset/next isolation,
metric calculations and timer completion. It also runs ten-second core smoke
for each scenario and the unchanged visibility, pedestrian/intersection,
actuator, trajectory continuity, and closed-loop motion tests. Smoke safety
observations are reported, not relabelled as scenario-specific acceptance.
Results and actual simulation screenshots are written to `results/judge_p3*`.

RoadRunner integration status and the supported future workflow are documented
in `roadrunner/integration/README.md`. The bridge exports actual scalar telemetry
only; it does not duplicate dynamics or claim an untested live RoadRunner link.
