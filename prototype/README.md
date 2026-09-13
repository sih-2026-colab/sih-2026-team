# AutoNex judge console — P1

From the repository root in MATLAB R2026a:

```matlab
startup
addpath('prototype')
app = run_autonex_judge_prototype('occluded_pedestrian');
```

Press START. PAUSE becomes RESUME. RESET initializes the selected scenario at
simulation time zero; changing the scenario also resets it. Closing the window
stops and deletes its timer. The 0.5x selector doubles the display interval;
the original simulation timestep remains unchanged. Computation/rendering cost
can make wall-clock playback slower than the selected rate.

The default uses the existing `camera_radar_lidar` perception mode. Pass a
normal AutoNex options struct as the second argument to choose another mode or
duration. Default duration is 10 simulated seconds. START after completion
restarts the selected scene. No video is generated.

The perspective view and top-down view read `out.actors`, `out.tracks`,
`out.candidates`, and `out.selected`. Actor dimensions come from the existing
geometry utility. Road rectangles come from the scenario's existing road
geometry. Observed free cells and the corridor come from `state.drivable` and
`state.corridor`. Camera-follow smoothing affects only axes limits.

Cards map directly to `speed`, `targetSpeed`, `ax`, `steeringAngle` (converted
from radians to degrees), `targetY`, `selectionMode`, `selectedName`,
`guardianMode`, `longitudinalCommand`, `emergency`, `trackCount`,
`candidateCount`, `minClearance`, `collision`, and `boundaryViolation`.
The prominent decision uses `motionMode`; no presentation decision is inferred.
Missing values display N/A. Infinite clearance displays "No finite clearance".

Grey lines show all actual candidates, not asserted-safe or rejected candidates:
the existing output does not expose an individual candidate rejection mask.
The selected path is green and confirmed tracks are yellow. The two views use
different camera projections of the same actual simulation state. P1 excludes
uncertainty/TTC overlays, sensor cones, risk plots and video export.

Run `test_autonex_judge_prototype` for exact core-output comparison, UI field
checks, three simulated seconds in both requested scenes, stable graphics
object counts, Start/Pause/Resume/Reset checks and screenshots. It also invokes
existing actuator, continuity and closed-loop motion tests without modifying
their assertions. Screenshots and machine-readable evidence go to `results/`.

All dashboard code lives in this folder. Graphics functions read data and do
not advance the simulation. Only the runner calls `autonex_step`. Actor and
label handles are pooled and reused; the candidate collection uses one NaN-
separated line. No growing history or extra figure is created per step.
