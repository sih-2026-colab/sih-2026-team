# AutoNex: running the current prototype

Start MATLAB in `SIH-Autonomous-Driving`, then run `rehash; addpath(pwd);`.

## Current shared simulation

```matlab
report = run_autonex_2d_closed_loop;
report = run_autonex_animated(pwd, true);
```

The first command displays the shared closed loop. The second records it.
Both use simulated depth/radar/RGB/thermal observations, GNN tracks, a connected
free-space corridor, trajectory selection, the guardian and vehicle controllers.
The original fixed-lane demonstration remains in `run_autonex_2d_legacy` and
`run_autonex_highway_animated`. `autonex_driving_animation.mp4` is that earlier demo.

The `77–81 GHz` label describes the intended radar hardware class. The current
radar code is a noisy range/azimuth/range-rate abstraction, not an FMCW waveform
or RF propagation simulation. Camera and thermal observations are also synthetic.

## Validation

```matlab
summaries = validate_autonex;
finish_autonex_verification;
```

The MATLAB suite writes `results/matlab_validation_summary.json`. The similarly
named `results/validation_summary.json` belongs to the separate, simpler Python
AEB suite; its passing results do not certify the MATLAB planner.

`finish_autonex_verification` additionally runs a close rear vehicle at -18 m
as `close_rear_cut_in`, checks lane-marking independence, compares the full
six-second Simulink output with MATLAB, and decodes all 121 frames of
`autonex_safe_space_demo.mp4`. The normal highway scenario currently starts the
rear vehicle at -60 m and cut-in vehicle at +35 m. Those scenarios must not be
treated as equivalent. `verify_original_close_range` restores both original
positions (rear -18 m, cut-in +4.1 m) and saves its result separately.

The collision metric uses 4.5 x 1.9 m axis-aligned vehicle rectangles in the
current 2-D model. It is a sampled simulation check, not continuous collision
certification. Other traffic follows scripted motion and does not react to ego.
Remembered free space expires after ten seconds; tracked moving objects are
checked in the temporal planner instead of permanently blocking the road map.

## Simulink

```matlab
build_autonex_simulink;
open_system(fullfile('simulink','integration','main_autonomous_vehicle.slx'));
verify_autonex_simulink;
```

The executable model uses a Level-2 MATLAB S-function at 0.05 s and shares the
same simulation step. Its eight outputs are x, y, speed in km/h, ax, ay, track
count, emergency flag and collision flag. This supports normal simulation;
code generation, saved operating-point restore and hardware deployment are not
implemented. The older empty `perception.slx` and `planner.slx` files are unused
placeholders, not working submodels.

## RoadRunner: external prerequisites remain

`check_roadrunner_readiness` records what is installed. The inspected machine
has no RoadRunner MATLAB API or Automated Driving Toolbox, and this repository
has no native `.rrproj` / `.rrscenario` assets. JSON scene descriptions are not
native RoadRunner projects.

After installing/licensing the required products, creating the native scene and
scenario, and implementing/assigning an actor behavior that reads RoadRunner
state instead of the synthetic world, use:

```matlab
simulation = run_autonex_roadrunner(projectFolder, scenarioFile);
```

The existing standalone Simulink model is not itself a completed RoadRunner
actor adapter. The launcher fails explicitly if prerequisites are absent.
See [MathWorks RoadRunner integration documentation](https://www.mathworks.com/help/driving/roadrunner-scenario-simulation.html).

## Broader SIH work still outside this validated baseline

Dedicated pothole/road-surface perception, an IMM tracker, a trained intent
model and its training/evaluation data, full MATLAB animal/night-pedestrian
scenarios, and the final SIH presentation remain separate work. Python AEB
animal/pedestrian examples are foundations, not full multimodal MATLAB scenarios.
