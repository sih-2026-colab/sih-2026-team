# SIH - Autonomous Driving

Autonomous driving pipeline integrating Python (perception / prediction / evaluation) with MATLAB/Simulink (planning / control), with RoadRunner scenarios for simulation.

## Project Structure

```
SIH-Autonomous-Driving/
├── data/                    # Datasets (images, lidar, maps)
├── python/
│   ├── perception/          # Object detection, lane detection
│   ├── prediction/          # Trajectory / behavior prediction
│   ├── evaluation/          # Metrics, scoring, validation
│   └── utilities/           # Shared helpers, loaders, visualization
├── matlab/
│   ├── perception/          # MATLAB perception modules
│   ├── prediction/          # MATLAB prediction modules
│   ├── planning/            # Path planning algorithms
│   └── control/             # Control (PID, MPC, etc.)
├── simulink/                # Simulink models
├── roadrunner/              # RoadRunner scenario assets
├── results/                 # Outputs, logs, plots
├── tests/                   # Unit & integration tests
└── docs/                    # Documentation
```

## Setup

```bash
pip install -r requirements.txt
```

MATLAB modules require MATLAB R2023b or later with Automated Driving Toolbox, RoadRunner, and Simulink.

Continuous Integration
----------------------

This repository includes a GitHub Actions workflow `.github/workflows/python-tests.yml` that runs the Python tests on push and pull requests to `main`.

Run tests locally:

```powershell
python -m pip install -r requirements.txt pytest
python -m pytest -q
```

## Prototype preview (start here)

The target demo is **VRU emergency braking** at an urban intersection.

1. Open `preview/index.html` in a browser and click **Play both**.
   Left = AEB off (collision). Right = AEB on (ego stops).
2. Same physics in Python: `python -m python.prototype.aeb_sim`
3. Same physics in MATLAB: `matlab/main/preview_aeb.m`
4. When RoadRunner is installed: `matlab/main/run_roadrunner_demo.m`
   Build `simulink/ego_aeb.slx` with `matlab/simulink/build_ego_aeb.m` and attach it to the ego actor. Scene layout lives in `roadrunner/urban_intersection/scenario.json`.

## Usage

- Python pipeline: see `python/` submodules
- Simulink: generate `simulink/ego_aeb.slx` from MATLAB (`build_ego_aeb`)

MATLAB/Simulink integration demo
--------------------------------

We've added starter MATLAB scripts (stubs) and a Python demo showing MAT-file exchange:

- `matlab/perception/run_perception.m` — loads `input_objects.mat`, writes `perception_output.mat`
- `matlab/planning/run_planner.m` — loads `perception_output.mat`, writes `planning_output.mat`
- `matlab/main/run_integration.m` — runs perception and planning in sequence

Python utilities:

- `python/utilities/matlab_bridge.py` — read/write MAT and JSON helper functions (requires `scipy`)
- `python/utilities/matlab_integration_demo.py` — demo runner: writes `input_objects.mat`, optionally invokes MATLAB, then reads `planning_output.mat`.

How to run the demo (locally with MATLAB installed):

1. Ensure MATLAB is installed and accessible (e.g. `matlab` on PATH). On Windows, MATLAB executable is typically `matlab.exe` and may need full path.
2. Install Python dependencies: `pip install -r requirements.txt` (includes `scipy`).
3. From the project root, run the demo without launching MATLAB (to just write input files):

```powershell
python python\utilities\matlab_integration_demo.py
```

4. To run MATLAB automatically, set environment variable `RUN_MATLAB=1` and ensure `MATLAB_CMD` points to a valid MATLAB command if needed. Example on PowerShell:

```powershell
$env:RUN_MATLAB = "1"; python python\utilities\matlab_integration_demo.py
```

After running MATLAB, check `planning_output.mat` for planner results. The MATLAB scripts are placeholders — open them in MATLAB and replace the stubs with your real models or calls to Simulink models.

## Simulator & Validation (software-only)

This is a **software-only autonomous-driving simulator** — no physical sensors,
vehicle hardware, CAN data, or real sensor datasets are required. All data is
synthetic and deterministic.

### Pipeline

Perception → sensor fusion (RGB / thermal IR / radar / depth, simulated) →
tracking → prediction → safe-corridor planning → trajectory selection →
emergency braking (AEB) → control.

### Deterministic scenarios (8)

`python/prototype/scenarios.py` defines fixed scenes; `run_validation.py` runs
them all with AEB on/off and writes `results/validation_summary.json`:

1. `urban_intersection` — car + crossing pedestrian + cow on the road
2. `highway` — fast lead car, ego at 30 m/s
3. `village_road` — animal steps onto a narrow rural road
4. `pedestrian_crossing` — pedestrian enters the ego lane
5. `cut_in` — vehicle merges into the ego lane
6. `stationary_obstacle` — stalled vehicle in the lane
7. `tracking_loss` — object dropped from tracking after 1 s
8. `degraded_sensing` — sensor range limited to 30 m

### Measured metrics

Per scenario: **collision status, first TTC, minimum clearance (m), stopping
distance (m), emergency response time (s), false braking, emergency steps**.
A run passes when there is no collision, no false braking, and AEB-on
clearance ≥ AEB-off clearance.

### Running

```bash
python -m pytest          # unit + scenario tests (all deterministic)
python run_validation.py  # 8-scenario validation -> results/validation_summary.json
python run_pipeline.py    # perception→prediction→evaluation demo report
```

### MATLAB validation

MATLAB-side sensor simulation (thermal IR, radar, GNN tracking, fusion) and
scenario tests live in the repository root and `matlab/` (e.g.
`simulate_thermal_ir.m`, `test_radar_thermal_fusion.m`,
`matlab/scenarios/run_roadrunner_demo.m`). RoadRunner and Simulink are
**optional adapters** — the complete pipeline runs with MATLAB/Python alone.

> **Note:** This project does **not** claim hardware or real-vehicle support.
> All validation is software simulation with synthetic data.
