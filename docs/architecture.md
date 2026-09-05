# Architecture

## Overview

The system follows a classic autonomous-driving stack:

```
             ┌────────────┐      ┌────────────┐      ┌────────────┐
  sensors ──▶│ PERCEPTION │─────▶│ PREDICTION │─────▶│  PLANNING  │
             └────────────┘      └────────────┘      └─────┬──────┘
                     │                    │                ▼
                     │              ┌─────┴─────┐    ┌───────────┐
                     └─────────────▶│ EVALUATION│    │  CONTROL  │
                                    └───────────┘    └───────────┘
```

## Data flow

1. **Perception** (`python/perception/`, `matlab/perception/`)
   Turns raw sensor data into `DetectedObject` records — labelled,
   positioned (bird's-eye x/y), with speed and detector confidence.
   Output: a `PerceptionFrame` per timestamp (MAT: `perception_output.mat`).

2. **Prediction** (`python/prediction/`, `matlab/prediction/`)
   Predicts each object's future trajectory. Baseline model:
   constant velocity. Also computes Time-To-Collision (TTC) vs. ego.
   Output: trajectories + TTC (MAT: `prediction_output.mat`).

3. **Planning** (`matlab/planning/`, `simulink/planner.slx`)
   MATLAB/Simulink side: generates ego waypoints from detections.

4. **Control** (`matlab/control/`)
   Converts planned waypoints into vehicle actuator commands (PID/MPC).

5. **Evaluation** (`python/evaluation/`)
   Scores the pipeline:
   - detection: precision / recall / F1 (greedy position+label matching)
   - trajectory: ADE / FDE vs. ground truth

## Python ↔ MATLAB bridge

`python/utilities/matlab_bridge.py` reads/writes MAT-files (via `scipy`) so
both sides exchange data through files:

```
input_objects.mat ─▶ run_perception.m ─▶ perception_output.mat
                  ─▶ run_prediction.m ─▶ prediction_output.mat
                  ─▶ run_planner.m    ─▶ planning_output.mat
```

The demo (`python/utilities/matlab_integration_demo.py`) runs the chain,
optionally launching MATLAB with `RUN_MATLAB=1`.

## End-to-end Python pipeline

`run_pipeline.py` builds a synthetic urban-intersection scene, runs
perception → prediction → evaluation, and writes a JSON report to
`results/pipeline_report_<timestamp>.json`.

## Scenarios

`roadrunner/` holds RoadRunner scene assets:
- `village_road/` — low-speed rural scenario
- `urban_intersection/` — intersection with VRUs (pedestrians, animals)

## Testing

```
python -m pytest        # 18 tests: smoke + module units + MATLAB bridge
python run_pipeline.py  # end-to-end pipeline demo
```
