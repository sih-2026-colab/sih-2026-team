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

## Usage

- Python pipeline: see `python/` submodules
- Simulink integration: open `simulink/main_integration.slx`
