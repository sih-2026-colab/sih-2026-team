# AutoNex multi-sensor world model

## Mode and scope

`perceptionMode='legacy'` remains the frozen default. Select
`perceptionMode='camera_radar_lidar'` for the new perception mode. The existing
GNN/IMM tracker, prediction, uncertainty-based safety envelope, corridor logic,
candidate scoring, Cat Reflex, rear-risk handling, Pure Pursuit and bicycle
motion chain remain in place. Radar-timeout braking authority is NOT relaxed
when other sensors remain available. No IR or surface-hazard feature is added.

## Data flow

Simulated camera + four existing 77-81 GHz radar simulators + planar depth
returns -> world-frame detections -> gated one-to-one cross-sensor association
-> covariance-intersection position/velocity reports -> shared GNN/IMM tracker
-> worldModel and compatible planner-facing tracks/fused uncertainty -> existing
risk/planner/controller chain -> updated ego -> next sensor cycle.

One report per associated object, not one track per sensor. Actor IDs are used
inside simulators to exclude ego, never as association keys. Camera contributes
class and noisy synthetic position. Radar range/angle determines position;
Doppler plus sensor-world velocity constrains LOS velocity. Perpendicular
velocity retains high variance, rather than inventing a fully observed vector.
Overlapping radar observations use covariance intersection to avoid unjustified
independence assumptions.

## World-model record

`out.worldModel` and `state.worldModel` contain one record per confirmed track:

```matlab
id, class, x, y, vx, vy, heading, confidence, uncertainty, ...
sensorSources, sensorDevices, lastMeasurementTime, coasted
```

Positions are world metres; velocity is world m/s; heading is radians.
Confidence is [0,1]. Uncertainty combines position covariance with missing-
modality and staleness penalties; it is not a calibrated probability guarantee.
`sensorSources` contains current modalities in camera/radar/lidar order.
`sensorDevices` retains contributing FRONT/REAR/CORNER_RADAR identifiers.
Coasted tracks have no current sources; confidence decays and uncertainty grows.
Class history survives brief dropout; normal GNN confirmation/deletion applies.

## Geometry and dimensions

`sensorConfig` supports cameraPose, radarPoses (4x3), lidarPose, and lidarNoise.
Each pose is [forward metres, left metres, yaw radians] in ego coordinates.
Sensor -> ego -> world transforms include rotation and translation. Radar
velocity compensation includes the yaw-rate cross mounting-offset term.
The loop supplies yaw rate from its existing speed, steering and wheelbase.

New perception uses these synthetic size priors, not silent generic-car sizes:

| Class | Length x width (m) |
|---|---|
| car | 4.5 x 1.9 |
| bus | 10 x 2.5 |
| two-wheeler / bike | 2 x 0.8 |
| auto-rickshaw | 2.6 x 1.4 |
| pedestrian | 0.6 x 0.6 |
| animal | 2 x 0.8 |
| pushcart | 1.8 x 1 |
| unknown/static obstacle | 1 x 1, explicitly an unknown prior |

Valid explicit actor.length/actor.width override priors. These are simulation
defaults, not universal real dimensions. The frozen planner's footprint logic
is unchanged; class-specific perception dimensions do NOT certify class-specific
planner clearance handling. Vehicle shape priors matter especially for buses.

## LiDAR limitation

This is **planar synthetic depth/LiDAR object detection**, not production-grade
3-D LiDAR simulation. Oriented ray/box first returns feed the installed
`pointCloud` and `pcsegdist` functions. Known synthetic road-edge returns and
max-range no-returns are removed. Exposed-surface cluster centres are not claimed
to be true actor centres; conservative covariance/gates account for that bias.
Very close/touching or occluded objects remain an association/clustering limit.
Optional point noise is tested, not a complete beam/reflectivity/weather model.

MATLAB R2026a installation includes Lidar, Computer Vision, Sensor Fusion and
Tracking, Radar, and Navigation toolboxes. `lidarSensor` exists but is not used
by this minimal planar pipeline. `lidarPointCloudGenerator`,
`visionDetectionGenerator`, `drivingRadarDataGenerator`, and `pccluster` were
not available and are not used. Camera classes are simulated, not image-model
recognition. No model has been trained or downloaded.

## Reproduce

From the AutoNex project root in MATLAB R2026a:

```matlab
startup
test_multisensor_world_model
acceptance = validate_multisensor_acceptance();
assert(all([acceptance.passed]));
r = run_autonex_simulation(struct('perceptionMode','camera_radar_lidar', ...
    'scenario','cut_in','duration',10));
regressions = run_motion_regressions();
assert(all([regressions.passed]));
checks = validate_rear_mitigation_regression();
assert(all([checks.passed]));
validate_closed_loop_path_following();
```

`sensorAvailability=[camera radar lidar]` controls modality availability.
Acceptance tests separately count seven combinations/provenance, nine classes
and dimension priors, yaw/offset/turning geometry, Doppler observability, nearby
noisy depth clustering, four dropout/recovery cases, quality degradation and
three new-mode closed-loop cases. Full frozen-mode scenarios are a distinct
regression check, not substitutes for new-mode validation.

## Files

Changed: autonex_options.m, create_autonex_gnn_tracker.m, autonex_step.m and
matlab/core/autonex_step.m (perception interfaces only).

New in this milestone: autonex_object_class.m, autonex_sensor_config.m,
autonex_perception_dimensions.m, autonex_planar_returns.m,
autonex_sensor_frame.m, simulate_autonex_lidar_objects.m,
fuse_autonex_detections.m, update_autonex_world_model.m,
tests/test_multisensor_world_model.m, tests/validate_multisensor_acceptance.m,
and this document. Some were created in the prior continuation and completed
here. Existing controller, bicycle, risk and planner implementations unchanged.

Greedy gated association does not guarantee zero ID switches in arbitrary
crowds. Finite deterministic test success is not real-vehicle certification.
