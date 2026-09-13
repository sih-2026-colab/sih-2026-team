# AutoNex → RoadRunner integration boundary

`autonex_rr_telemetry(out)` emits only measured AutoNex output: timestamp,
world x/y in metres, yaw and steering in radians, speed converted from km/h
to m/s, selected maneuver and command. It does not run physics, advance time,
or change AutoNex. P3 validation writes a real telemetry JSONL sample stream.

Run `inspect_autonex_roadrunner` for a machine-readable local availability
report. Having a function or toolbox alone does not prove an executable,
license, scene, or actor binding works.

The current `Scenes/VillageRoad.rrscene` is byte-identical to
`village_road/scenario.json`. `Scenarios/VillageBasic.rrscenario` is
byte-identical to `urban_intersection/scenario.json`. These are JSON design
descriptions with misleading native extensions, not verified scenes. They
are preserved. Village describes an ego and animal; urban describes ego,
car, pedestrian, cow and a simple intersection layout. No authored buildings,
vegetation, RoadRunner roads or runnable actor behaviors were verified.

## Supported integration to implement when RoadRunner is available

1. Install/configure the RoadRunner executable and MATLAB connection. Open a
   real project and verify a native VillageRoad scene independently.
2. Author the narrow unmarked village road, roadside context, parked obstacle
   and conflict actor; preserve the authoritative AutoNex scenario coordinates.
3. Use MathWorks `roadrunner`, `openScenario`, `createSimulation`, and an actor
   behavior using `ActorSimulation.setAttribute('Pose', pose)`.
4. Bind the ego explicitly; calibrate the world-origin transform, yaw convention,
   road elevation, actor reference point and RoadRunner pose-matrix convention.
   Do not assume the raw packet is a RoadRunner pose matrix.
5. Consume exactly one AutoNex bicycle-model output per simulation step. Disable
   competing ego motion behaviors; transport pose for visualization only.
6. Prove x/y/yaw synchronization, no drift, reset, pause and stop before extending
   the same bridge to an unsignalized intersection with cross traffic/pedestrians.

No runtime pose setter is claimed or tested in P3 because the connection and
native assets are unavailable. No RoadRunner screenshots were fabricated.

Official workflow references:
- https://www.mathworks.com/help/driving/ug/connect-matlab-and-roadrunner.html
- https://www.mathworks.com/help/driving/ug/co-simulate-roadrunner-with-agents-modeled-in-matlab.html
- https://www.mathworks.com/help/driving/ref/simulink.actorsimulation.setattribute.html
