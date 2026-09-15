# AutoNex SIH26037 — Blender Stage 1

This is a telemetry-driven proof scene, not the final film. No MATLAB execution,
RoadRunner integration, new driving algorithms, or source-telemetry writes.

## Open the built scene

1. Start Blender 5.1.
2. File > Open > `animation/blender/assets/autonex_missing_lane_stage1.blend`.
3. Switch to the Scripting workspace. In the Text Editor's text dropdown choose
   `AUTONEX_PLAYBACK.py` and click Run Script (Alt+P while the pointer is there).
   This activates the read-only playback handler; merely opening the blend does
   not register a Python handler. No global auto-run preference is changed.
4. Return to Layout, press Numpad 0 for the active camera, then Space to play.
   Scrub frames 1–300. The saved initial proof frame is an actual selected-path
   state. Use frame 301 separately to inspect the exact t=10 endpoint.
5. To change view, select a camera in AUTONEX_CAMERAS and use Ctrl+Numpad 0,
   then scrub one frame so the telemetry HUD attaches to that camera.

## Rebuild from scripts

Use a fresh Blender file (File > New > General). In Scripting > Text Editor >
Open, choose `animation/blender/scripts/setup_autonex_master.py`, then Run Script.
The script creates a separate AUTONEX_STAGE1 scene without deleting any other
scene. It refuses to rebuild into a file that already has that scene. It saves
the generated blend under assets and validation/provenance under references.

Command line from the repository root:

```powershell
& 'C:\Program Files\Blender Foundation\Blender 5.1\blender.exe' --background --factory-startup --python animation/blender/scripts/setup_autonex_master.py
```

Add `-- --proof` to render two still PNGs (technical and chase). This is not an animation
render. Setup uses Eevee, 1920x1080, 30 fps, frames 1–300. Keep the repository and
telemetry location intact: the scene records the absolute source path and its
SHA-256. Playback refuses changed telemetry. After relocating the repository,
rebuild from scripts to record the new source location.

## Data authority and coordinates

The authoritative source is `results/animation/missing_lane/telemetry.json`.
The setup also reads `ego.csv` for an independent interpolation cross-check.
Its embedded frames include the same ego, candidates, track state and events
as the CSV/JSON companion exports. Blender metres equal MATLAB metres:
`Blender X = world X`, `Blender Y = world Y`, +Z up. Positive heading is rotation
about +Z, counterclockwise from +X, radians. No recentering or scale distortion.
Vehicles point along local +X. Car length/width is 4.5/1.9 m, consistent with the
existing simulation's passenger-car footprint. Mesh height, tyres, body shape,
roadside decoration and daylight are visual placeholders, not sensed geometry.

Truth actors use `source.actors.id`; perceived markers use `tracks.id`. They
remain separate collections. No actor-to-track association is invented.
Actors retained far away by the original scenario remain far away in Blender.

Frame time is `(frame-1)/30`. Frames 1–300 are 300 images / 30 fps = 10 s,
covering sampled display times 0–9.9666667 s. Frame 301 is the exact 10 s endpoint
and is outside the render range. No stretching of 10 s to 301 rendered frames.
Ego XY and truth-actor XY use linear interpolation; headings use shortest-angle
interpolation. This is display interpolation, not new vehicle-motion logic.
Recorded controls, tracks, candidate paths and decisions are held at the latest
sample at or before the display time. No interpolation between unrelated paths
or track IDs. `references/frame_provenance.csv` records both source brackets and
alpha for all 301 inspectable frames. Scene and animated root custom properties
also retain the display time and source brackets.

## Road and collections

There is a 300 m long, 6 m asphalt strip centred at world Y=5.25, no lane paint,
uneven gravel shoulders, soil transition, simple plaster buildings, poles and
lightweight vegetation. This placement accommodates the actual ego path without
moving either blocker. The lower blocker at Y=0 sits beside the asphalt, on soil.
This is an illustrative unstructured road; it is **not** a claim that the original
planner's full drivable domain or occupancy map was exported. No green road mesh
is labelled "safe". The original source legal bounds are wider than the asphalt.

Collections: AUTONEX_WORLD, AUTONEX_ROAD, AUTONEX_EGO, AUTONEX_ACTORS,
AUTONEX_SENSORS, AUTONEX_PATHS, AUTONEX_OVERLAYS, AUTONEX_CAMERAS,
AUTONEX_LIGHTING. AUTONEX_SENSORS is intentionally reserved and empty.

Cameras: CAM_HERO_CHASE, CAM_TOP_TECH, CAM_FRONT_SENSOR, CAM_SIDE_TRACK.
Chase/front/side use fixed offsets from ego; top follows ego X without heading
roll. There are no cinematic motion curves. Top camera shows the local road,
truth actors, tracks and real returned candidate geometry.

## Technical layers

- Green: selected path copied from `source.selected.trajectory`; main candidates
  marked selected also retain their actual flags. No path chosen for appearance.
- Red: candidate whose recorded combined safe flag is false. Grey/cyan: safe
  candidate. All returned candidates are pooled; coordinates are copied exactly.
- Blue: full **recorded** ego trace, explicitly labelled hindsight, not prediction.
- Yellow rings: confirmed track positions. Yellow arrows: recorded velocity
  vectors at 0.5 metres per m/s; arrow length is a display scale, not a forecast.
- HUD: actual sample decision, Guardian state and latest recorded events with
  their original timestamps. Materials for CRUISE/REPLAN/BRAKE are reusable.
- Track covariance exists in source, but this stage does not reconstruct an
  uncertainty region. Sensor frusta/rays are not fabricated.
- Object predictions, safety bubbles, safe-space/occupancy mesh and rejected
  paths discarded before export are unavailable and remain absent.

Geometry is deliberately simple and replaceable. Replace EGO_ROOT's mesh children
with an unbranded asset aligned +X, metres, origin at ground-projected vehicle
centre; keep the root, its identity and telemetry driver. No licensed external
assets, branded models, physics simulations, volumetrics or dense geometry.

## Validation / limitations

Setup checks 201 samples, monotonic time, finite ego states, independent CSV
agreement, absent unsupported
geometry, displayed ego values, every candidate polyline/flag, and stable object
count through frames 1–301. `references/validation.json` records the result and
source hash. Blender mesh positions use float precision (tolerance 0.1 mm).
Playback is handler-driven rather than baked keyframes; activate the bundled
text after opening. The source JSON must remain accessible. The viewport may
drop frames on a laptop; frame-to-time mapping is unchanged. Development lighting
and low-detail assets are not final cinematic assets.

Handlers update data during frame changes; the script enables Blender's render
interface lock as advised by the official Blender application-handler API:
https://docs.blender.org/api/5.2/bpy.app.handlers.html
