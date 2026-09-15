"""Run with the generated .blend open. No simulation or telemetry writes."""
import bpy
import json
from pathlib import Path

scene=bpy.data.scenes['AUTONEX_STAGE1']; bpy.context.window.scene=scene
exec(compile(bpy.data.texts['AUTONEX_PLAYBACK.py'].as_string(),'AUTONEX_PLAYBACK.py','exec'),{'__name__':'__main__'})
before=len(scene.objects)
for frame in (1,2,4,91,181,300,301):
    scene.frame_set(frame)
    assert abs(scene['display_time_s']-(frame-1)/30)<1e-9
    assert len(scene.objects)==before
    assert scene['safety_mesh_available'] is False and scene['object_predictions_available'] is False
assert scene.frame_end==300 and scene.render.fps==30 and scene.render.engine=='BLENDER_EEVEE'
folder=Path(bpy.data.filepath).parent.parent/'references'
(folder/'reopen_validation.json').write_text(json.dumps({'passed':True,'playback_reactivated':True,
    'frames_checked':[1,2,4,91,181,300,301],'objects':before},indent=2))
print('AUTONEX_SAVED_SCENE_REOPEN_PASS')
