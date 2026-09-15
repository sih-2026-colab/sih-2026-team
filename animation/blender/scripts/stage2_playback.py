"""Stage 2 presentation wrapper around the preserved Stage 1 telemetry driver."""
import bpy
import math
from mathutils import Vector


def activate():
    namespace={'__name__':'autonex_stage1_preserved'}
    exec(compile(bpy.data.texts['AUTONEX_PLAYBACK.py'].as_string(),'AUTONEX_PLAYBACK.py','exec'),namespace)
    telemetry, original=namespace['activate']()
    scene=bpy.context.scene
    bpy.app.handlers.frame_change_post.remove(original)
    for fn in list(bpy.app.handlers.frame_change_post):
        if getattr(fn,'_autonex_stage2',False): bpy.app.handlers.frame_change_post.remove(fn)

    def update(scene,*unused):
        frame=scene.frame_current
        if scene.get('stage2_camera_sequence',True):
            name='CAM_HERO_CHASE' if frame<61 or frame>=241 else ('CAM_TOP_TECH' if frame<151 else 'CAM_SIDE_TRACK')
            scene.camera=bpy.data.objects[name]
        original(scene)
        source=telemetry.sample(frame)['source']
        bpy.data.objects['LIVE_TELEMETRY'].hide_render=True
        bpy.data.objects['LIVE_TELEMETRY'].hide_viewport=True
        bpy.data.objects['EGO_EXECUTED_TRACE'].hide_render=True
        bpy.data.objects['EGO_EXECUTED_TRACE'].hide_viewport=True
        for i,c in enumerate(namespace['array'](source['candidates'])):
            obj=bpy.data.objects[f'CANDIDATE_{i:02d}']
            obj.data.bevel_depth=.055 if c['selected'] else (.028 if not c['safe'] else .013)
            obj.data.bevel_resolution=2
        for id in scene['track_ids']:
            bpy.data.objects[f'TRACK_{id}'].hide_render=True
            bpy.data.objects[f'TRACK_{id}'].hide_viewport=True
            matches=[tr for tr in namespace['array'](source['tracks']) if tr['id']==id]
            objects=[bpy.data.objects[f'S2_TRACKBOX_{id}'],bpy.data.objects[f'S2_TRACKLABEL_{id}'],bpy.data.objects[f'S2_RELATIVE_{id}']]
            for obj in objects: obj.hide_render=obj.hide_viewport=not bool(matches)
            if not matches: continue
            tr=matches[0]; box,label,relative=objects
            # Fixed glyph dimensions are not estimated physical object extents.
            box.location=(tr['x'],tr['y'],.1)
            label.location=(tr['x'],tr['y']+1.6,2.4)
            label.rotation_euler=scene.camera.rotation_euler if scene.camera.parent is None else scene.camera.matrix_world.to_euler()
            label.data.body=f"T{id}  {tr['class'].upper()}\n{tr['rangeM']:.1f} m | dVx {tr['relativeVxMps']:+.1f} m/s"
            label['source_time_s']=source['time']; box['source_time_s']=source['time']
            x,y=tr['x'],tr['y']; vx,vy=tr['relativeVxMps'],tr['relativeVyMps']
            end=(x+.35*vx,y+.35*vy,.28); angle=math.atan2(vy,vx)
            coords=[(x,y,.28),end,(end[0]-.4*math.cos(angle-.45),end[1]-.4*math.sin(angle-.45),.28),end,
                    (end[0]-.4*math.cos(angle+.45),end[1]-.4*math.sin(angle+.45),.28)]
            for point,xyz in zip(relative.data.splines[0].points,coords): point.co=(*xyz,1)
        # Normalized camera-plane layout; consistent dimensions for all four cameras.
        cam=scene.camera; distance=2
        width=cam.data.ortho_scale if cam.data.type=='ORTHO' else 2*distance*math.tan(cam.data.angle_x/2)
        height=width*scene.render.resolution_y/scene.render.resolution_x
        for name,x,y,size in [('S2_TITLE',-.455,.425,.022),('S2_STATE',-.455,.347,.033),
                              ('S2_INFO',-.455,.285,.016),('S2_FOOTER',-.455,-.44,.014)]:
            obj=bpy.data.objects[name]; obj.parent=cam; obj.location=(x*width,y*height,-distance)
            obj.rotation_euler=(0,0,0); obj.scale=(width*size,)*3
        panel=bpy.data.objects['S2_HUD_PANEL']; panel.parent=cam
        panel.location=(-.278*width,.266*height,-distance-.025); panel.rotation_euler=(0,0,0)
        panel.scale=(width*.40,height*.405,1)
        state=source['decision']; bpy.data.objects['S2_STATE'].data.body=f"{state}   /   {source['ego']['speedMps']*3.6:.1f} km/h"
        bpy.data.objects['S2_STATE'].data.materials.clear()
        bpy.data.objects['S2_STATE'].data.materials.append(bpy.data.materials.get('S2_'+state,bpy.data.materials['S2_WHITE']))
        selected=source['selected']; candidates=namespace['array'](source['candidates'])
        chosen=next((c for c in candidates if c['selected']),None)
        riskmode=source['source']['minimumRiskMode']
        title='SELECTED PATH' if selected else 'NO SELECTED PATH'
        if selected and selected.get('minimumRiskMode') in ('LATERAL_ESCAPE','CREEP','CONTROLLED_BRAKE'):
            title='SELECTED MIN-RISK PATH'
        intervention=source['guardian'] not in ('APPROVED','NO_ACTION')
        lines=[f"t {scene['display_time_s']:05.2f}s  |  source {source['time']:05.2f}s",title,
               f"{sum(not c['safe'] for c in candidates)} CANDIDATE REJECTED / {len(candidates)}",
               ('GUARDIAN INTERVENTION' if intervention else 'GUARDIAN APPROVED'),source['guardian']]
        if chosen:
            g=chosen['guardian']; ttc=g['minFrontTTC']
            lines.append(f"Selected front TTC  {ttc:.2f}s" if isinstance(ttc,(int,float)) and math.isfinite(ttc) else 'Selected front TTC  unavailable')
        else: lines.append('Selected front TTC  unavailable')
        bpy.data.objects['S2_INFO'].data.body='\n'.join(lines)
        bpy.data.objects['S2_FOOTER'].data.body='GREEN selected | RED rejected | YELLOW tracks | MAGENTA relative velocity\nSensor arcs: conceptual coverage, NOT detections. Safety mesh / predictions: unavailable.'
        scene['stage2_decision']=state; scene['stage2_guardian_intervention']=intervention
    update._autonex_stage2=True
    bpy.app.handlers.frame_change_post.append(update)
    bpy.app.driver_namespace['AUTONEX_STAGE2_UPDATE']=update
    update(scene)
    return telemetry,update

if __name__=='__main__': activate()
