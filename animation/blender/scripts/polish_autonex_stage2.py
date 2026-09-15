"""Polish an OPEN Stage 1 blend. Never executes setup_autonex_master.py."""
import bpy
import math
import json
import random
import sys
from pathlib import Path
from mathutils import Vector

BASE=Path(__file__).resolve().parent.parent
scene=bpy.data.scenes['AUTONEX_STAGE1']; bpy.context.window.scene=scene
assert not scene.get('stage2_polished'), 'Open the original Stage 1 file first'
scene['stage2_polished']=True; scene['stage2_camera_sequence']=True
scene.render.engine='BLENDER_EEVEE'; scene.render.resolution_percentage=100
scene.render.use_lock_interface=True
scene.world.node_tree.nodes['Background'].inputs[0].default_value=(.50,.64,.82,1)
scene.world.node_tree.nodes['Background'].inputs[1].default_value=.38
sun=bpy.data.objects['Morning_sun']; sun.rotation_euler=(.85,-.45,-.55)
sun.data.energy=2.5; sun.data.color=(1,.86,.70); sun.data.angle=math.radians(8)
if hasattr(scene.eevee,'use_raytracing'): scene.eevee.use_raytracing=True
if hasattr(scene.render,'use_motion_blur'): scene.render.use_motion_blur=False

def mat(name,color,metal=0,rough=.5,emission=0):
    m=bpy.data.materials.get(name) or bpy.data.materials.new(name); m.use_nodes=True
    m.diffuse_color=(*color,1); p=m.node_tree.nodes.get('Principled BSDF')
    p.inputs['Base Color'].default_value=(*color,1); p.inputs['Metallic'].default_value=metal; p.inputs['Roughness'].default_value=rough
    if emission: p.inputs['Emission Color'].default_value=(*color,1); p.inputs['Emission Strength'].default_value=emission
    return m

def link(o,group):
    for c in list(o.users_collection): c.objects.unlink(o)
    bpy.data.collections['AUTONEX_'+group].objects.link(o); return o

def box(name,xyz,dims,material,group='WORLD',parent=None,bevel=.02):
    bpy.ops.mesh.primitive_cube_add(size=1,location=xyz); o=link(bpy.context.object,group); o.name=name; o.dimensions=dims
    bpy.ops.object.transform_apply(location=False,rotation=False,scale=True); o.data.materials.append(material)
    if parent: o.parent=parent
    if bevel: mod=o.modifiers.new('Soft edges','BEVEL'); mod.width=bevel; mod.segments=2
    return o

def line(name,points,material,group='WORLD',width=.02,parent=None):
    c=bpy.data.curves.new(name,'CURVE'); c.dimensions='3D'; c.bevel_depth=width; c.bevel_resolution=2
    s=c.splines.new('POLY'); s.points.add(len(points)-1)
    for p,xyz in zip(s.points,points): p.co=(*xyz,1)
    o=bpy.data.objects.new(name,c); bpy.data.collections['AUTONEX_'+group].objects.link(o); c.materials.append(material)
    if parent: o.parent=parent
    return o

def label(name,body,size,material,group='OVERLAYS'):
    c=bpy.data.curves.new(name,'FONT'); c.body=body; c.size=size; c.space_line=1.3
    o=bpy.data.objects.new(name,c); bpy.data.collections['AUTONEX_'+group].objects.link(o); c.materials.append(material); return o

silver=mat('VEHICLE_SILVER',(.58,.62,.66),.8,.23)
glass=mat('DARK_GLASS',(.025,.05,.065),.45,.15)
black=mat('S2_TRIM',(.018,.023,.028),.15,.35); chrome=mat('S2_ALLOY',(.38,.42,.45),.85,.2)
red=mat('S2_TAIL_LAMP',(.45,.015,.008),.2,.22,.2); head=mat('S2_HEADLAMP',(.8,.9,1),.4,.15,.15)
white=mat('S2_WHITE',(.88,.93,.96),0,.8,1)
for name,color in [('CRUISE',(.15,.7,.34)),('REPLAN',(.9,.57,.10)),('BRAKE',(.95,.25,.1)),('STOP',(.95,.15,.06))]: mat('S2_'+name,color,0,.5,.6)
yellow=mat('S2_TRACK',(.85,.56,.10),0,.5,.25); pink=mat('S2_RELATIVE',(.64,.27,.45),0,.5,.15)
dark=mat('S2_HUD',(.008,.022,.03),0,1,.5)
for name,color in [('PATH_SELECTED',(.08,.58,.22)),('PATH_REJECTED',(.65,.07,.03)),('PATH_SAFE',(.16,.3,.32))]: mat(name,color,0,.5,.25)

# Small procedural surfaces; no image textures or heavy scatter systems.
def texture(name,c1,c2,scale,rough):
    m=bpy.data.materials[name]; p=m.node_tree.nodes.get('Principled BSDF'); p.inputs['Roughness'].default_value=rough
    n=m.node_tree.nodes.new('ShaderNodeTexNoise'); n.inputs['Scale'].default_value=scale; n.inputs['Detail'].default_value=2
    ramp=m.node_tree.nodes.new('ShaderNodeValToRGB'); ramp.color_ramp.elements[0].color=(*c1,1); ramp.color_ramp.elements[1].color=(*c2,1)
    m.node_tree.links.new(n.outputs['Fac'],ramp.inputs['Fac']); m.node_tree.links.new(ramp.outputs['Color'],p.inputs['Base Color'])
texture('WEATHERED_ASPHALT',(.055,.065,.07),(.18,.19,.19),75,.87)
texture('DRY_SOIL',(.16,.10,.055),(.40,.29,.16),4,.95)
texture('GRAVEL_SHOULDER',(.18,.16,.12),(.40,.36,.27),48,.95)
patch=mat('S2_ASPHALT_REPAIR',(.07,.078,.078),0,.85)
rng=random.Random(260372)
for i in range(30):
    o=box('S2_Road_repair',(rng.uniform(-40,240),rng.uniform(3,7.5),.032),(rng.uniform(.6,2.8),rng.uniform(.3,.8),.006),patch,'ROAD',bevel=.06)
    o.rotation_euler.z=rng.uniform(-.2,.2)
# Shared smoother tree crown mesh; replace faceted Stage 1 crowns only.
bpy.ops.mesh.primitive_uv_sphere_add(segments=12,ring_count=8,radius=1)
prototype=bpy.context.object; foliage=prototype.data
foliage.materials.append(mat('S2_FOLIAGE',(.11,.19,.055),0,.9))
for p in foliage.polygons: p.use_smooth=True
prototype.hide_render=True; prototype.hide_viewport=True; link(prototype,'WORLD'); prototype.name='S2_Foliage_mesh_template'
crowns=[o for o in list(scene.objects) if o.name.startswith('Lightweight_tree')]
for o in crowns:
    old_radius=max(v.co.length for v in o.data.vertices); o.data=foliage; o.scale=(old_radius,old_radius*.9,old_radius*1.2)
    for j in range(2):
        leaf=o.copy(); leaf.data=foliage; bpy.data.collections['AUTONEX_WORLD'].objects.link(leaf)
        leaf.location.x+=rng.uniform(-1,1); leaf.location.y+=rng.uniform(-1,1); leaf.location.z+=rng.uniform(-.7,.3); leaf.scale*=.72
# Roadside dressing remains outside the actual telemetry interaction area.
plaster=mat('S2_LIME_PLASTER',(.65,.53,.36),0,.9); terracotta=mat('S2_ROOF',(.28,.11,.055),0,.85)
teal=mat('S2_SHUTTER',(.07,.19,.18),.1,.65)
for x in (-15,14,56,87,125):
    box('S2_Village_shop',(x,17,2),(7,5,4),plaster)
    box('S2_Corrugated_roof',(x,17,4.15),(7.8,5.8,.20),terracotta)
    box('S2_Shop_shutter',(x,14.45,1.35),(3,.08,2.7),teal)
    for z in [i*.18+.1 for i in range(15)]: box('S2_Shutter_rib',(x,14.39,z),(3,.055,.025),black,bevel=0)
    box('S2_Awning',(x,13.8,3),(5,1.5,.1),teal)
    board=box('S2_Shop_board',(x,14.3,3.55),(5,.1,.55),dark)
    text=label('S2_Shop_sign','GENERAL STORES',.30,white,'WORLD'); text.location=(x-1.7,14.22,3.4); text.rotation_euler=(math.pi/2,0,0)
    box('S2_Compound_wall',(x+7,18,.65),(5,.25,1.3),plaster)
    for j in range(3): box('S2_Market_crate',(x-2+j*.6,13.4,.25),(.5,.5,.5),terracotta)
for y in (-14,22):
    for x in range(-35,215,25):
        for offset in (-.55,.55):
            pts=[(x+25*t/16,y+offset,7.3-.6*math.sin(math.pi*t/16)) for t in range(17)]
            line('S2_Utility_cable',pts,black,width=.012)
# A pair of decorative, explicitly non-telemetry parked two-wheelers by shops.
for x in (12,58):
    for dx in (-.65,.65):
        bpy.ops.mesh.primitive_torus_add(major_radius=.28,minor_radius=.065,major_segments=16,minor_segments=6,location=(x+dx,12.2,.34),rotation=(math.pi/2,0,0))
        o=link(bpy.context.object,'WORLD'); o.name='S2_Decorative_parked_two_wheeler'; o.data.materials.append(black); o['not_telemetry_actor']=True
    box('S2_Decorative_seat',(x,12.2,.82),(1,.33,.15),black)
    line('S2_Decorative_bike_frame',[(x-.65,12.2,.34),(x,12.2,.85),(x+.65,12.2,.34),(x+.52,12.2,1.12)],chrome,width=.035)

# Retain every existing animated root and mesh footprint; add unbranded details.
roots=[bpy.data.objects['EGO_ROOT']]+[o for o in scene.objects if o.name.startswith('ACTOR_') and o.type=='EMPTY']
for root in roots:
    group='EGO' if root.name=='EGO_ROOT' else 'ACTORS'
    for y in (-.83,.83):
        box('S2_Window_B_pillar',(-.20,y,1.27),(.09,.055,.57),black,group,root)
        box('S2_Door_handle',(-.2,y*1.14,.99),(.24,.035,.05),chrome,group,root)
        box('S2_Mirror',( .65,y*1.22,1.13),(.32,.21,.16),silver,group,root,.05)
        box('S2_Tail_light',(-2.26,y*.8,.78),(.04,.37,.2),red,group,root)
        box('S2_Headlamp',(2.265,y*.8,.78),(.04,.4,.18),head,group,root)
    box('S2_Grille',(2.27,0,.56),(.045,.95,.2),black,group,root)
    box('S2_Rear_bumper',(-2.27,0,.42),(.06,1.65,.13),black,group,root)
    for x in (-1.43,1.4):
        for y in (-1.035,1.035):
            bpy.ops.mesh.primitive_cylinder_add(vertices=24,radius=.23,depth=.025,location=(x,y,.35),rotation=(math.pi/2,0,0))
            o=link(bpy.context.object,group); o.name='S2_Alloy_wheel'; o.data.materials.append(chrome); o.parent=root
            for p in o.data.polygons: p.use_smooth=True

for id in scene['track_ids']:
    pts=[(-1,-1,0),(1,-1,0),(1,1,0),(-1,1,0),(-1,-1,0),(-1,-1,2),(1,-1,2),(1,1,2),(-1,1,2),(-1,-1,2),
         (-1,1,2),(-1,1,0),(-1,1,2),(1,1,2),(1,1,0),(1,1,2),(1,-1,2),(1,-1,0)]
    o=line(f'S2_TRACKBOX_{id}',pts,yellow,'OVERLAYS',.018); o['meaning']='Fixed 2m marker glyph; not measured dimensions'
    label(f'S2_TRACKLABEL_{id}','',.32,yellow)
    line(f'S2_RELATIVE_{id}',[(0,0,0)]*5,pink,'OVERLAYS',.023)
label('S2_TITLE','AUTONEX  /  SIH26037\nUNSTRUCTURED ROAD',1,white)
label('S2_STATE','',1,white); label('S2_INFO','',1,white); label('S2_FOOTER','',1,white)
bpy.ops.mesh.primitive_plane_add(size=1); o=link(bpy.context.object,'OVERLAYS'); o.name='S2_HUD_PANEL'; o.data.materials.append(dark)
# Deliberately modest schematic coverage arcs. Never drawn as returns/detections.
coverage=mat('S2_CONCEPT_COVERAGE',(.12,.32,.38),0,.8,.05)
ego=bpy.data.objects['EGO_ROOT']
for name,direction,span,radius in [('CAMERA_FOV',0,55,5),('FRONT_RADAR',0,95,7),('REAR_RADAR',180,95,4),
                                    ('LEFT_CORNER_RADAR',60,65,4),('RIGHT_CORNER_RADAR',-60,65,4),('LIDAR_DEPTH',0,360,2.5)]:
    arc=[(radius*math.cos(math.radians(direction-span/2+i*span/32)),radius*math.sin(math.radians(direction-span/2+i*span/32)),.08) for i in range(33)]
    o=line('S2_CONCEPT_'+name,arc,coverage,'SENSORS',.009,ego)
    o['conceptual_only']=True; o['not_calibrated_fov']=True; o['not_detection_evidence']=True
for name,loc,target in [('CAM_HERO_CHASE',(-11,-8,5.4),(9,1,.6)),('CAM_SIDE_TRACK',(-2,-15,4.2),(7,0,.8)),('CAM_FRONT_SENSOR',(2.3,0,1.6),(25,0,1.5))]:
    o=bpy.data.objects[name]; o.location=loc; o.rotation_euler=(Vector(target)-Vector(loc)).to_track_quat('-Z','Y').to_euler(); o.data.lens=32
    o.data.dof.use_dof=name=='CAM_HERO_CHASE'; o.data.dof.focus_distance=18; o.data.dof.aperture_fstop=8
bpy.data.objects['CAM_TOP_TECH'].data.dof.use_dof=False
for f,name in [(1,'CAM_HERO_CHASE'),(61,'CAM_TOP_TECH'),(151,'CAM_SIDE_TRACK'),(241,'CAM_HERO_CHASE')]:
    marker=scene.timeline_markers.new(name,frame=f); marker.camera=bpy.data.objects[name]
txt=bpy.data.texts.new('AUTONEX_STAGE2_PLAYBACK.py'); txt.write((BASE/'scripts/stage2_playback.py').read_text())
namespace={'__name__':'stage2_module'}; exec(compile(txt.as_string(),txt.name,'exec'),namespace)
telemetry,update=namespace['activate']()
count=len(scene.objects)
for frame in range(1,302):
    scene.frame_set(frame); s=telemetry.sample(frame); source=s['source']
    assert abs(ego.location.x-s['ego']['x'])<1e-5 and abs(ego.location.y-s['ego']['y'])<1e-5
    assert abs(ego.rotation_euler.z-s['ego']['headingRad'])<1e-6
    assert scene['stage2_decision']==source['decision']
    for i,c in enumerate(source['candidates']):
        obj=bpy.data.objects[f'CANDIDATE_{i:02d}']; assert obj['safe']==c['safe'] and obj['selected']==c['selected']
        for p,x,y in zip(obj.data.splines[0].points,c['trajectory']['x'],c['trajectory']['y']): assert abs(p.co.x-x)<1e-4 and abs(p.co.y-y)<1e-4
    assert len(scene.objects)==count
assert not scene['safety_mesh_available'] and not scene['object_predictions_available']
scene.frame_set(4)
bpy.context.preferences.filepaths.save_version=0
bpy.ops.wm.save_as_mainfile(filepath=str(BASE/'assets/autonex_missing_lane_stage2.blend'))
report=dict(passed=True,source_sha256=telemetry.sha256,samples=201,frames_validated=301,objects=count,
    mesh_polygons=sum(len(o.data.polygons) for o in scene.objects if o.type=='MESH'),fps=30,render_frames=300,
    motion_unchanged=True,candidates_unchanged=True,decisions_unchanged=True,unavailable_geometry_created=False)
(BASE/'references/stage2_validation.json').write_text(json.dumps(report,indent=2))
if '--proof' in sys.argv:
    scene['stage2_camera_sequence']=False
    proof=next(i for i in range(1,301) if telemetry.sample(i)['source']['selected'] and telemetry.sample(i)['source']['tracks']
               and any(not c['safe'] for c in telemetry.sample(i)['source']['candidates']))
    guardian=next(i for i in range(1,301) if telemetry.sample(i)['source']['guardian'] not in ('APPROVED','NO_ACTION'))
    for name,camera,frame in [('hero','CAM_HERO_CHASE',proof),('top','CAM_TOP_TECH',proof),('candidates','CAM_SIDE_TRACK',proof),('guardian','CAM_TOP_TECH',guardian)]:
        scene.frame_set(frame); scene.camera=bpy.data.objects[camera]; update(scene)
        scene.render.filepath=str(BASE/f'renders/stage2_{name}_proof.png'); bpy.ops.render.render(write_still=True)
    report['proof_frames']={'hero':proof,'top':proof,'candidates':proof,'guardian':guardian}
    (BASE/'references/stage2_validation.json').write_text(json.dumps(report,indent=2))
print('AUTONEX_STAGE2_PASS')
