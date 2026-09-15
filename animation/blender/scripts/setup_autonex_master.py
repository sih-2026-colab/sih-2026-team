"""AutoNex Stage 1. New standalone scene; never modifies source telemetry."""
import bpy
import csv
import importlib.util
import json
import math
import random
import sys
from pathlib import Path
from mathutils import Vector

HERE=Path(__file__).resolve().parent
BASE=HERE.parent
ROOT=BASE.parent.parent
spec=importlib.util.spec_from_file_location('autonex_importer',HERE/'import_autonex_telemetry.py')
imp=importlib.util.module_from_spec(spec); spec.loader.exec_module(imp)
data=imp.Telemetry(ROOT/'results/animation/missing_lane/telemetry.json')
for folder in ('assets','materials','renders','references'):
    (BASE/folder).mkdir(exist_ok=True,parents=True)
if bpy.data.scenes.get('AUTONEX_STAGE1'):
    raise RuntimeError('Stage 1 already exists. Open the saved scene and run its playback text instead.')
scene=bpy.data.scenes.new('AUTONEX_STAGE1')
bpy.context.window.scene=scene
scene.unit_settings.system='METRIC'; scene.unit_settings.scale_length=1
scene.render.engine='BLENDER_EEVEE'
scene.render.resolution_x=1920; scene.render.resolution_y=1080; scene.render.resolution_percentage=100
scene.render.fps=30; scene.render.fps_base=1
scene.frame_start=1; scene.frame_end=300
scene.render.use_lock_interface=True
scene['telemetry_path']=str(data.path); scene['telemetry_sha256']=data.sha256
scene['road_is_presentation_geometry']=True
scene['safety_mesh_available']=False; scene['object_predictions_available']=False
COLLECTIONS=['WORLD','ROAD','EGO','ACTORS','SENSORS','PATHS','OVERLAYS','CAMERAS','LIGHTING']
collections={}
for name in COLLECTIONS:
    c=bpy.data.collections.new('AUTONEX_'+name); scene.collection.children.link(c); collections[name]=c

def link(obj, group):
    for c in list(obj.users_collection): c.objects.unlink(obj)
    collections[group].objects.link(obj)
    return obj

def material(name,color,metal=0,rough=.65):
    m=bpy.data.materials.new(name); m.diffuse_color=(*color,1); m.use_nodes=True
    p=m.node_tree.nodes.get('Principled BSDF'); p.inputs['Base Color'].default_value=(*color,1)
    p.inputs['Metallic'].default_value=metal; p.inputs['Roughness'].default_value=rough
    return m

mats={
    'silver':material('VEHICLE_SILVER',(.48,.52,.55),.7,.3),
    'glass':material('DARK_GLASS',(.035,.07,.08),.35,.2),
    'rubber':material('TYRE',(.022,.025,.023)),
    'road':material('WEATHERED_ASPHALT',(.12,.125,.12)),
    'dirt':material('DRY_SOIL',(.32,.25,.16)),
    'gravel':material('GRAVEL_SHOULDER',(.38,.35,.28)),
    'green':material('VEGETATION',(.17,.25,.10)),
    'building':material('PLASTER',(.60,.52,.39)),
    'pole':material('CONCRETE',(.4,.4,.37)),
    'actor':material('ACTOR_PAINT',(.23,.30,.36),.45,.4),
    'white':material('HUD_WHITE',(.86,.9,.90)),
}
for name,col in [('PATH_SELECTED',(.12,.65,.22)),('PATH_REJECTED',(.72,.09,.045)),
                 ('PATH_SAFE',(.24,.40,.43)),('TRACK_YELLOW',(.95,.65,.06)),('EGO_TRACE',(.08,.40,.58)),
                 ('CRUISE',(.13,.55,.20)),('REPLAN',(.9,.5,.06)),('BRAKE',(.8,.13,.045))]:
    material(name,col,0,.45)
# Sparse procedural asphalt texture; presentation only.
nodes=mats['road'].node_tree.nodes; links=mats['road'].node_tree.links
noise=nodes.new('ShaderNodeTexNoise'); noise.inputs['Scale'].default_value=32
bump=nodes.new('ShaderNodeBump'); bump.inputs['Strength'].default_value=.16; bump.inputs['Distance'].default_value=.015
links.new(noise.outputs['Fac'],bump.inputs['Height']); links.new(bump.outputs['Normal'],nodes.get('Principled BSDF').inputs['Normal'])

def box(name,loc,scale,mat,group,parent=None,bevel=0):
    bpy.ops.mesh.primitive_cube_add(size=1,location=loc)
    o=link(bpy.context.object,group); o.name=name; o.dimensions=scale
    bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
    o.data.materials.append(mat)
    if bevel:
        b=o.modifiers.new('Soft manufactured edges','BEVEL'); b.width=bevel; b.segments=2
    if parent: o.parent=parent
    return o

def vehicle(name,group,paint):
    root=bpy.data.objects.new(name,None); collections[group].objects.link(root)
    root['asset_role']='REPLACEABLE_UNBRANDED_PASSENGER_CAR'; root['length_m']=4.5; root['width_m']=1.9
    box(name+'_body',(0,0,.68),(4.5,1.9,.75),paint,group,root,.12)
    box(name+'_cabin',(-.2,0,1.26),(2.35,1.62,.68),mats['glass'],group,root,.20)
    box(name+'_roof',(-.28,0,1.60),(1.75,1.55,.08),paint,group,root,.06)
    for x in (-1.43,1.40):
        for y in (-.91,.91):
            bpy.ops.mesh.primitive_cylinder_add(vertices=16,radius=.34,depth=.22,location=(x,y,.35),rotation=(math.pi/2,0,0))
            o=link(bpy.context.object,group); o.name=name+'_wheel'; o.data.materials.append(mats['rubber']); o.parent=root
    for y in (-.63,.63):
        box(name+'_headlamp',(2.26,y,.75),(.025,.45,.17),mats['white'],group,root)
    return root

box('GROUND',(100,4,-.25),(340,100,.4),mats['dirt'],'WORLD')
box('ASPHALT_300m_6m',(100,5.25,-.025),(300,6,.10),mats['road'],'ROAD')
rng=random.Random(26037)
for x in range(-50,250,5):
    for y in (1.65,8.85):
        o=box('Irregular_gravel_shoulder',(x+2.5,y,-.04),(5.1,1.3+rng.random()*.55,.10),mats['gravel'],'ROAD')
        o.rotation_euler.z=rng.uniform(-.015,.015)
for x in range(-35,240,25):
    for y in (-14,22):
        box('Utility_pole',(x,y,4),( .23,.23,8),mats['pole'],'WORLD')
        box('Crossarm',(x,y,7.4),(.15,1.8,.15),mats['pole'],'WORLD')
    if x%2:
        box('Roadside_plaster_building',(x,-25,2.5),(8,7,5),mats['building'],'WORLD',bevel=.06)
        box('Shop_opening',(x,-21.45,1.2),(3,.08,2.4),mats['glass'],'WORLD')
for i in range(70):
    x=rng.uniform(-45,245); y=rng.choice((-1,1))*rng.uniform(15,36)+4
    box('Tree_trunk',(x,y,1.2),(.25,.25,2.4),mats['dirt'],'WORLD')
    bpy.ops.mesh.primitive_ico_sphere_add(subdivisions=1,radius=rng.uniform(1.2,2.2),location=(x,y,3.0))
    o=link(bpy.context.object,'WORLD'); o.name='Lightweight_tree'; o.scale.z=1.35; o.data.materials.append(mats['green'])
ego=vehicle('EGO_ROOT','EGO',mats['silver'])
for a in imp.array(data.frames[0]['source']['actors']):
    if a['id']!=1:
        o=vehicle(f"ACTOR_{a['id']}",'ACTORS',mats['actor']); o['truth_actor_id']=a['id']; o['source_name']=a['name']

def line(name,count,mat,width):
    curve=bpy.data.curves.new(name,'CURVE'); curve.dimensions='3D'; curve.bevel_depth=width; curve.bevel_resolution=0
    s=curve.splines.new('POLY'); s.points.add(count-1)
    o=bpy.data.objects.new(name,curve); collections['PATHS'].objects.link(o); curve.materials.append(bpy.data.materials[mat]); return o

max_candidates=max(len(imp.array(f['candidates'])) for f in data.frames)
max_points=max(len(c['trajectory']['x']) for f in data.frames for c in imp.array(f['candidates']))
scene['candidate_pool_size']=max_candidates
for i in range(max_candidates): line(f'CANDIDATE_{i:02d}',max_points,'PATH_SAFE',.022)
line('SELECTED_PATH',max_points,'PATH_SELECTED',.065)
trace=line('EGO_EXECUTED_TRACE',len(data.frames),'EGO_TRACE',.025)
for p,f in zip(trace.data.splines[0].points,data.frames): p.co=(f['ego']['x'],f['ego']['y'],.07,1)
trace['meaning']='FULL RECORDED EGO TRACE; hindsight, not prediction'
ids=sorted({t['id'] for f in data.frames for t in imp.array(f['tracks'])}); scene['track_ids']=ids
for id in ids:
    bpy.ops.mesh.primitive_torus_add(major_radius=1.05,minor_radius=.035,major_segments=20,minor_segments=6)
    o=link(bpy.context.object,'OVERLAYS'); o.name=f'TRACK_{id}'; o.data.materials.append(bpy.data.materials['TRACK_YELLOW'])
    line(f'VELOCITY_{id}',5,'TRACK_YELLOW',.025)

def camera(name,loc,target,parent=None,ortho=None):
    d=bpy.data.cameras.new(name); o=bpy.data.objects.new(name,d); collections['CAMERAS'].objects.link(o)
    o.location=loc; o.rotation_euler=(Vector(target)-Vector(loc)).to_track_quat('-Z','Y').to_euler()
    d.lens=35; d.clip_end=1000
    if parent: o.parent=parent
    if ortho: d.type='ORTHO'; d.ortho_scale=ortho
    return o
camera('CAM_HERO_CHASE',(-13,-10,8),(9,0,.6),ego)
top=camera('CAM_TOP_TECH',(17,4,60),(17,4,0),ortho=72)
camera('CAM_FRONT_SENSOR',(2.3,0,1.5),(30,0,1.3),ego)
camera('CAM_SIDE_TRACK',(0,-17,4),(4,0,.8),ego)
scene.camera=top
font=bpy.data.curves.new('Telemetry_HUD','FONT'); font.size=1; font.space_line=1.3
label=bpy.data.objects.new('LIVE_TELEMETRY',font); collections['OVERLAYS'].objects.link(label)
font.materials.append(mats['white'])
world=bpy.data.worlds.new('Soft_daylight'); scene.world=world; world.use_nodes=True
world.node_tree.nodes['Background'].inputs[0].default_value=(.62,.72,.85,1)
world.node_tree.nodes['Background'].inputs[1].default_value=.45
light=bpy.data.lights.new('Morning_sun','SUN'); light.energy=2.4; light.angle=math.radians(12)
o=bpy.data.objects.new('Morning_sun',light); collections['LIGHTING'].objects.link(o); o.rotation_euler=(.45,-.5,-.35)
scene.view_settings.view_transform='AgX'
text=bpy.data.texts.new('AUTONEX_PLAYBACK.py'); text.write((HERE/'import_autonex_telemetry.py').read_text())
text['instructions']='Run this text after opening blend to activate telemetry playback. No automatic trust preference changes.'
telemetry,update=imp.activate()
# Independent CSV cross-check: do not rely only on the importer's own sampler.
with (ROOT/'results/animation/missing_lane/ego.csv').open(newline='') as f:
    csv_ego=list(csv.DictReader(f))
assert len(csv_ego)==201
# Proof: all display samples match source interpolation; overlays match held snapshots.
audit=[]; count=len(scene.objects)
for frame in range(1,302):
    scene.frame_set(frame); s=data.sample(frame)
    t=(frame-1)/30
    n=min(int(math.floor((t+1e-12)/.05)),200); m=min(n+1,200)
    weight=0 if n==m else (t-float(csv_ego[n]['time']))/(float(csv_ego[m]['time'])-float(csv_ego[n]['time']))
    for axis in ('x','y'):
        expected=float(csv_ego[n][axis])+weight*(float(csv_ego[m][axis])-float(csv_ego[n][axis]))
        assert abs(getattr(ego.location,axis)-expected)<1e-5
    h0=float(csv_ego[n]['heading_rad']); h1=float(csv_ego[m]['heading_rad'])
    expected=h0+weight*math.atan2(math.sin(h1-h0),math.cos(h1-h0))
    assert abs(ego.rotation_euler.z-expected)<1e-6
    assert abs(ego.location.x-s['ego']['x'])<1e-5 and abs(ego.location.y-s['ego']['y'])<1e-5
    for i,c in enumerate(imp.array(s['source']['candidates'])):
        obj=bpy.data.objects[f'CANDIDATE_{i:02d}']
        assert obj['safe']==c['safe'] and obj['selected']==c['selected']
        for point,x,y in zip(obj.data.splines[0].points,c['trajectory']['x'],c['trajectory']['y']):
            assert abs(point.co.x-x)<1e-4 and abs(point.co.y-y)<1e-4
    assert len(scene.objects)==count
    audit.append(dict(frame=frame,time=s['time'],source_lower=s['lower'],source_upper=s['upper'],alpha=s['alpha']))
with (BASE/'references/frame_provenance.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=list(audit[0])); w.writeheader(); w.writerows(audit)
proof=next((i for i in range(1,301) if data.sample(i)['source']['selected']
            and data.sample(i)['source']['tracks']
            and any(not c['safe'] for c in imp.array(data.sample(i)['source']['candidates']))),1)
scene.frame_set(proof)
for screen in bpy.data.screens:
    for area in screen.areas:
        if area.type=='VIEW_3D': area.spaces.active.region_3d.view_perspective='CAMERA'
bpy.context.preferences.filepaths.save_version=0  # this process only; no preferences saved
bpy.ops.wm.save_as_mainfile(filepath=str(BASE/'assets/autonex_missing_lane_stage1.blend'))
(BASE/'references/validation.json').write_text(json.dumps(dict(passed=True,blender=bpy.app.version_string,
    source=str(data.path),sha256=data.sha256,samples=201,frames_validated=301,render_frames=300,fps=30,
    duration_seconds=10,proof_frame=proof,stable_object_count=count,independent_ego_csv_match=True,
    unavailable_geometry_fabricated=False),indent=2))
if '--proof' in sys.argv:
    scene.render.filepath=str(BASE/'renders/stage1_top_proof.png'); bpy.ops.render.render(write_still=True)
    scene.camera=bpy.data.objects['CAM_HERO_CHASE']; update(scene)
    scene.render.filepath=str(BASE/'renders/stage1_hero_proof.png'); bpy.ops.render.render(write_still=True)
print('AUTONEX_BLENDER_STAGE1_PASS')
