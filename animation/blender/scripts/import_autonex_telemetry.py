"""Read-only presentation sampling. No MATLAB, planner or risk calculation."""
import bisect
import hashlib
import json
import math
from pathlib import Path


def array(value):
    return value if isinstance(value, list) else [value]


class Telemetry:
    def __init__(self, filename):
        self.path = Path(filename).resolve()
        raw = self.path.read_bytes()
        self.sha256 = hashlib.sha256(raw).hexdigest()
        self.data = json.loads(raw, parse_constant=lambda s: (_ for _ in ()).throw(ValueError(s)))
        assert self.data['scenario'] == 'missing_lane'
        self.frames = array(self.data['frames'])
        self.times = [f['time'] for f in self.frames]
        assert len(self.times) == 201 and abs(self.times[-1] - 10) < 1e-9
        assert all(b > a for a, b in zip(self.times, self.times[1:]))
        self.events = array(self.data['events'])
        for f in self.frames:
            assert all(math.isfinite(f['ego'][k]) for k in ('x','y','headingRad','speedMps'))
            assert not f['predictedObjectTrajectories'] and not f['safetyBubble']

    def sample(self, frame):
        # 300 presentation frames at 30 fps. Frame 301 is the non-rendered endpoint.
        t = min(10., max(0., (frame - 1) / 30.))
        lo = max(0, bisect.bisect_right(self.times, t + 1e-12) - 1)
        hi = min(lo + 1, len(self.times) - 1)
        a, b = self.frames[lo], self.frames[hi]
        alpha = 0 if hi == lo else max(0., (t-self.times[lo])/(self.times[hi]-self.times[lo]))
        def lerp(x, y):
            return x + alpha * (y-x)
        def angle(x, y):
            return x + alpha * math.atan2(math.sin(y-x), math.cos(y-x))
        ego = {k: lerp(a['ego'][k], b['ego'][k]) for k in ('x','y','speedMps')}
        ego['headingRad'] = angle(a['ego']['headingRad'], b['ego']['headingRad'])
        actors = []
        later = {v['id']: v for v in array(b['source']['actors'])}
        for actor in array(a['source']['actors']):
            if actor['id'] == 1:
                continue
            v = dict(actor); nxt = later.get(v['id'], v)
            for key in ('x','y'):
                v[key] = lerp(v[key], nxt[key])
            v['heading'] = angle(v['heading'], nxt['heading'])
            actors.append(v)
        return dict(time=t, lower=self.times[lo], upper=self.times[hi], alpha=alpha,
                    source_index=lo, ego=ego, actors=actors, source=a)


def activate():
    """Reconnect dynamic overlays when opening the saved blend; no asset rebuild."""
    import bpy
    from mathutils import Vector
    scene = bpy.context.scene
    source = bpy.path.abspath(scene['telemetry_path'])
    telemetry = Telemetry(source)
    assert telemetry.sha256 == scene['telemetry_sha256'], 'Source telemetry changed'

    def show_line(name, points, material=None):
        obj = bpy.data.objects[name]
        obj.hide_render = obj.hide_viewport = not bool(points)
        if not points:
            return
        spline = obj.data.splines[0]
        assert len(points) <= len(spline.points)
        padded = list(points) + [points[-1]] * (len(spline.points)-len(points))
        for p, xyz in zip(spline.points, padded):
            p.co = (*xyz, 1)
        if material:
            obj.data.materials.clear(); obj.data.materials.append(bpy.data.materials[material])

    def update(scene, *unused):
        s = telemetry.sample(scene.frame_current)
        scene['display_time_s'] = s['time']
        scene['source_lower_s'] = s['lower']; scene['source_upper_s'] = s['upper']
        scene['interpolation_alpha'] = s['alpha']
        ego = bpy.data.objects['EGO_ROOT']
        ego.location = (s['ego']['x'],s['ego']['y'],0)
        ego.rotation_euler.z = s['ego']['headingRad']
        for key in ('time','lower','upper','alpha'):
            ego['telemetry_'+key]=s[key]
        for actor in s['actors']:
            obj = bpy.data.objects[f"ACTOR_{actor['id']}"]
            obj.location = (actor['x'],actor['y'],0); obj.rotation_euler.z = actor['heading']
            for key in ('time','lower','upper','alpha'):
                obj['telemetry_'+key]=s[key]
        source = s['source']
        for i in range(scene['candidate_pool_size']):
            name = f'CANDIDATE_{i:02d}'
            candidates = array(source['candidates'])
            if i >= len(candidates):
                show_line(name, []); continue
            c = candidates[i]
            material = 'PATH_SELECTED' if c['selected'] else ('PATH_SAFE' if c['safe'] else 'PATH_REJECTED')
            tr = c['trajectory']; xyz = list(zip(tr['x'],tr['y'],[.10]*len(tr['x'])))
            show_line(name, xyz, material)
            obj = bpy.data.objects[name]; obj.data.bevel_depth = .065 if c['selected'] else .022
            obj['source_candidate_id'] = c['id']; obj['source_time_s'] = source['time']
            obj['safe'] = c['safe']; obj['selected'] = c['selected']
        selected = source['selected']
        tr = selected['trajectory'] if selected else None
        show_line('SELECTED_PATH', list(zip(tr['x'],tr['y'],[.13]*len(tr['x']))) if tr else [])
        tracks = {v['id']:v for v in array(source['tracks'])}
        for track_id in scene['track_ids']:
            obj = bpy.data.objects[f'TRACK_{track_id}']; tr = tracks.get(track_id)
            obj.hide_render = obj.hide_viewport = tr is None
            if tr:
                obj.location = (tr['x'],tr['y'],1.9)
                obj['source_time_s'] = source['time']
                x,y,vx,vy = tr['x'],tr['y'],tr['vx'],tr['vy']
                end = (x+vx*.5,y+vy*.5,.25)
                direction = math.atan2(vy,vx); tip=.35
                points=[(x,y,.25),end,(end[0]-tip*math.cos(direction-.5),end[1]-tip*math.sin(direction-.5),.25),end,
                        (end[0]-tip*math.cos(direction+.5),end[1]-tip*math.sin(direction+.5),.25)]
                show_line(f'VELOCITY_{track_id}',points)
            else:
                show_line(f'VELOCITY_{track_id}',[])
        # Technical camera follows position only: no invented heading or camera roll.
        top=bpy.data.objects['CAM_TOP_TECH']; top.location=(s['ego']['x']+17,4,60)
        recent=[e for e in telemetry.events if e['time'] <= s['time']+1e-12][-3:]
        body = 'AUTONEX  /  SIH26037\nMISSING LANE  |  TELEMETRY PROOF\n'
        body += f"{s['time']:05.2f} s   {source['decision']}   {s['ego']['speedMps']*3.6:04.1f} km/h\n"
        body += f"GUARDIAN: {source['guardian']}\n"
        body += 'GREEN selected / RED rejected / GREY safe\n'
        body += 'BLUE recorded ego trace (not prediction)\n'
        body += '\n'.join(f"{e['time']:05.2f}  {e['event']}" for e in recent)
        body += '\nPrediction / safety mesh: unavailable'
        label=bpy.data.objects['LIVE_TELEMETRY']; label.data.body=body
        # A camera-parented text panel for each view; selected active camera controls HUD.
        label.parent=scene.camera
        label.location=(-.90,.43,-2.0); label.rotation_euler=(0,0,0)
        label.scale=(.033,.033,.033) if scene.camera.type == 'CAMERA' and scene.camera.data.type != 'ORTHO' else (.70,.70,.70)
        if scene.camera.data.type == 'ORTHO':
            label.location=(-29,15,-2)
        scene['decision_source_s']=source['time']
    update._autonex_stage1 = True
    for handler in list(bpy.app.handlers.frame_change_post):
        if getattr(handler,'_autonex_stage1',False):
            bpy.app.handlers.frame_change_post.remove(handler)
    bpy.app.handlers.frame_change_post.append(update)
    bpy.app.driver_namespace['AUTONEX_STAGE1_TELEMETRY'] = telemetry
    update(scene)
    return telemetry, update


if __name__ == '__main__':
    activate()
