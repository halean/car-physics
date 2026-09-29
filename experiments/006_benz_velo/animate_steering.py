"""Animate a car's steering and front suspension, posed with the same linkage solver the checks use.

blender --background <blend> --python-exit-code 1 --python animate_steering.py -- <geometry.json> <out.mp4> [hide-prefixes.json]

Two panels, body hidden: a low front-quarter view and a straight front elevation. Sequence (9 s):
the column sweeps lock to lock at ride height; then the front wheels move on their suspension
with the column straight (independent cars: one wheel up, the other down); then both at once.
Never saves the scene. Needs ffmpeg.
"""
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import bpy
from mathutils import Matrix, Vector

sys.path.insert(0, str(Path(__file__).resolve().parent))
from linkage import body_transforms, suspended_pose, kinematics

argv = sys.argv[sys.argv.index('--')+1:]
geometry = json.loads(Path(argv[0]).read_text())
out = Path(argv[1])
hide = tuple(json.loads(Path(argv[2]).read_text())) if len(argv) > 2 else ()
report = json.loads(Path(bpy.data.filepath).with_name('report.json').read_text())
scene = bpy.context.scene
objects = [o for o in bpy.data.collections[report['collection']].objects if o.type in {'MESH', 'CURVE'}]
rest = {o.name: o.matrix_world.copy() for o in objects}
steer = {'knuckle_left', 'knuckle_right', 'tie_rod', 'column', 'drag_link', 'track_rod_left', 'track_rod_right'}
suspension = ('pillar', 'lower_arm', 'upper_arm', 'upright')
body_parts = ('Seat', 'Bench', 'Body', 'Bonnet', 'Engine box', 'Dash', 'Footboard', 'Fuel tank', 'Mudguard', 'Floor',
              'Front mudguard', 'Rear mudguard', 'Front wing', 'Rear wing', 'Running board', 'Tonneau', 'Front seat',
              'Rear seat', 'Scuttle', 'Radiator', 'Handwheel') + hide
for o in objects:
    b = o['body']
    o.color = ((.8, .1, .1, 1) if 'brake' in o.name.lower() else
               (1, .45, .05, 1) if b in steer else
               (.1, .5, .9, 1) if b.startswith(suspension) or b.endswith('_axle') or o.name.startswith(('Front spring', 'Rear spring')) else
               (.55, .25, .75, 1) if b.startswith('halfshaft') else
               (.35, .35, .35, 1) if b.startswith(('front', 'rear')) else (.2, .45, .3, 1))
    o.hide_render = o.name.startswith(body_parts)
scene.render.engine = 'BLENDER_WORKBENCH'
shading = scene.display.shading
shading.light, shading.color_type, shading.show_object_outline = 'STUDIO', 'OBJECT', True
scene.world.color = (.9, .9, .9)
if 'Ground' in bpy.data.objects:
    bpy.data.objects['Ground'].hide_render = True
scene.render.resolution_x, scene.render.resolution_y = 900, 600
scene.render.image_settings.file_format = 'PNG'
camera = bpy.data.objects.new('Animation camera', bpy.data.cameras.new('Animation camera'))
scene.collection.objects.link(camera)
scene.camera = camera
wb, fr = geometry['parameters']['wheelbase'], geometry['parameters']['front_radius']
views = {'quarter': ((wb+1.7, 1.55, .32), 'PERSP', None, (wb-.15, .05, .40)),
         'front': ((wb+6.0, 0, fr+.05), 'ORTHO', 2.0, (wb, 0, fr+.05))}
sus = geometry.get('suspension') or {}
limit = math.radians(geometry['steering']['column_limit_deg'])
p = report['parameters']
travel = p.get('front_travel_m', .06)
if sus.get('independent_front') and sus.get('wishbones'):
    bump = lambda a: dict(front_left=travel*a, front_right=-travel*a)
    caption2 = 'ONE WHEEL UP, THE OTHER DOWN: each wheel rises on its own wishbones; each track rod follows its own wheel'
elif sus.get('independent_front'):
    bump = lambda a: dict(front_left=travel*a, front_right=-travel*a)
    caption2 = 'ONE WHEEL UP, THE OTHER DOWN: each pillar moves alone; the tie rod tilts, the drag link stays put'
elif 'front_swing_joint' in sus:
    lf = sus.get('front_radius_rod_m', 1.0)
    bump = lambda a: dict(front_swing=-travel/lf*a, front_roll=math.radians(p.get('roll_travel_deg', 4))*a*.5)
    caption2 = 'AXLE BUMP AND ROLL: the whole beam axle moves, and the drag link steers the wheels a little'
elif 'front_heave_joint' in sus:
    bump = lambda a: dict(front_heave=travel*a, front_roll=math.radians(p.get('roll_travel_deg', 4))*a*.5)
    caption2 = 'AXLE BUMP AND ROLL: the whole beam axle moves, and the drag link steers the wheels a little'
else:
    bump = lambda a: {}
    caption2 = ''
FPS, SEG = 30, (4.0, 2.5, 2.5)          # sweep, suspension, both


def schedule(t):
    if t < SEG[0]:
        return limit*math.sin(2*math.pi*t/SEG[0]), bump(0)
    if t < SEG[0]+SEG[1]:
        return 0.0, bump(math.sin(math.pi*(t-SEG[0])/SEG[1]))
    u = t-SEG[0]-SEG[1]
    return limit*math.sin(2*math.pi*u/SEG[2]), bump(math.sin(math.pi*u/SEG[2]))


def pose(phi, susp):
    if sus:
        T = body_transforms(geometry, suspended_pose(geometry, phi, susp))
        for o in objects:
            o.matrix_world = Matrix([list(row) for row in T[o['body']]]) @ rest[o.name]
    else:
        from linkage import world_point
        angles = kinematics(geometry, phi) if phi else {}
        for o in objects:
            b = o['body']
            if b not in steer or not angles:
                o.matrix_world = rest[o.name]
                continue
            chain, n = 0.0, b
            while n:
                chain += angles.get(n, 0.0)
                n = geometry['bodies'][n]['parent']
            origin = Vector(geometry['bodies'][b]['origin'])
            moved = Vector(world_point(geometry, b, geometry['bodies'][b]['origin'], angles))
            o.matrix_world = Matrix.Translation(moved) @ Matrix.Rotation(chain, 4, 'Z') @ Matrix.Translation(-origin) @ rest[o.name]
    bpy.context.view_layer.update()


tmp = Path(tempfile.mkdtemp(prefix='steer_anim_'))
frames = int(sum(SEG)*FPS)
for f in range(frames):
    phi, susp = schedule(f/FPS)
    pose(phi, susp)
    for name, (loc, kind, scale, target) in views.items():
        camera.location = loc
        camera.rotation_euler = (Vector(target)-camera.location).to_track_quat('-Z', 'Y').to_euler()
        camera.data.type = kind
        if scale:
            camera.data.ortho_scale = scale
        else:
            camera.data.lens = 32
        scene.render.filepath = str(tmp/f'{name}_{f:04d}.png')
        bpy.ops.render.render(write_still=True)
pose(0.0, bump(0))
esc = lambda s: s.replace(':', '\\:').replace(',', '\\,').replace("'", '')
caps = [(0, SEG[0], 'STEERING LOCK TO LOCK at ride height'), (SEG[0], SEG[0]+SEG[1], caption2),
        (SEG[0]+SEG[1], sum(SEG), 'STEERING WHILE THE WHEELS MOVE')]
vf = ['[0:v][1:v]hstack=inputs=2[v]']
draw = ','.join(f"drawtext=text='{esc(c)}':x=16:y=14:fontcolor=white:fontsize=22:box=1:boxcolor=black@.45:boxborderw=8:enable='between(t,{a},{b})'"
                for a, b, c in caps if c)
draw += f",drawtext=text='{esc(report['collection'].split('| ')[-1]+'. Orange: steering. Blue: suspension. Purple: drive shafts. Body hidden. Posed by the linkage solver of the build checks.')}':x=16:y=h-34:fontcolor=white:fontsize=17:box=1:boxcolor=black@.45:boxborderw=6"
subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-framerate', str(FPS), '-i', str(tmp/'quarter_%04d.png'),
                '-framerate', str(FPS), '-i', str(tmp/'front_%04d.png'), '-filter_complex', f'{vf[0]};[v]{draw}[o]',
                '-map', '[o]', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', str(out)], check=True)
for f in tmp.iterdir():
    f.unlink()
tmp.rmdir()
print('ANIM_OK', out, frames, 'frames')
