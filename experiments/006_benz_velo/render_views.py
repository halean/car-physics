"""Render Velo inspection views at both steering locks (never saves the scene).

blender --background <blend> --python render_views.py -- <geometry.json> <output dir> [extra_views.json]
Steering bodies are posed with linkage.py, the same kinematics the tests use.
"""
import json
import math
from pathlib import Path
import sys
import bpy
from mathutils import Matrix, Vector

sys.path.insert(0, str(Path(__file__).resolve().parent))
from linkage import body_transforms, kinematics, suspended_pose, world_point

argv = sys.argv[sys.argv.index('--')+1:]
geometry = json.loads(Path(argv[0]).read_text())
out = Path(argv[1])/'views'
out.mkdir(parents=True, exist_ok=True)
report = json.loads(Path(bpy.data.filepath).with_name('report.json').read_text())
bodies = geometry['bodies']
scene = bpy.context.scene
objects = [o for o in bpy.data.collections[report['collection']].objects if o.type in {'MESH', 'CURVE'}]
rest = {o.name: o.matrix_world.copy() for o in objects}
steer = {'knuckle_left', 'knuckle_right', 'tie_rod', 'column', 'drag_link', 'front_left', 'front_right'}
body_parts = ('Seat', 'Bench', 'Body', 'Bonnet', 'Engine box', 'Dash', 'Footboard', 'Fuel tank', 'Mudguard', 'Floor')
colors = {'chassis': (.2, .45, .3, 1)}
for o in objects:
    b = o['body']
    o.color = ((.8, .1, .1, 1) if 'brake' in o.name.lower() else
               (1, .45, .05, 1) if b in steer and not b.startswith('front') else
               (.35, .35, .35, 1) if b.startswith(('front', 'rear')) else
               (.15, .35, .9, 1) if 'sweep' in o or o.name.startswith(('Drive', 'Countershaft', 'Engine pulley'))
               else colors['chassis'])
scene.render.engine = 'BLENDER_WORKBENCH'
shading = scene.display.shading
shading.light, shading.color_type, shading.show_object_outline = 'STUDIO', 'OBJECT', True
scene.render.resolution_x, scene.render.resolution_y = 1100, 800
scene.world.color = (.9, .9, .9)
if 'Ground' in bpy.data.objects:
    bpy.data.objects['Ground'].hide_render = True
camera = bpy.data.objects.new('Inspection camera', bpy.data.cameras.new('Inspection camera'))
scene.collection.objects.link(camera)
scene.camera = camera
wb = geometry['parameters']['wheelbase']


def pose(phi):
    if geometry.get('suspension'):   # sprung car: general forward kinematics
        T = body_transforms(geometry, suspended_pose(geometry, phi, {}))
        for o in objects:
            o.matrix_world = Matrix([list(row) for row in T[o['body']]]) @ rest[o.name]
        bpy.context.view_layer.update()
        return
    angles = kinematics(geometry, phi) if phi else {}
    for o in objects:
        b = o['body']
        if b not in steer or not angles:
            o.matrix_world = rest[o.name]
            continue
        chain, n = 0.0, b
        while n:
            if n in angles:
                chain += angles[n]
            n = bodies[n]['parent']
        origin = Vector(bodies[b]['origin'])
        moved = Vector(world_point(geometry, b, bodies[b]['origin'], angles))
        o.matrix_world = (Matrix.Translation(moved) @ Matrix.Rotation(chain, 4, 'Z') @
                          Matrix.Translation(-origin) @ rest[o.name])
    bpy.context.view_layer.update()


views = {'top': ((wb/2, 0, 6), 'ORTHO', 2.6, (wb/2, 0, .3), None),
         'side': ((wb/2, -6, .5), 'ORTHO', 2.6, (wb/2, 0, .5), None),
         'front': ((6, 0, .5), 'ORTHO', 1.8, (wb, 0, .5), None),
         'perspective': ((wb+2.4, -2.2, 1.6), 'PERSP', None, (wb*.5, 0, .45), None),
         'front_linkage': ((wb+.9, -.7, -.35), 'PERSP', None, (wb-.1, 0, .2), None),
         'linkage_top': ((wb-.1, 0, 3), 'ORTHO', 1.3, (wb-.1, 0, .2), 'body'),
         'rear_brake': ((.55, .05, .95), 'PERSP', None, (0, .42, .45), 'body'),
         'drive_under': ((-.2, -.9, -.25), 'PERSP', None, (-.35, .1, .33), 'body')}
if len(argv) > 2:   # optional JSON of extra close-ups: {name: [camera, 'PERSP'|'ORTHO', scale, target, hide]}
    views.update({k: tuple(v) for k, v in json.loads(Path(argv[2]).read_text()).items()})
rest_only = {'side', 'rear_brake', 'drive_under'} | (set(views) - {'top', 'front', 'perspective', 'front_linkage',
                                                                    'linkage_top'})
limit = geometry['steering'].get('column_limit_deg') or bodies['column']['joint']['range_deg'][1]
for angle in (-limit, 0, limit):
    pose(math.radians(angle))
    for name, (loc, kind, scale, target, hide) in views.items():
        if angle and name in rest_only:
            continue
        for o in objects:
            o.hide_render = hide == 'body' and o.name.startswith(body_parts+('Handwheel',))
        camera.location = loc
        camera.rotation_euler = (Vector(target)-camera.location).to_track_quat('-Z', 'Y').to_euler()
        camera.data.type = kind
        if scale:
            camera.data.ortho_scale = scale
        else:
            camera.data.lens = 35
        scene.render.filepath = str(out/f'{name}_{angle:+.0f}deg.png')
        bpy.ops.render.render(write_still=True)
pose(0)
print('VIEWS_OK', out)
