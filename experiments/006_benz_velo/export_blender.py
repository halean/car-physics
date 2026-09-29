"""Export a saved body-tagged car: per-body colliders and evaluated meshes.

blender --background <blend> --python export_blender.py -- <output dir>

Each object's 'body' property selects its physics body; colliders and meshes
are written in that body's frame (rest pose, all bodies axis-aligned).
Rods -> cylinders, swept curves and rings -> capsules, procedural panels -> convex hulls of
their patches, other meshes -> boxes.
"""
import hashlib
import json
from pathlib import Path
import sys
import bpy
from mathutils import Vector

out = Path(sys.argv[sys.argv.index('--')+1]).resolve()
out.mkdir(parents=True, exist_ok=True)
source = Path(bpy.data.filepath)
report = json.loads(source.with_name('report.json').read_text())
bodies = report['bodies']
objects = [o for o in bpy.data.collections[report['collection']].objects if o.type in {'MESH', 'CURVE'}]
origin = {name: Vector(b['origin']) for name, b in bodies.items()}
colliders = {name: [] for name in bodies}
meshes = {name: [] for name in bodies}
for obj in objects:
    body = obj['body']
    if not obj.get('envelope'):       # swept-solid helpers are colliders, not visuals
        meshes[body].append(obj)
    if not obj.get('collide', True):
        continue
    common = dict(object=obj.name, role=obj.get('role', 'general'), mates=list(obj.get('mates', [])),
                  sweep=obj.get('sweep'))
    offset = origin[body]
    if 'patches' in obj:              # procedural panels: one convex hull per patch (bodywork.py)
        M = obj.matrix_world
        for i, flat in enumerate(obj['patches']):
            pts = [list(M @ Vector(flat[k:k+3])-offset) for k in range(0, len(flat), 3)]
            colliders[body].append(dict(name=f'{obj.name}:{i}', type='mesh', vertices=pts, **common))
        continue
    if 'centerline' in obj:
        points = [list(obj.matrix_world @ Vector(p)-offset) for p in obj['centerline']]
        kind = 'capsule' if obj.type == 'CURVE' or obj.get('capsule') else 'cylinder'
        radius = obj.get('envelope_radius', obj['section_radius'])
        for i, (a, b) in enumerate(zip(points, points[1:])):
            colliders[body].append(dict(name=f'{obj.name}:{i}', type=kind, fromto=a+b, size=[radius], **common))
    else:
        colliders[body].append(dict(name=obj.name, type='box', pos=list(obj.matrix_world.translation-offset),
                                    quat=list(obj.matrix_world.to_quaternion()),
                                    size=[v/2 for v in obj.dimensions], **common))
deps = bpy.context.evaluated_depsgraph_get()
for name, items in meshes.items():
    lines, index = [], 1
    for obj in items:
        evaluated = obj.evaluated_get(deps)
        mesh = evaluated.to_mesh()
        mesh.calc_loop_triangles()
        for v in mesh.vertices:
            lines.append('v '+' '.join(f'{c:.7f}' for c in evaluated.matrix_world @ v.co-origin[name]))
        for tri in mesh.loop_triangles:
            lines.append('f '+' '.join(str(index+i) for i in tri.vertices))
        index += len(mesh.vertices)
        evaluated.to_mesh_clear()
    (out/f'{name}.obj').write_text('\n'.join(lines)+'\n')
geometry = dict(source=str(source), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                stage=report['stage'], parameters=report['parameters'], bodies=bodies, loops=report['loops'],
                steering=report['steering'], joins=report['joins'], drive=report.get('drive'),
                rest_joins=report.get('rest_joins', []), suspension=report.get('suspension'),
                couplings=report.get('couplings', []),
                excluded_pairs=report.get('excluded_pairs', []),
                colliders=colliders, collider_objects=sorted({c['object'] for v in colliders.values() for c in v}),
                source_objects=len(objects))
(out/'geometry.json').write_text(json.dumps(geometry, indent=2)+'\n')
print('VELO_EXPORT_OK', sum(map(len, colliders.values())), 'colliders on', len(bodies), 'bodies')
