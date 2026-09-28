"""Export saved car assemblies, colliders and evaluated visual meshes."""
import hashlib
import json
import math
from pathlib import Path
import sys
import bpy
from mathutils import Vector

out=Path(sys.argv[sys.argv.index('--')+1]).resolve()
out.mkdir(parents=True,exist_ok=True)
source=Path(bpy.data.filepath)
report=json.loads(source.with_name('report.json').read_text())
p=report['parameters']
wheels=[('rear_left',(0,p['rear_track']/2,p['rear_radius']),p['rear_radius']),
        ('rear_right',(0,-p['rear_track']/2,p['rear_radius']),p['rear_radius']),
        ('front',(p['wheelbase'],0,p['front_radius']),p['front_radius'])]
rig=bpy.data.objects.get('Steering pivot')
if rig and abs(rig['angle_deg'])>1e-8:
    raise ValueError('Save the steering rig at angle_deg=0 before exporting the rest geometry')
pivot=list(rig.matrix_world.translation) if rig else None
objects=[obj for obj in bpy.data.collections['VEHICLE | 1886 study'].objects if obj.type in {'MESH','CURVE'}]
groups={'chassis':[]}|{name:[] for name,_,_ in wheels}
if pivot:
    groups['steering']=[]
offsets={'chassis':Vector((0,0,0))}|{name:Vector(pos) for name,pos,_ in wheels}
if pivot:
    offsets['steering']=Vector(pivot)
colliders=[]
steering_colliders=[]
wheel_colliders={name:[] for name,_,_ in wheels}
for obj in objects:
    group=next((name for name,_,_ in wheels if obj.name.startswith(name+'.')),obj.get('assembly','chassis'))
    groups[group].append(obj)
    offset=offsets[group]
    if group in wheel_colliders:
        if obj.name.endswith(('.brake_drum','.brake_sleeve')):
            points=[list(obj.matrix_world @ Vector(v)-offset) for v in obj['centerline']]
            wheel_colliders[group].append(dict(name=obj.name,type='cylinder',
                fromto=points[0]+points[1],size=[obj['section_radius']]))
        continue
    target=steering_colliders if group=='steering' else colliders
    role=('axle' if obj.name in ('Rear axle','Front axle') else
          'bearing' if obj.name=='Steering bearing' else
          'shaft' if obj.name=='Steering column' else group)
    if 'centerline' in obj:
        points=[list(obj.matrix_world @ Vector(v)-offset) for v in obj['centerline']]
        for i,(a,b) in enumerate(zip(points,points[1:])):
            target.append(dict(name=f'{obj.name}:{i}',type='cylinder' if obj.type=='MESH' else 'capsule',
                fromto=a+b,size=[obj['section_radius']],axle=role=='axle',role=role))
    elif obj.name=='Horizontal flywheel':
        for i in range(48):
            points=[list(obj.matrix_world @ Vector((.255*math.cos(a),.255*math.sin(a),0))-offset)
                    for a in (math.tau*i/48,math.tau*(i+1)/48)]
            target.append(dict(name=f'Flywheel rim:{i}',type='capsule',fromto=sum(points,[]),
                               size=[.025],axle=False,role=role))
    else:
        target.append(dict(name=obj.name,type='box',pos=list(obj.matrix_world.translation-offset),
            quat=list(obj.matrix_world.to_quaternion()),size=[v/2 for v in obj.dimensions],axle=False,role=role))
deps=bpy.context.evaluated_depsgraph_get()
for name,items in groups.items():
    lines=[]
    index=1
    for obj in items:
        evaluated=obj.evaluated_get(deps)
        mesh=evaluated.to_mesh()
        mesh.calc_loop_triangles()
        for vertex in mesh.vertices:
            v=evaluated.matrix_world @ vertex.co-offsets[name]
            lines.append('v '+' '.join(f'{x:.8f}' for x in v))
        for tri in mesh.loop_triangles:
            lines.append('f '+' '.join(str(index+i) for i in tri.vertices))
        index+=len(mesh.vertices)
        evaluated.to_mesh_clear()
    (out/f'{name}.obj').write_text('\n'.join(lines)+'\n')
(out/'geometry.json').write_text(json.dumps(dict(source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    parameters=p,wheels=wheels,colliders=colliders,wheel_colliders=wheel_colliders,
    steering_pivot=pivot,steering_limit_deg=p.get('steering_limit_deg',0),
    steering_colliders=steering_colliders,source_objects=len(objects)),indent=2)+'\n')
print('PHYSICS_EXPORT_OK',len(colliders),'chassis primitives;',len(steering_colliders),'steering primitives')
