"""Export the saved Model T as a GLB for the web page: every vehicle object as a mesh in its
MuJoCo body's frame (rest pose), named '<body>|<n>', Z up, with its Blender material.

blender --background <model_t.blend> --python export_glb.py -- <out.glb>
"""
import json
from pathlib import Path
import sys
import bpy
from mathutils import Matrix, Vector

out = Path(sys.argv[sys.argv.index('--')+1]).resolve()
source = Path(bpy.data.filepath)
report = json.loads(source.with_name('report.json').read_text())
origin = {name: Vector(b['origin']) for name, b in report['bodies'].items()}
objects = [o for o in bpy.data.collections[report['collection']].objects
           if o.type in {'MESH', 'CURVE'} and not o.get('envelope')]
deps = bpy.context.evaluated_depsgraph_get()
export = bpy.data.collections.new('WEB EXPORT')
bpy.context.scene.collection.children.link(export)
for i, obj in enumerate(objects):
    evaluated = obj.evaluated_get(deps)
    mesh = bpy.data.meshes.new_from_object(evaluated, depsgraph=deps)
    item = bpy.data.objects.new(f"{obj['body']}|{i}", mesh)
    item.matrix_world = Matrix.Translation(-origin[obj['body']]) @ obj.matrix_world
    if not mesh.materials and obj.active_material:
        mesh.materials.append(obj.active_material)
    export.objects.link(item)
bpy.ops.object.select_all(action='DESELECT')
for item in export.objects:
    item.select_set(True)
bpy.ops.export_scene.gltf(filepath=str(out), export_format='GLB', use_selection=True, export_yup=False,
                          export_apply=True, export_extras=False, export_cameras=False, export_lights=False)
print('GLB_OK', out, len(export.objects), 'objects', out.stat().st_size, 'bytes')
