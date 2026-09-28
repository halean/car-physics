"""Render steering inspection views at both locks from the saved scene (never saves it).

blender --background <blend> --python render_views.py -- <output dir>
"""
import math
from pathlib import Path
import sys
import bpy
from mathutils import Vector

out=Path(sys.argv[sys.argv.index('--')+1])/'views'
out.mkdir(parents=True,exist_ok=True)
scene=bpy.context.scene
rig=bpy.data.objects['Steering pivot']
limit=rig['limit_deg']
vehicle=bpy.data.collections['VEHICLE | 1886 study']
# Bodywork/trim hidden in the structure view; frame, wheels, steering and brakes remain.
body=('Bench','Backrest','Seat','Floor plank','Brass reservoir','Cylinder','Engine bed',
      'Exhaust','Flywheel','Horizontal')
steering={o.name for o in rig.children_recursive}

scene.render.engine='BLENDER_WORKBENCH'
shading=scene.display.shading
shading.light='STUDIO'
shading.color_type='OBJECT'
shading.show_object_outline=True
scene.render.resolution_x=960
scene.render.resolution_y=720
scene.render.film_transparent=False
scene.world.color=(.9,.9,.9)
for obj in vehicle.objects:
    # Steering/front assembly orange, rolling rear wheels grey, chassis green.
    obj.color=((1,.45,.05,1) if obj.name in steering else
               (.35,.35,.35,1) if obj.name.startswith('rear_') else (.2,.45,.3,1))
if 'Ground' in bpy.data.objects:
    bpy.data.objects['Ground'].hide_render=True

camera=bpy.data.objects.new('Inspection camera',bpy.data.cameras.new('Inspection camera'))
scene.collection.objects.link(camera)
scene.camera=camera
center=Vector((.6,0,.55))
views={'top':((.6,0,6),'ORTHO',3.4),'front':((6,0,.55),'ORTHO',2.4),
       'side':((.6,-6,.55),'ORTHO',3.4),'perspective':((3.6,-2.8,2.2),'PERSP',None),
       'front_close':((3.2,-1.3,1.4),'PERSP',None)}
for angle in (-limit,0,limit):
    rig['angle_deg']=float(angle)
    rig.update_tag()
    bpy.context.view_layer.update()
    measured=math.degrees(rig.matrix_world.to_euler().z)
    assert abs(measured-angle)<1e-3,(measured,angle)
    for hide in (False,True):
        for obj in vehicle.objects:
            obj.hide_render=hide and obj.name.startswith(body)
        for name,(location,kind,scale) in views.items():
            if hide and name not in ('top','front_close'):
                continue
            camera.location=location
            target=Vector((1.45,0,.45)) if name=='front_close' else center
            camera.rotation_euler=(target-camera.location).to_track_quat('-Z','Y').to_euler()
            camera.data.type=kind
            if scale:
                camera.data.ortho_scale=scale
            else:
                camera.data.lens=35 if name=='perspective' else 50
            tag=f'{"structure_" if hide else ""}{name}_{angle:+.0f}deg'
            scene.render.filepath=str(out/f'{tag}.png')
            bpy.ops.render.render(write_still=True)
print('VIEWS_OK',out)
