"""Render drivetrain inspection views from the saved scene (never saves it).

blender --background <blend> --python render_views.py -- <output dir>
"""
from pathlib import Path
import sys
import bpy
from mathutils import Vector

out=Path(sys.argv[sys.argv.index('--')+1])/'views'
out.mkdir(parents=True,exist_ok=True)
scene=bpy.context.scene
vehicle=bpy.data.collections['VEHICLE | 1886 study']
# Bodywork/trim hidden in 'structure' views; the engine stays visible.
body=('Bench','Backrest','Seat','Floor plank')
drive={o.name for o in vehicle.objects if 'sweep' in o or 'mates' in o
       or o.name.endswith('.drive_sprocket') or o.name=='Bevel gear housing'}

scene.render.engine='BLENDER_WORKBENCH'
shading=scene.display.shading
shading.light='STUDIO'
shading.color_type='OBJECT'
shading.show_object_outline=True
scene.render.resolution_x=1100
scene.render.resolution_y=800
scene.world.color=(.9,.9,.9)
for obj in vehicle.objects:
    # Drivetrain orange, brakes red, wheels grey, chassis green.
    obj.color=((1,.45,.05,1) if obj.name in drive else
               (.8,.1,.1,1) if 'brake' in obj.name.lower() else
               (.35,.35,.35,1) if obj.name.startswith(('rear_','front.')) else (.2,.45,.3,1))
if 'Ground' in bpy.data.objects:
    bpy.data.objects['Ground'].hide_render=True
camera=bpy.data.objects.new('Inspection camera',bpy.data.cameras.new('Inspection camera'))
scene.collection.objects.link(camera)
scene.camera=camera
views={'side_left':((-.1,6,.5),'ORTHO',2.2,(-.1,0,.5),False),
       'side_right':((-.1,-6,.5),'ORTHO',2.2,(-.1,0,.5),False),
       'top_structure':((-.1,0,6),'ORTHO',1.8,(-.1,0,.4),True),
       'underside':((-1.3,1.2,-.6),'PERSP',None,(-.25,.1,.35),False),
       'rear_quarter':((-1.9,1.7,1.3),'PERSP',None,(-.2,.1,.45),True),
       'chain_close':((-.35,1.25,.55),'PERSP',None,(-.07,.53,.38),True),
       'belt_close':((-.35,-.9,.15),'PERSP',None,(-.3,.2,.3),True),
       'rear':((-4,0,.5),'ORTHO',1.6,(0,0,.5),False)}
for name,(location,kind,scale,target,hide) in views.items():
    for obj in vehicle.objects:
        obj.hide_render=hide and obj.name.startswith(body)
    camera.location=location
    up='Y' if name!='top_structure' else 'Y'
    camera.rotation_euler=(Vector(target)-camera.location).to_track_quat('-Z',up).to_euler()
    camera.data.type=kind
    if scale:
        camera.data.ortho_scale=scale
    else:
        camera.data.lens=35
    scene.render.filepath=str(out/f'{name}.png')
    bpy.ops.render.render(write_still=True)
print('VIEWS_OK',out,len(drive),'drive objects')
