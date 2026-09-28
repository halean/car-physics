"""Record the flat launch, 3-degree hill start and powered left turn used by the test."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import numpy as np
import mujoco

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from control import MAX_TORQUE, EngineControl, Tiller
from visual import LEGEND, add_force_arrows


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


physics=load('downhill_physics',HERE.parent/'002_downhill/run.py')
BrakeControl=load('rear_brake_control',HERE.parent/'003_brakes/control.py').BrakeControl
out=HERE/'output'
geometry=json.loads((out/'geometry.json').read_text())
scenarios=[('FLAT LAUNCH',0,EngineControl(engage_at=.5),'side'),
           ('3 DEG HILL START',-math.radians(3),
            EngineControl(engage_at=.5,brake=BrakeControl(apply_at=0,release_at=1.5)),'side'),
           ('POWERED LEFT TURN',0,EngineControl(engage_at=.5,steering=Tiller(1)),'top')]
models=[physics.make_model(geometry,out,angle,engine_torque_nm=MAX_TORQUE) for _,angle,_,_ in scenarios]
physics.make_model(geometry,out,0,engine_torque_nm=MAX_TORQUE)  # restore deliverable XML
states=[mujoco.MjData(model) for model in models]
options=mujoco.MjvOption()
options.geomgroup[3]=0
width,height,fps,seconds=560,420,30,12
labels=[f"drawtext=text='{title}':x={20+i*width}:y=16:fontcolor=white:fontsize=22"
        for i,(title,_,_,_) in enumerate(scenarios)]
labels+=[f"drawtext=text='BELT ENGAGED':x={20+i*width}:y=46:fontcolor=yellow:fontsize=20:enable='gte(t,0.5)'"
         for i in range(3)]
labels+=[f"drawtext=text='BRAKE RELEASED':x={20+width}:y=74:fontcolor=orange:fontsize=20:enable='gte(t,1.5)'",
         "drawtext=text='"+LEGEND.replace(':','\\:').replace(',','\\,')+"':"
         f"x=20:y={height-56}:fontcolor=white:fontsize=17",
         "drawtext=text='0.5 kW governed at 400 rpm; tracking camera (left, centre), fixed overhead (right)':"
         f"x=20:y={height-30}:fontcolor=white:fontsize=17"]
renderers=[mujoco.Renderer(model,height,width) for model in models]
process=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24',
    '-s',f'{width*3}x{height}','-r',str(fps),'-i','-','-vf',','.join(labels),'-an','-c:v','libx264',
    '-pix_fmt','yuv420p',str(out/'engine_drive.mp4')],stdin=subprocess.PIPE)
try:
    for frame in range(seconds*fps+1):
        images=[]
        for (_,angle,control,view),model,data,renderer in zip(scenarios,models,states,renderers):
            while data.time<frame/fps:
                control(model,data)
                mujoco.mj_step(model,data)
            mujoco.mj_forward(model,data)
            camera=mujoco.MjvCamera()
            if view=='side':
                camera.distance,camera.azimuth,camera.elevation=4.5,90,-12
                camera.lookat[:]=data.body('chassis').xpos+[.5,0,.3]
            else:
                camera.distance,camera.azimuth,camera.elevation=17,0,-85
                camera.lookat[:]=[1,6.2,0]
            renderer.update_scene(data,camera=camera,scene_option=options)
            add_force_arrows(model,data,renderer.scene)
            images.append(renderer.render())
        process.stdin.write(np.concatenate(images,axis=1).tobytes())
finally:
    process.stdin.close()
    status=process.wait()
    for renderer in renderers:
        renderer.close()
if status:
    raise SystemExit(status)
print(out/'engine_drive.mp4')
