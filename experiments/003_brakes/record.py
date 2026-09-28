"""Record the same six-second coast/brake comparison used by the test."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import subprocess
import numpy as np
import mujoco
from control import BrakeControl

out=Path(__file__).resolve().parent/'output'
models=[mujoco.MjModel.from_xml_path(str(out/'downhill.xml')) for _ in range(2)]
states=[mujoco.MjData(model) for model in models]
control=BrakeControl()
options=mujoco.MjvOption()
options.geomgroup[3]=0
camera=mujoco.MjvCamera()
camera.distance=4.5
camera.azimuth=125
camera.elevation=-18
width,height,fps=640,480,30
labels=("drawtext=text='COAST':x=20:y=20:fontcolor=white:fontsize=24,"
        "drawtext=text='REAR BRAKES':x=660:y=20:fontcolor=white:fontsize=24,"
        "drawtext=text='APPLIED':x=660:y=55:fontcolor=yellow:fontsize=22:enable='gte(t,2)',"
        "drawtext=text='Brake applied at 2 seconds - same 5 degree slope':x=20:y=450:fontcolor=white:fontsize=18")
renderers=[mujoco.Renderer(model,height,width) for model in models]
process=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24',
    '-s',f'{width*2}x{height}','-r',str(fps),'-i','-','-vf',labels,'-an','-c:v','libx264',
    '-pix_fmt','yuv420p',str(out/'brake_comparison.mp4')],stdin=subprocess.PIPE)
try:
    for frame in range(181):
        images=[]
        for i,(model,data,renderer) in enumerate(zip(models,states,renderers)):
            while data.time<frame/fps:
                if i==1:
                    control(model,data)
                mujoco.mj_step(model,data)
            mujoco.mj_forward(model,data)
            camera.lookat[:]=data.body('chassis').xpos+[.4,0,.6]
            renderer.update_scene(data,camera=camera,scene_option=options)
            images.append(renderer.render())
        process.stdin.write(np.concatenate(images,axis=1).tobytes())
finally:
    process.stdin.close()
    status=process.wait()
    for renderer in renderers:
        renderer.close()
if status:
    raise SystemExit(status)
print(out/'brake_comparison.mp4')
