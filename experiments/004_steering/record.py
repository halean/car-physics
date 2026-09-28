"""Record the same six-second left/right steered, braked descents used by the test."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import subprocess
import sys
import numpy as np
import mujoco

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from control import SteeringControl

out=HERE/'output'
models=[mujoco.MjModel.from_xml_path(str(out/'downhill.xml')) for _ in range(2)]
states=[mujoco.MjData(model) for model in models]
controls=[SteeringControl(1),SteeringControl(-1)]
options=mujoco.MjvOption()
options.geomgroup[3]=0
camera=mujoco.MjvCamera()
# Near-overhead, travel direction up the screen, so a left turn moves left.
camera.distance=11.5
camera.azimuth=0
camera.elevation=-80
width,height,fps=640,480,30
labels=("drawtext=text='LEFT (+15 deg tiller)':x=20:y=20:fontcolor=white:fontsize=24,"
        "drawtext=text='RIGHT (-15 deg tiller)':x=660:y=20:fontcolor=white:fontsize=24,"
        "drawtext=text='REAR BRAKES APPLIED':x=20:y=55:fontcolor=yellow:fontsize=22:enable='gte(t,3)',"
        "drawtext=text='REAR BRAKES APPLIED':x=660:y=55:fontcolor=yellow:fontsize=22:enable='gte(t,3)',"
        "drawtext=text='Unpowered, 5 degree slope. Tiller ramps 0.75-1.5 s; fixed camera':x=20:y=450:fontcolor=white:fontsize=18")
renderers=[mujoco.Renderer(model,height,width) for model in models]
process=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24',
    '-s',f'{width*2}x{height}','-r',str(fps),'-i','-','-vf',labels,'-an','-c:v','libx264',
    '-pix_fmt','yuv420p',str(out/'steering_comparison.mp4')],stdin=subprocess.PIPE)
try:
    for frame in range(181):
        images=[]
        for model,data,control,renderer,side in zip(models,states,controls,renderers,(1,-1)):
            while data.time<frame/fps:
                control(model,data)
                mujoco.mj_step(model,data)
            mujoco.mj_forward(model,data)
            # Fixed camera over the run so heading change is visible.
            camera.lookat[:]=[3.8,.9*side,.2]
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
print(out/'steering_comparison.mp4')
