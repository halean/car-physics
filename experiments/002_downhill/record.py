"""Record the generated MuJoCo model; requires ffmpeg and an EGL-capable driver."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import subprocess
import mujoco

out=Path(__file__).resolve().parent/'output'
model=mujoco.MjModel.from_xml_path(str(out/'downhill.xml'))
data=mujoco.MjData(model)
options=mujoco.MjvOption()
options.geomgroup[3]=0
camera=mujoco.MjvCamera()
camera.distance=5
camera.azimuth=130
camera.elevation=-20
width,height,fps=640,480,30
with mujoco.Renderer(model,height,width) as renderer:
    process=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24',
        '-s',f'{width}x{height}','-r',str(fps),'-i','-','-an','-c:v','libx264',
        '-pix_fmt','yuv420p',str(out/'downhill.mp4')],stdin=subprocess.PIPE)
    try:
        for frame in range(121):
            while data.time < frame/fps:
                mujoco.mj_step(model,data)
            mujoco.mj_forward(model,data)
            camera.lookat[:]=data.body('chassis').xpos+[.4,0,.6]
            renderer.update_scene(data,camera=camera,scene_option=options)
            process.stdin.write(renderer.render().tobytes())
    finally:
        process.stdin.close()
        status=process.wait()
    if status:
        raise SystemExit(status)
print(out/'downhill.mp4')
