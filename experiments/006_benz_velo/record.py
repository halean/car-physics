"""Record Velo scenarios with per-wheel force arrows (same controls as the tests).

Panels: low-gear launch with shift to high | 5-degree hill start in low | powered 20-degree left turn
"""
import os
os.environ.setdefault('MUJOCO_GL', 'egl')
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import numpy as np
import mujoco

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import vehicle
from control import MAX_TORQUE, Brake, Controller, Steer

spec = importlib.util.spec_from_file_location('arrows', HERE.parent/'005_engine_drive/visual.py')
arrows = importlib.util.module_from_spec(spec)
spec.loader.exec_module(arrows)

physics = HERE/'output/full/physics'
geometry = json.loads((physics/'geometry.json').read_text())
ratios = {g: v['overall_ratio'] for g, v in geometry['drive']['gears'].items()}
engine = dict(torque_nm=MAX_TORQUE, ratios=ratios)
scenarios = [('LOW GEAR LAUNCH, SHIFT TO HIGH AT 6 S', 0,
              Controller(gears=[(.5, 'low'), (6.0, None), (6.2, 'high')], steer=Steer(hold_heading=True)), 'side'),
             ('5 DEG HILL START (LOW GEAR)', -math.radians(5),
              Controller(gears=[(.5, 'low')], brake=Brake(apply_at=0, release_at=1.5)), 'side'),
             ('POWERED LEFT TURN (LOW, 20 DEG)', 0, Controller(gears=[(.5, 'low')], steer=Steer(20)), 'top')]
models = [vehicle.make_model(geometry, physics, slope, engine=engine, path_name=f'record_{i}.xml')
          for i, (_, slope, _, _) in enumerate(scenarios)]
states = [mujoco.MjData(m) for m in models]
options = mujoco.MjvOption()
options.geomgroup[3] = 0
width, height, fps, seconds = 560, 420, 30, 12
esc = lambda text: text.replace(':', '\\:').replace(',', '\\,')
labels = [f"drawtext=text='{esc(t)}':x={16+i*width}:y=14:fontcolor=white:fontsize=19"
          for i, (t, _, _, _) in enumerate(scenarios)]
labels += [f"drawtext=text='BRAKE RELEASED':x={16+width}:y=40:fontcolor=orange:fontsize=18:enable='gte(t,1.5)'",
           f"drawtext=text='HIGH GEAR':x=16:y=40:fontcolor=yellow:fontsize=18:enable='gte(t,6.2)'",
           f"drawtext=text='{esc(arrows.LEGEND)}':x=16:y={height-52}:fontcolor=white:fontsize=16",
           f"drawtext=text='{esc('Benz Velo study, 280 kg, 1.1 kW at 450 rpm; low 6.52, high 3.48 overall')}':"
           f"x=16:y={height-28}:fontcolor=white:fontsize=16"]
renderers = [mujoco.Renderer(m, height, width) for m in models]
process = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                            '-s', f'{width*3}x{height}', '-r', str(fps), '-i', '-', '-vf', ','.join(labels),
                            '-an', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', str(physics/'velo_drive.mp4')],
                           stdin=subprocess.PIPE)
try:
    for frame in range(seconds*fps+1):
        images = []
        for (_, _, ctrl, view), m, d, renderer in zip(scenarios, models, states, renderers):
            while d.time < frame/fps:
                ctrl(m, d)
                mujoco.mj_step(m, d)
            mujoco.mj_forward(m, d)
            cam = mujoco.MjvCamera()
            if view == 'side':
                cam.distance, cam.azimuth, cam.elevation = 4.2, 70, -14
                cam.lookat[:] = d.body('chassis').xpos+[.6, 0, .4]
            else:
                cam.distance, cam.azimuth, cam.elevation = 13, 0, -85
                cam.lookat[:] = [1.2, 4, 0]
            renderer.update_scene(d, camera=cam, scene_option=options)
            arrows.add_force_arrows(m, d, renderer.scene)
            images.append(renderer.render())
        process.stdin.write(np.concatenate(images, axis=1).tobytes())
finally:
    process.stdin.close()
    status = process.wait()
    for r in renderers:
        r.close()
if status:
    raise SystemExit(status)
print(physics/'velo_drive.mp4')
