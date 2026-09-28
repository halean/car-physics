"""Record Panhard scenarios with per-wheel force arrows (same controls as the tests).

Panels: launch through three gears | 10-degree hill start in first | powered 20-degree left turn in second
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
sys.path.insert(0, str(HERE.parent/'006_benz_velo'))
import vehicle
vehicle.CONFIG = json.loads((HERE/'vehicle.json').read_text())
sys.path.insert(0, str(HERE))
from control import ENGINE, MAX_TORQUE, Brake, Controller, Steer
from dynamics import SHIFTS

spec = importlib.util.spec_from_file_location('arrows', HERE.parent/'005_engine_drive/visual.py')
arrows = importlib.util.module_from_spec(spec)
spec.loader.exec_module(arrows)

physics = HERE/'output/full/physics'
geometry = json.loads((physics/'geometry.json').read_text())
ratios = {g: n*geometry['drive']['chain_ratio'] for g, n in ENGINE['gearbox_ratios'].items()}
engine = dict(torque_nm=MAX_TORQUE, ratios=ratios)
scenarios = [('LAUNCH: 1ST, 2ND AT 4 S, 3RD AT 8 S', 0, Controller(gears=SHIFTS, steer=Steer(hold_heading=True)),
              'side'),
             ('10 DEG HILL START (FIRST GEAR)', -math.radians(10),
              Controller(gears=[(.5, 'first')], brake=Brake(apply_at=0, release_at=1.5)), 'side'),
             ('POWERED LEFT TURN (SECOND, 20 DEG)', 0, Controller(gears=[(.5, 'second')], steer=Steer(20)), 'top')]
models = [vehicle.make_model(geometry, physics, slope, engine=engine, path_name=f'record_{i}.xml')
          for i, (_, slope, _, _) in enumerate(scenarios)]
states = [mujoco.MjData(m) for m in models]
averagers = [arrows.ForceAverager(.05) for _ in models]   # 50 ms mean: see visual.ForceAverager
options = mujoco.MjvOption()
options.geomgroup[3] = 0
width, height, fps, seconds = 560, 420, 30, 12
esc = lambda text: text.replace(':', '\\:').replace(',', '\\,')
labels = [f"drawtext=text='{esc(t)}':x={16+i*width}:y=14:fontcolor=white:fontsize=19"
          for i, (t, _, _, _) in enumerate(scenarios)]
labels += [f"drawtext=text='BRAKE RELEASED':x={16+width}:y=40:fontcolor=orange:fontsize=18:enable='gte(t,1.5)'",
           "drawtext=text='2ND':x=16:y=40:fontcolor=yellow:fontsize=18:enable='between(t,4.2,8.2)'",
           "drawtext=text='3RD':x=16:y=40:fontcolor=yellow:fontsize=18:enable='gte(t,8.2)'",
           f"drawtext=text='{esc(arrows.LEGEND+'; 50 ms average')}':x=16:y={height-52}:fontcolor=white:fontsize=16",
           f"drawtext=text='{esc('Panhard et Levassor 1891 study, 600 kg assumed, 4 bhp V-twin; overall 19.0 / 11.5 / 7.4')}':"
           f"x=16:y={height-28}:fontcolor=white:fontsize=16"]
renderers = [mujoco.Renderer(m, height, width) for m in models]
process = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                            '-s', f'{width*3}x{height}', '-r', str(fps), '-i', '-', '-vf', ','.join(labels),
                            '-an', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', str(physics/'panhard_drive.mp4')],
                           stdin=subprocess.PIPE)
try:
    for frame in range(seconds*fps+1):
        images = []
        for (_, _, ctrl, view), m, d, renderer, avg in zip(scenarios, models, states, renderers, averagers):
            while d.time < frame/fps:
                ctrl(m, d)
                mujoco.mj_step(m, d)
                avg.sample(m, d)
            mujoco.mj_forward(m, d)
            cam = mujoco.MjvCamera()
            if view == 'side':
                cam.distance, cam.azimuth, cam.elevation = 4.8, 70, -14
                cam.lookat[:] = d.body('chassis').xpos+[.6, 0, .4]
            else:
                cam.distance, cam.azimuth, cam.elevation = 15, 0, -85
                cam.lookat[:] = [1.4, 4.6, 0]
            renderer.update_scene(d, camera=cam, scene_option=options)
            avg.draw(renderer.scene)
            images.append(renderer.render())
        process.stdin.write(np.concatenate(images, axis=1).tobytes())
finally:
    process.stdin.close()
    status = process.wait()
    for r in renderers:
        r.close()
if status:
    raise SystemExit(status)
print(physics/'panhard_drive.mp4')
