"""Record Mercedes scenarios with per-wheel force arrows (same controls and calibrated springs as the tests).

Panels: launch through four gears | 15-degree hill start in first | powered 10-degree left turn in first
Run after run.py --stage full (the spring preloads come from its report).
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
# A 1200 kg car: ~3000 N per tyre would draw 3 m arrows at the light cars' scale.
arrows.LOAD_M_PER_N, arrows.TIRE_M_PER_N = .0002, .001
LEGEND = 'Arrows: blue = tire load (1 m per 5000 N), orange = road force on tire (1 m per 1000 N)'

physics = HERE/'output/full/physics'
geometry = json.loads((physics/'geometry.json').read_text())
report = json.loads((physics/'report.json').read_text())
assert report['source_sha256'] == geometry['source_sha256'], 'report is from another build: rerun run.py'
for body in ('front_axle', 'rear_axle'):
    for j in geometry['bodies'][body]['joint']:
        key = f'{body}_{j["name"]}'
        j.update(vehicle.CONFIG['suspension'][key], springref=report['springs']['springref'][key])
ratios = {g: n*ENGINE['bevel_ratio']*geometry['drive']['chain_ratio'] for g, n in ENGINE['gearbox_ratios'].items()}
engine = dict(torque_nm=MAX_TORQUE, ratios=ratios)
scenarios = [('LAUNCH: 1ST, 2ND, 3RD, 4TH', 0, Controller(gears=SHIFTS, steer=Steer(hold_heading=True)), 'chase'),
             ('15 DEG HILL START (FIRST GEAR)', -math.radians(15),
              Controller(gears=[(.5, 'first')], brake=Brake(apply_at=0, release_at=1.5)), 'side'),
             ('POWERED LEFT TURN (FIRST, 10 DEG)', 0, Controller(gears=[(.5, 'first')], steer=Steer(10)), 'top')]
models = [vehicle.make_model(geometry, physics, slope, engine=engine, path_name=f'record_{i}.xml')
          for i, (_, slope, _, _) in enumerate(scenarios)]
states = [mujoco.MjData(m) for m in models]
averagers = [arrows.ForceAverager(.05) for _ in models]   # 50 ms mean: see visual.ForceAverager
options = mujoco.MjvOption()
options.geomgroup[3] = 0
width, height, fps, seconds = 560, 420, 30, 14
esc = lambda text: text.replace(':', '\\:').replace(',', '\\,')
labels = [f"drawtext=text='{esc(t)}':x={16+i*width}:y=14:fontcolor=white:fontsize=19"
          for i, (t, _, _, _) in enumerate(scenarios)]
labels += [f"drawtext=text='BRAKE RELEASED':x={16+width}:y=40:fontcolor=orange:fontsize=18:enable='gte(t,1.5)'"]
for name, (start, end) in dict(first=(.5, 2.5), second=(2.7, 5.0), third=(5.2, 9.5), fourth=(9.7, 99)).items():
    labels.append(f"drawtext=text='{name.upper()}':x=16:y=40:fontcolor=yellow:fontsize=18:"
                  f"enable='between(t,{start},{end})'")
labels += [f"drawtext=text='{esc(LEGEND+'; 50 ms average')}':x=16:y={height-52}:fontcolor=white:fontsize=16",
           f"drawtext=text='{esc('Mercedes 35 HP 1901 study, 1200 kg, 35 hp; leaf springs, chain drive, gate-change 4 speeds')}':"
           f"x=16:y={height-28}:fontcolor=white:fontsize=16"]
renderers = [mujoco.Renderer(m, height, width) for m in models]
out = physics/'mercedes_drive.mp4'
process = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                            '-s', f'{width*3}x{height}', '-r', str(fps), '-i', '-', '-vf', ','.join(labels),
                            '-an', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', str(out)],
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
            if view == 'chase':      # follows the car at up to ~20 m/s
                cam.distance, cam.azimuth, cam.elevation = 7.0, 145, -16
                cam.lookat[:] = d.body('chassis').xpos+[.8, 0, .2]
            elif view == 'side':
                cam.distance, cam.azimuth, cam.elevation = 6.0, 70, -12
                cam.lookat[:] = d.body('chassis').xpos+[1.0, 0, .3]
            else:
                cam.distance, cam.azimuth, cam.elevation = 44, 0, -88
                cam.lookat[:] = [.4, 13.4, 0]   # the ~13 m circle
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
print(out)
