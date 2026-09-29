"""Model T with and without aftermarket friction shock absorbers, over the bump course.

.venv/bin/python experiments/010_ford_model_t_1909/compare_shocks.py [--record]
Run after run.py --stage full and run.py --stage shocks. Same course, driver and gear as
bumpy_road.py (low, ~6.4 m/s); both cars use their own calibrated springs.

Checks, fixed before the first run (the shocks car must stay a working car):
- no forbidden contacts, upright, keeps going, crosses the course, never on the stops
Reported, not gated (friction dampers can go either way on small bumps):
- RMS seat vertical acceleration on the course
- bounce after the course: RMS seat vertical velocity over the flat road after the last bar
- time with a wheel off the ground; peak suspension travel
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import numpy as np

import bumpy_road as bumpy   # sets up this car's modules and the shared adapter
import mujoco
from control import ENGINE, GOVERNED, MAX_TORQUE, Controller, Steer

HERE = Path(__file__).resolve().parent
vehicle = bumpy.vehicle


def build(stage):
    physics, g, _ = bumpy.geometries(stage)
    ratios = {k: n*ENGINE['axle_ratio'] for k, n in ENGINE['gearbox_ratios'].items()}
    model = vehicle.make_model(g, physics, 0, engine=dict(torque_nm=MAX_TORQUE, ratios=ratios), extra=bumpy.course(),
                               path_name='compare.xml')
    return physics, g, model, ratios


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record', action='store_true')
    args = parser.parse_args()
    cars = {stage: build(stage) for stage in ('full', 'shocks')}
    results, checks = {}, {}
    for stage, (physics, g, model, ratios) in cars.items():
        rows, r = bumpy.run(model, g)
        after = rows[rows[:, 1] > bumpy.END_X+1]            # flat road after the last bar
        r['bounce_after_course_seat_vz_rms_mps'] = float(np.sqrt(np.mean(after[:, 4]**2))) if len(after) else None
        r['source_sha256'] = g['source_sha256']
        results[stage] = r
        governed_low = GOVERNED/ratios['low']*g['parameters']['rear_radius']
        checks[stage] = dict(no_forbidden_contacts=not r['forbidden_contacts'], upright=r['max_tilt_deg'] < 10,
                             keeps_going=r['final_speed_mps'] > .8*governed_low, crosses_course=r['final_x_m'] > bumpy.END_X,
                             within_travel=max(r['suspension_travel_used'].values()) < 1.0)
        np.savetxt(physics/'compare_shocks.csv', rows, delimiter=',',
                   header='time,x,y,speed,seat_vz,wheels_on_ground', comments='')
    base, shock = results['full'], results['shocks']
    change = {k: (shock[k]-base[k])/base[k] for k in ('rms_seat_vertical_acc_g', 'peak_seat_vertical_acc_g',
                                                      'wheel_off_ground_fraction', 'bounce_after_course_seat_vz_rms_mps')}
    out = dict(mujoco_version=mujoco.__version__, bars=bumpy.BARS, results=results, checks=checks,
               relative_change_with_shocks=change, passed=all(all(c.values()) for c in checks.values()))
    target = cars['shocks'][0]/'compare_shocks.json'
    target.write_text(json.dumps(out, indent=2, default=lambda o: o.item())+'\n')
    print(json.dumps(dict(checks=checks, change=change,
                          full={k: v for k, v in base.items() if k not in ('forbidden_contacts', 'source_sha256')},
                          shocks={k: v for k, v in shock.items() if k not in ('forbidden_contacts', 'source_sha256')}),
                     indent=1, default=lambda o: o.item()))
    if args.record:
        record({s: (c[2], c[1]) for s, c in cars.items()}, cars['shocks'][0])
    if not out['passed']:
        raise SystemExit(1)


def record(cars, physics):
    width, height, fps = 640, 480, 30
    names = ('full', 'shocks')
    models = {n: cars[n][0] for n in names}
    datas = {n: mujoco.MjData(models[n]) for n in names}
    ctrls = {n: Controller(gears=[(.5, 'low')], steer=Steer(hold_heading=True)) for n in names}
    avgs = {n: bumpy.arrows.ForceAverager(.05) for n in names}
    renderers = {n: mujoco.Renderer(models[n], height, width) for n in names}
    options = mujoco.MjvOption()
    options.geomgroup[3] = 0
    esc = lambda text: text.replace(':', '\\:').replace(',', '\\,')
    labels = [f"drawtext=text='{esc(t)}':x={16+i*width}:y=14:fontcolor=white:fontsize=20"
              for i, t in enumerate(('AS BUILT: LEAF SPRINGS ONLY', 'AFTERMARKET FRICTION SHOCKS'))]
    labels += [f"drawtext=text='{esc('Model T 1909 study, low gear ~6.4 m/s; bars 30-50 mm, washboard 15 mm')}':"
               f"x=16:y={height-52}:fontcolor=white:fontsize=16",
               f"drawtext=text='{esc('Hartford-type scissor shocks, 40 N m knee friction each; arrows 50 ms average')}':"
               f"x=16:y={height-28}:fontcolor=white:fontsize=16"]
    out = physics/'compare_shocks.mp4'
    proc = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                             '-s', f'{width*2}x{height}', '-r', str(fps), '-i', '-', '-vf', ','.join(labels),
                             '-an', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', str(out)], stdin=subprocess.PIPE)
    try:
        for frame in range(round(bumpy.SECONDS*fps)+1):
            images = []
            for n in names:
                m, d = models[n], datas[n]
                while d.time < frame/fps:
                    ctrls[n](m, d)
                    mujoco.mj_step(m, d)
                    avgs[n].sample(m, d)
                mujoco.mj_forward(m, d)
                cam = mujoco.MjvCamera()
                cam.distance, cam.azimuth, cam.elevation = 5.0, 60, -10   # front quarter: shows the front shocks
                cam.lookat[:] = [d.body('chassis').xpos[0]+1.6, 0, .55]
                renderers[n].update_scene(d, camera=cam, scene_option=options)
                avgs[n].draw(renderers[n].scene)
                images.append(renderers[n].render())
            proc.stdin.write(np.concatenate(images, axis=1).tobytes())
    finally:
        proc.stdin.close()
        status = proc.wait()
        for r in renderers.values():
            r.close()
    if status:
        raise SystemExit(status)
    print(out)


if __name__ == '__main__':
    main()
