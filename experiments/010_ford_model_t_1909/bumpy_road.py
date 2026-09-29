"""Model T over a bumpy road: the sprung car against the same car with its axles locked.

.venv/bin/python experiments/010_ford_model_t_1909/bumpy_road.py [--record]
Run after run.py --stage full (spring preloads come from its report). Uses the tested
build unchanged; the rigid control only welds the two axles to the frame.

Course (low gear, ~6.4 m/s, pure-pursuit driver holding the start line): full-width,
left-only and right-only bars of 30-50 mm, a pair spaced near the wheelbase, and a
15 mm washboard. A bar is a capsule lying on the road, so its radius is its height.

Checks, fixed before the first run:
- sprung car: no forbidden contacts (only tyres touch the road and bars), upright
  (tilt < 10 deg), keeps going (final speed > 80% of low-gear governed speed),
  never on the stops (travel used < 100%), crosses the whole course
- comparison: the sprung car's RMS vertical acceleration at the driver's seat and its
  time with a wheel off the ground are both lower than the rigid car's
"""
import argparse
import copy
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import numpy as np

os.environ.setdefault('MUJOCO_GL', 'egl')
import mujoco

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('trun', HERE/'run.py')
trun = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trun)            # this car's vehicle config and the shared adapter
vehicle, velo = trun.vehicle, trun.velo
from control import ENGINE, GOVERNED, MAX_TORQUE, Controller, Steer

spec = importlib.util.spec_from_file_location('arrows', HERE.parent/'005_engine_drive/visual.py')
arrows = importlib.util.module_from_spec(spec)
spec.loader.exec_module(arrows)
arrows.LOAD_M_PER_N, arrows.TIRE_M_PER_N = .0005, .002

SEAT = np.array([1.19, .30, 1.0])         # driver's cushion (left-hand drive), chassis frame
SECONDS = 13.0
BARS = [(10.0, .040, 'full'), (16.0, .040, 'left'), (21.0, .040, 'right'), (27.0, .030, 'full'),
        (29.5, .030, 'full'), (36.0, .050, 'left'), (38.5, .050, 'right')]
BARS += [(45.0+i, .015, 'full') for i in range(11)]          # washboard, 1 m pitch
END_X = 58.0


def course():
    span = dict(full=(-1.3, 1.3), left=(.25, 1.3), right=(-1.3, -.25))
    return [('world', dict(name=f'road_bar_{i}', type='capsule', size=f'{h}',
                           fromto=vehicle.numbers([x, span[side][0], 0, x, span[side][1], 0])))
            for i, (x, h, side) in enumerate(BARS)]


def geometries():
    physics = HERE/'output/full/physics'
    g = json.loads((physics/'geometry.json').read_text())
    report = json.loads((physics/'report.json').read_text())
    assert report['source_sha256'] == g['source_sha256'], 'report is from another build: rerun run.py'
    trun.apply_springs(g)
    for body in ('front_axle', 'rear_axle'):
        for j in g['bodies'][body]['joint']:
            j['springref'] = report['springs']['springref'][f'{body}_{j["name"]}']
    rigid = copy.deepcopy(g)
    for body in ('front_axle', 'rear_axle'):
        rigid['bodies'][body]['joint'] = []   # welded to the frame at ride height
    return physics, g, rigid


def seat_velocity_z(model, data):
    R = data.body('chassis').xmat.reshape(3, 3)
    w = R@data.qvel[3:6]
    r = R@(SEAT-model.body_pos[model.body('chassis').id])
    return float(data.qvel[2]+np.cross(w, r)[2])


def run(model, geometry, record=None):
    data = mujoco.MjData(model)
    ctrl = Controller(gears=[(.5, 'low')], steer=Steer(hold_heading=True))
    dt = model.opt.timestep
    wheels = [n for n, b in geometry['bodies'].items() if 'wheel' in b]
    joints = [f'{b}_{j["name"]}' for b in ('front_axle', 'rear_axle') for j in geometry['bodies'][b]['joint']]
    rows, bad, airborne_steps, steps_on_course = [], set(), 0, 0
    used = dict.fromkeys(joints, 0.0)
    tilt = 0.0
    while data.time < SECONDS:
        ctrl(model, data)
        mujoco.mj_step(model, data)
        pairs, ground = velo.contacts(model, data)
        bad |= pairs
        x = float(data.qpos[0])
        if BARS[0][0]-3 < x < END_X:
            steps_on_course += 1
            airborne_steps += len(ground) < len(wheels)
        for n in joints:
            j = model.joint(n)
            used[n] = max(used[n], abs(float(data.qpos[j.qposadr[0]]))/j.range[1])
        z_axis = data.body('chassis').xmat.reshape(3, 3)[:, 2]
        tilt = max(tilt, math.degrees(math.acos(min(1.0, z_axis[2]))))
        rows.append((data.time, x, float(data.qpos[1]), float(np.linalg.norm(data.qvel[:2])),
                     seat_velocity_z(model, data), len(ground)))
        if record:
            record(model, data)
    rows = np.array(rows)
    # Seat acceleration from 20 ms-averaged velocity (removes contact-solver noise, keeps < ~25 Hz).
    n = max(1, round(.02/dt))
    v = np.convolve(rows[:, 4], np.ones(n)/n, mode='same')
    acc = np.gradient(v, rows[:, 0])
    on = (rows[:, 1] > BARS[0][0]-3) & (rows[:, 1] < END_X)
    return rows, dict(
        rms_seat_vertical_acc_g=float(np.sqrt(np.mean(acc[on]**2))/9.81),
        peak_seat_vertical_acc_g=float(np.abs(acc[on]).max()/9.81),
        wheel_off_ground_fraction=airborne_steps/max(steps_on_course, 1),
        suspension_travel_used=used, max_tilt_deg=tilt, final_x_m=float(rows[-1, 1]),
        final_speed_mps=float(rows[-1, 3]), lateral_m=float(rows[-1, 2]),
        forbidden_contacts=sorted('/'.join(p) for p in bad))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record', action='store_true')
    args = parser.parse_args()
    physics, sprung_g, rigid_g = geometries()
    ratios = {g: n*ENGINE['axle_ratio'] for g, n in ENGINE['gearbox_ratios'].items()}
    engine = dict(torque_nm=MAX_TORQUE, ratios=ratios)
    models = {name: vehicle.make_model(g, physics, 0, engine=engine, extra=course(), path_name=f'bumpy_{name}.xml')
              for name, g in (('sprung', sprung_g), ('rigid', rigid_g))}
    results, runs = {}, {}
    for name, g in (('sprung', sprung_g), ('rigid', rigid_g)):
        runs[name], results[name] = run(models[name], g)
    governed_low = GOVERNED/ratios['low']*geometry_radius(sprung_g)
    s, r = results['sprung'], results['rigid']
    checks = dict(no_forbidden_contacts=not s['forbidden_contacts'], upright=s['max_tilt_deg'] < 10,
                  keeps_going=s['final_speed_mps'] > .8*governed_low, crosses_course=s['final_x_m'] > END_X,
                  within_travel=max(s['suspension_travel_used'].values()) < 1.0,
                  smoother_seat=s['rms_seat_vertical_acc_g'] < r['rms_seat_vertical_acc_g'],
                  wheels_stay_down_more=s['wheel_off_ground_fraction'] < r['wheel_off_ground_fraction'])
    out = dict(source_sha256=json.loads((physics/'geometry.json').read_text())['source_sha256'],
               mujoco_version=mujoco.__version__, bars=BARS, seat_point=SEAT.tolist(), governed_low_mps=governed_low,
               results=results, checks=checks, passed=all(checks.values()))
    (physics/'bumpy_road.json').write_text(json.dumps(out, indent=2, default=lambda o: o.item())+'\n')
    for name, rows in runs.items():
        np.savetxt(physics/f'bumpy_road_{name}.csv', rows, delimiter=',',
                   header='time,x,y,speed,seat_vz,wheels_on_ground', comments='')
    print(json.dumps(dict(checks=checks, sprung={k: v for k, v in s.items() if k != 'forbidden_contacts'},
                          rigid={k: v for k, v in r.items() if k != 'forbidden_contacts'},
                          rigid_forbidden=r['forbidden_contacts']), indent=1, default=lambda o: o.item()))
    if args.record:
        record_video(physics, models, (sprung_g, rigid_g))
    if not out['passed']:
        raise SystemExit(1)


def geometry_radius(g):
    return g['parameters']['rear_radius']


def record_video(physics, models, geoms):
    width, height, fps = 640, 480, 30
    names = ('sprung', 'rigid')
    datas = {n: mujoco.MjData(models[n]) for n in names}
    ctrls = {n: Controller(gears=[(.5, 'low')], steer=Steer(hold_heading=True)) for n in names}
    avgs = {n: arrows.ForceAverager(.05) for n in names}
    renderers = {n: mujoco.Renderer(models[n], height, width) for n in names}
    options = mujoco.MjvOption()
    options.geomgroup[3] = 0
    esc = lambda text: text.replace(':', '\\:').replace(',', '\\,')
    labels = [f"drawtext=text='{esc(t)}':x={16+i*width}:y=14:fontcolor=white:fontsize=20"
              for i, t in enumerate(('SPRUNG: TRANSVERSE LEAF SPRINGS', 'CONTROL: AXLES LOCKED TO FRAME'))]
    labels += [f"drawtext=text='{esc('Model T 1909 study, low gear ~6.4 m/s; bars 30-50 mm, washboard 15 mm')}':"
               f"x=16:y={height-52}:fontcolor=white:fontsize=16",
               f"drawtext=text='{esc('Arrows: blue = tire load (1 m per 2000 N), orange = road force (1 m per 500 N); 50 ms average')}':"
               f"x=16:y={height-28}:fontcolor=white:fontsize=16"]
    out = physics/'bumpy_road.mp4'
    proc = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                             '-s', f'{width*2}x{height}', '-r', str(fps), '-i', '-', '-vf', ','.join(labels),
                             '-an', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', str(out)], stdin=subprocess.PIPE)
    try:
        for frame in range(round(SECONDS*fps)+1):
            images = []
            for n in names:
                m, d = models[n], datas[n]
                while d.time < frame/fps:
                    ctrls[n](m, d)
                    mujoco.mj_step(m, d)
                    avgs[n].sample(m, d)
                mujoco.mj_forward(m, d)
                cam = mujoco.MjvCamera()
                cam.distance, cam.azimuth, cam.elevation = 5.5, 80, -8   # low side view: wheels against body
                cam.lookat[:] = [d.body('chassis').xpos[0]+1.2, 0, .55]
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
