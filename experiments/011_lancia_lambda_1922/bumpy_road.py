"""Lancia Lambda over the Model T's bump course: independent front against locked suspension,
and against the Model T's beam axles.

.venv/bin/python experiments/011_lancia_lambda_1922/bumpy_road.py [--record]
Run after run.py --stage full (spring preloads come from its report), and after
010's run.py --stage full for the Model T comparison. Same course, driver and gear as
010/bumpy_road.py (the Lambda in first gear, ~8.4 m/s; the Model T in low, ~6.4 m/s).

Checks, fixed before the first run:
- the sprung Lambda: only tyres touch the road and bars, tilt < 10 deg, keeps going (final
  speed > 80% of first-gear governed speed), never on the stops, crosses the course
- comparison with its suspension locked: lower RMS seat acceleration, less time with a wheel
  off the ground, and less body roll over the one-sided bars
Reported, not gated: the Model T on the same course (different speed and mass), whose beam
axles must roll the body when one wheel rises; the Lambda's pillars need not.
"""
import argparse
import copy
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import numpy as np

import mujoco

HERE = Path(__file__).resolve().parent
EXP = HERE.parent


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


bumpy10 = load('bumpy10', EXP/'010_ford_model_t_1909/bumpy_road.py')   # course, Model T geometry and control
lrun = load('lrun', HERE/'run.py')                                     # Lambda geometry, springs, shared adapter
lcontrol = load('lcontrol', HERE/'control.py')
vehicle = lrun.vehicle                                                  # the same shared module for both cars
arrows = bumpy10.arrows
LAMBDA_SEAT = np.array([1.15, -.30, .85])   # driver's cushion (right-hand drive), chassis frame
MODEL_T_SEAT = bumpy10.SEAT
BARS, END_X, SECONDS = bumpy10.BARS, bumpy10.END_X, bumpy10.SECONDS
LAMBDA_CONFIG = json.loads((HERE/'vehicle.json').read_text())
MODEL_T_CONFIG = json.loads((EXP/'010_ford_model_t_1909/vehicle.json').read_text())


def lambda_geometries():
    physics = HERE/'output/full/physics'
    g = json.loads((physics/'geometry.json').read_text())
    report = json.loads((physics/'report.json').read_text())
    assert report['source_sha256'] == g['source_sha256'], 'report is from another build: rerun run.py'
    lrun.apply_springs(g)
    for body in lrun.SPRUNG:
        for j in g['bodies'][body]['joint']:
            j['springref'] = report['springs']['springref'][f'{body}_{j["name"]}']
    rigid = copy.deepcopy(g)
    for body in lrun.SPRUNG:
        rigid['bodies'][body]['joint'] = []   # pillars and axle welded to the hull at ride height
    return physics, g, rigid


def seat_velocity_z(model, data, seat):
    R = data.body('chassis').xmat.reshape(3, 3)
    w = R@data.qvel[3:6]
    r = R@(seat-model.body_pos[model.body('chassis').id])
    return float(data.qvel[2]+np.cross(w, r)[2])


def run(model, geometry, ctrl, seat, sprung_bodies, record=None):
    data = mujoco.MjData(model)
    dt = model.opt.timestep
    wheels = [n for n, b in geometry['bodies'].items() if 'wheel' in b]
    joints = [f'{b}_{j["name"]}' for b in sprung_bodies for j in geometry['bodies'][b]['joint']]
    rows, bad, airborne, on_course = [], set(), 0, 0
    used = dict.fromkeys(joints, 0.0)
    tilt = roll_max = 0.0
    while data.time < SECONDS:
        ctrl(model, data)
        mujoco.mj_step(model, data)
        pairs, ground = bumpy10.velo.contacts(model, data)
        bad |= pairs
        x = float(data.qpos[0])
        R = data.body('chassis').xmat.reshape(3, 3)
        roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))   # about the car's x axis
        if BARS[0][0]-3 < x < END_X:
            on_course += 1
            airborne += len(ground) < len(wheels)
            roll_max = max(roll_max, abs(roll))
        for n in joints:
            j = model.joint(n)
            used[n] = max(used[n], abs(float(data.qpos[j.qposadr[0]]))/j.range[1])
        tilt = max(tilt, math.degrees(math.acos(min(1.0, R[2, 2]))))
        rows.append((data.time, x, float(data.qpos[1]), float(np.linalg.norm(data.qvel[:2])),
                     seat_velocity_z(model, data, seat), len(ground), roll))
        if record:
            record(model, data)
    rows = np.array(rows)
    n = max(1, round(.02/dt))
    v = np.convolve(rows[:, 4], np.ones(n)/n, mode='same')
    acc = np.gradient(v, rows[:, 0])
    on = (rows[:, 1] > BARS[0][0]-3) & (rows[:, 1] < END_X)
    return rows, dict(
        rms_seat_vertical_acc_g=float(np.sqrt(np.mean(acc[on]**2))/9.81),
        peak_seat_vertical_acc_g=float(np.abs(acc[on]).max()/9.81),
        wheel_off_ground_fraction=airborne/max(on_course, 1), max_body_roll_deg=roll_max,
        suspension_travel_used=used, max_tilt_deg=tilt, final_x_m=float(rows[-1, 1]),
        final_speed_mps=float(rows[-1, 3]), forbidden_contacts=sorted('/'.join(p) for p in bad))


def lambda_model(g, physics, name):
    vehicle.CONFIG = LAMBDA_CONFIG
    ratios = {k: n*lcontrol.ENGINE['final_drive_ratio'] for k, n in lcontrol.ENGINE['gearbox_ratios'].items()}
    return vehicle.make_model(g, physics, 0, engine=dict(torque_nm=lcontrol.MAX_TORQUE, ratios=ratios),
                              extra=bumpy10.course(), path_name=f'bumpy_{name}.xml')


def model_t_model():
    vehicle.CONFIG = MODEL_T_CONFIG          # the shared adapter serves both cars: switch its config first
    physics, g, _ = bumpy10.geometries()
    ratios = {k: n*bumpy10.ENGINE['axle_ratio'] for k, n in bumpy10.ENGINE['gearbox_ratios'].items()}
    return g, vehicle.make_model(g, physics, 0, engine=dict(torque_nm=bumpy10.MAX_TORQUE, ratios=ratios),
                                 extra=bumpy10.course(), path_name='bumpy_vs_lambda.xml')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record', action='store_true')
    args = parser.parse_args()
    physics, sprung_g, rigid_g = lambda_geometries()
    lam = lambda: lcontrol.Controller(gears=[(.5, 'first')], steer=lcontrol.Steer(hold_heading=True))
    results, runs, models = {}, {}, {}
    for name, g in (('sprung', sprung_g), ('locked', rigid_g)):
        models[name] = lambda_model(g, physics, name)
        runs[name], results[name] = run(models[name], g, lam(), LAMBDA_SEAT, lrun.SPRUNG)
    gt, mt = model_t_model()
    tcontrol = bumpy10.Controller(gears=[(.5, 'low')], steer=bumpy10.Steer(hold_heading=True))
    runs['model_t'], results['model_t'] = run(mt, gt, tcontrol, MODEL_T_SEAT, ('front_axle', 'rear_axle'))
    governed_first = lcontrol.GOVERNED/(lcontrol.ENGINE['gearbox_ratios']['first']*lcontrol.ENGINE['final_drive_ratio'])
    governed_first *= sprung_g['parameters']['rear_radius']
    s, r = results['sprung'], results['locked']
    checks = dict(no_forbidden_contacts=not s['forbidden_contacts'], upright=s['max_tilt_deg'] < 10,
                  keeps_going=s['final_speed_mps'] > .8*governed_first, crosses_course=s['final_x_m'] > END_X,
                  within_travel=max(s['suspension_travel_used'].values()) < 1.0,
                  smoother_seat=s['rms_seat_vertical_acc_g'] < r['rms_seat_vertical_acc_g'],
                  wheels_stay_down_more=s['wheel_off_ground_fraction'] < r['wheel_off_ground_fraction'],
                  rolls_less=s['max_body_roll_deg'] < r['max_body_roll_deg'])
    out = dict(source_sha256=sprung_g['source_sha256'], model_t_sha256=gt['source_sha256'],
               mujoco_version=mujoco.__version__, bars=BARS, results=results, checks=checks,
               passed=all(checks.values()))
    (physics/'bumpy_road.json').write_text(json.dumps(out, indent=2, default=lambda o: o.item())+'\n')
    for name, rows in runs.items():
        np.savetxt(physics/f'bumpy_road_{name}.csv', rows, delimiter=',',
                   header='time,x,y,speed,seat_vz,wheels_on_ground,roll_deg', comments='')
    print(json.dumps(dict(checks=checks, **{k: {kk: vv for kk, vv in v.items() if kk != 'forbidden_contacts'}
                                             for k, v in results.items()}), indent=1, default=lambda o: o.item()))
    if args.record:
        record_video(physics, models, sprung_g, rigid_g)
    if not out['passed']:
        raise SystemExit(1)


def record_video(physics, models, sprung_g, rigid_g):
    width, height, fps = 640, 480, 30
    names = ('sprung', 'locked')
    vehicle.CONFIG = LAMBDA_CONFIG
    datas = {n: mujoco.MjData(models[n]) for n in names}
    ctrls = {n: lcontrol.Controller(gears=[(.5, 'first')], steer=lcontrol.Steer(hold_heading=True)) for n in names}
    avgs = {n: arrows.ForceAverager(.05) for n in names}
    renderers = {n: mujoco.Renderer(models[n], height, width) for n in names}
    options = mujoco.MjvOption()
    options.geomgroup[3] = 0
    esc = lambda text: text.replace(':', '\\:').replace(',', '\\,')
    labels = [f"drawtext=text='{esc(t)}':x={16+i*width}:y=14:fontcolor=white:fontsize=20"
              for i, t in enumerate(('SLIDING PILLARS AND REAR SPRINGS', 'CONTROL: SUSPENSION LOCKED'))]
    labels += [f"drawtext=text='{esc('Lancia Lambda 1922 study, first gear ~8.4 m/s; bars 30-50 mm (some one-sided), washboard 15 mm')}':"
               f"x=16:y={height-52}:fontcolor=white:fontsize=16",
               f"drawtext=text='{esc('Arrows: blue = tire load (1 m per 5000 N), orange = road force (1 m per 1000 N); 50 ms average')}':"
               f"x=16:y={height-28}:fontcolor=white:fontsize=16"]
    out = physics/'bumpy_road.mp4'
    proc = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                             '-s', f'{width*2}x{height}', '-r', str(fps), '-i', '-', '-vf', ','.join(labels),
                             '-an', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', str(out)], stdin=subprocess.PIPE)
    arrows.LOAD_M_PER_N, arrows.TIRE_M_PER_N = .0002, .001
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
                cam.distance, cam.azimuth, cam.elevation = 6.5, 60, -10   # front quarter: the pillars in view
                cam.lookat[:] = [d.body('chassis').xpos[0]+1.8, 0, .6]
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
