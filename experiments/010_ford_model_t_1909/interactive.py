"""Interactive Model T: drive it over the bump course and change the springs while it runs.

.venv/bin/python experiments/010_ford_model_t_1909/interactive.py            # bump course
.venv/bin/python experiments/010_ford_model_t_1909/interactive.py --flat     # flat road
.venv/bin/python experiments/010_ford_model_t_1909/interactive.py --selftest # no window: scripted check

Run after run.py --stage full (the calibrated spring preloads come from its report).
Uses the tested build. The only additions are zero-force 'ui_' actuators, whose sliders
appear in the viewer's right-hand Control panel. Everything else is the validated model.

Sliders (right panel, Control):
  ui_gear            0 neutral, 1 low, 2 high (engine governed; clutch engages over 0.5 s)
  ui_throttle        0-1, scales engine torque
  ui_foot_brake      0-1, transmission brake (through the differential)
  ui_hand_brake      0-1, hub bands (per rear wheel)
  ui_driver          1 = driver holds the start line, 0 = use ui_steer_deg
  ui_steer_deg       -25..25 at the pitman (x4 at the steering wheel)
  ui_front_rate      front spring rate at the axle, kN/m (design 28)
  ui_rear_rate       rear spring rate at the axle, kN/m (design 26)
  ui_front_damping   front damping at the axle, kN s/m (design 1.56)
  ui_rear_damping    rear damping at the axle, kN s/m (design 1.55)
  ui_keep_ride_height 1 = re-preload springs so ride height stays at design (like
                     re-arching the leaves); 0 = same free shape, so a softer spring sags
Roll stiffness and roll damping scale with each axle's rate and damping: one transverse
spring provides both. Backspace (viewer reset) puts the car back at the start with the
design settings. Ctrl + right-drag on the body pushes it (viewer perturbation).
"""
import argparse
from collections import deque
import importlib.util
import math
import os
os.environ['MUJOCO_GL'] = 'glfw'   # on-screen viewer (the recorders use EGL); must precede importing mujoco
from pathlib import Path
import sys
import time
import numpy as np
import mujoco

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('bumpy', HERE/'bumpy_road.py')
bumpy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bumpy)
vehicle = bumpy.vehicle
from control import ENGINE, MAX_TORQUE, Steer, engine_torque

DESIGN = dict(ui_gear=0, ui_throttle=1, ui_foot_brake=0, ui_hand_brake=0, ui_driver=1, ui_steer_deg=0,
              ui_front_rate=28, ui_rear_rate=26, ui_front_damping=1.56, ui_rear_damping=1.55,
              ui_keep_ride_height=1)
RANGES = dict(ui_gear=(0, 2), ui_throttle=(0, 1), ui_foot_brake=(0, 1), ui_hand_brake=(0, 1), ui_driver=(0, 1),
              ui_steer_deg=(-25, 25), ui_front_rate=(5, 120), ui_rear_rate=(5, 120), ui_front_damping=(0, 6),
              ui_rear_damping=(0, 6), ui_keep_ride_height=(0, 1))


def build(flat):
    physics, g, _ = bumpy.geometries()
    ratios = {k: n*ENGINE['axle_ratio'] for k, n in ENGINE['gearbox_ratios'].items()}
    path_name = 'interactive_base.xml'
    vehicle.make_model(g, physics, 0, engine=dict(torque_nm=MAX_TORQUE, ratios=ratios),
                       extra=() if flat else bumpy.course(), path_name=path_name)
    spec = mujoco.MjSpec.from_file(str(physics/path_name))
    for name in DESIGN:   # zero-gain actuators: sliders only, no force on the car
        a = spec.add_actuator(name=name, target='rear_left_joint', trntype=mujoco.mjtTrn.mjTRN_JOINT)
        a.gainprm[0] = 0.0
        a.ctrllimited = True
        a.ctrlrange = RANGES[name]
    return spec.compile(), g


class Car:
    """Applies the UI sliders to the model each step and keeps the readouts."""

    def __init__(self, model, geometry):
        self.m, self.g = model, geometry
        sus = geometry['suspension']
        self.lever = dict(front=sus['front_radius_rod_m'], rear=sus['radius_rod_m'])
        self.ui = {n: model.actuator(n).id for n in DESIGN}
        self.joints = {}
        for axle in ('front', 'rear'):
            for part in ('swing', 'roll'):
                j = model.joint(f'{axle}_axle_{part}')
                self.joints[axle, part] = dict(id=j.id, dof=j.dofadr[0], qpos=j.qposadr[0],
                                               k0=float(model.jnt_stiffness[j.id]), c0=float(model.dof_damping[j.dofadr[0]]),
                                               ref0=float(model.qpos_spring[j.qposadr[0]]))
        self.steer = Steer(hold_heading=True, start=0.0, seconds=1e-9)
        self.gear, self.since = 0, 0.0
        self.seat = deque(maxlen=2000)       # (time, seat vertical velocity), last 2 s
        self.peak = dict(front=0.0, rear=0.0)

    def defaults(self, d):
        for n, v in DESIGN.items():
            d.ctrl[self.ui[n]] = v
        self.gear, self.since = 0, 0.0
        self.seat.clear()
        self.peak = dict(front=0.0, rear=0.0)

    def value(self, d, name):
        lo, hi = RANGES[name]
        return float(np.clip(d.ctrl[self.ui[name]], lo, hi))

    def apply(self, d):
        m = self.m
        keep = self.value(d, 'ui_keep_ride_height') > .5
        for axle in ('front', 'rear'):
            rate, damp = self.value(d, f'ui_{axle}_rate'), self.value(d, f'ui_{axle}_damping')
            scale_k = rate/DESIGN[f'ui_{axle}_rate']
            scale_c = damp/DESIGN[f'ui_{axle}_damping']
            for part in ('swing', 'roll'):
                j = self.joints[axle, part]
                k = j['k0']*scale_k
                m.jnt_stiffness[j['id']] = k
                m.dof_damping[j['dof']] = j['c0']*scale_c
                # Same preload torque (ride height) or same free shape (the spring sags or rises).
                m.qpos_spring[j['qpos']] = j['ref0']*j['k0']/k if keep else j['ref0']
        # Gearbox: a new gear re-engages over the clutch time.
        gear = int(round(self.value(d, 'ui_gear')))
        if gear != self.gear:
            self.gear, self.since = gear, d.time
        throttle = self.value(d, 'ui_throttle')
        for i, name in enumerate(ENGINE['gearbox_ratios'], start=1):
            act = m.actuator(f'drive_{name}').id
            d.ctrl[act] = 0.0
            if gear == i:
                engage = min(1.0, (d.time-self.since)/ENGINE['clutch_seconds'])
                d.ctrl[act] = engage*throttle*engine_torque(float(d.actuator_velocity[act]))
        foot, hand = self.value(d, 'ui_foot_brake'), self.value(d, 'ui_hand_brake')
        m.tendon_frictionloss[m.tendon('differential').id] = foot*ENGINE['foot_brake_capacity_nm_at_wheels']
        for wheel in ('rear_left', 'rear_right'):
            m.dof_frictionloss[m.joint(f'{wheel}_joint').dofadr[0]] = hand*ENGINE['rear_drum_capacity_nm_each']
        steer = math.radians(self.value(d, 'ui_steer_deg'))
        if self.value(d, 'ui_driver') > .5:
            steer = self.steer.target(d.time, d)
        d.ctrl[m.actuator('column').id] = steer

    def observe(self, d):
        self.seat.append((d.time, bumpy.seat_velocity_z(self.m, d)))
        for axle in ('front', 'rear'):
            j = self.joints[axle, 'swing']
            self.peak[axle] = max(self.peak[axle], abs(d.qpos[j['qpos']])/self.m.jnt_range[j['id']][1])

    def readout(self, d):
        speed = float(np.linalg.norm(d.qvel[:2]))
        s = np.array(self.seat)
        rms = 0.0
        if len(s) > 60:
            n = 20
            v = np.convolve(s[:, 1], np.ones(n)/n, mode='valid')
            a = np.diff(v)/np.diff(s[n-1:, 0])
            rms = float(np.sqrt(np.mean(a**2))/9.81)
        _, ground = bumpy.velo.contacts(self.m, d)
        rows = [('Speed', f'{speed*3.6:5.1f} km/h'),
                ('Gear', ('neutral', 'low', 'high')[self.gear]),
                ('Position', f'{d.qpos[0]:6.1f} m  (bars 10-55 m)')]
        for axle in ('front', 'rear'):
            j = self.joints[axle, 'swing']
            # Axle travel relative to the body, up (compressed) positive: a positive swing lowers
            # the front axle (ahead of its ball) and raises the rear one (behind its ball).
            sag = d.qpos[j['qpos']]*self.lever[axle]*(1 if axle == 'rear' else -1)
            used = abs(d.qpos[j['qpos']])/self.m.jnt_range[j['id']][1]
            rows.append((f'{axle.title()} spring',
                         f'{self.value(d, f"ui_{axle}_rate"):5.1f} kN/m, {self.value(d, f"ui_{axle}_damping"):.2f} kN s/m'))
            rows.append((f'{axle.title()} axle', f'{sag*1000:+6.1f} mm (+ up) vs design, travel {used:4.0%} (peak {self.peak[axle]:4.0%})'
                         + ('  ON STOPS' if used >= 1.0 else '')))
        rows += [('Ride height', 'kept (preload adjusted)' if self.value(d, 'ui_keep_ride_height') > .5
                  else 'free (softer sags)'),
                 ('Seat RMS accel, 2 s', f'{rms:5.2f} g'),
                 ('Wheels on ground', f'{len(ground)} of 4')]
        return '\n'.join(r[0] for r in rows), '\n'.join(r[1] for r in rows)


def selftest(model, geometry):
    """Scripted run without a window: exercises every slider path and prints readouts."""
    d = mujoco.MjData(model)
    car = Car(model, geometry)
    car.defaults(d)
    script = [(0.5, 'ui_gear', 1), (6.0, 'ui_front_rate', 10), (6.0, 'ui_rear_rate', 10),
              (9.0, 'ui_keep_ride_height', 0), (12.0, 'ui_front_damping', 0.3), (12.0, 'ui_gear', 2),
              (16.0, 'ui_foot_brake', 1), (19.0, 'ui_driver', 0), (19.0, 'ui_steer_deg', 10)]
    report_at = [5.9, 8.9, 11.9, 15.9, 18.9, 22.0]
    while d.time < 22.0:
        for t, name, v in script:
            if abs(d.time-t) < 5e-4:
                d.ctrl[car.ui[name]] = v
        car.apply(d)
        mujoco.mj_step(model, d)
        car.observe(d)
        if any(abs(d.time-t) < 5e-4 for t in report_at):
            left, right = car.readout(d)
            print(f'--- t = {d.time:.1f} s')
            for a, b in zip(left.split('\n'), right.split('\n')):
                print(f'  {a:22s} {b}')
    assert np.all(np.isfinite(d.qpos)), 'state went non-finite'
    print('SELFTEST_OK')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--flat', action='store_true', help='flat road instead of the bump course')
    parser.add_argument('--selftest', action='store_true', help='scripted run without a window')
    parser.add_argument('--seconds', type=float, default=None, help='close the viewer after this long')
    args = parser.parse_args()
    model, geometry = build(args.flat)
    if args.selftest:
        selftest(model, geometry)
        return
    import mujoco.viewer
    d = mujoco.MjData(model)
    car = Car(model, geometry)
    car.defaults(d)
    last_time = d.time
    with mujoco.viewer.launch_passive(model, d) as viewer:
        with viewer.lock():
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            viewer.cam.trackbodyid = model.body('chassis').id
            viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 6.5, 125, -14
            viewer.opt.geomgroup[3] = 0    # hide collision proxies; show the meshes
        start, frame = time.time(), 1/60
        while viewer.is_running():
            tick = time.time()
            with viewer.lock():
                if d.time < last_time:         # viewer reset (Backspace)
                    car.defaults(d)
                steps = round(frame/model.opt.timestep)
                for _ in range(steps):
                    car.apply(d)
                    mujoco.mj_step(model, d)
                    car.observe(d)
                last_time = d.time
                left, right = car.readout(d)
            # Outside the lock: set_texts waits for the render thread, which needs the lock.
            viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT, left, right))
            viewer.sync()
            if args.seconds and time.time()-start > args.seconds:
                break
            time.sleep(max(0.0, frame-(time.time()-tick)))
    print('viewer closed', flush=True)
    os._exit(0)   # MuJoCo 3.14's GLFW teardown segfaults at interpreter exit after the window closes


if __name__ == '__main__':
    main()
