"""Citroën Traction Avant (1934) validation: build checks, downhill gate, scenarios.

.venv/bin/python experiments/012_citroen_traction_avant_1934/run.py --stage chassis   # gate before drivetrain/bodywork
.venv/bin/python experiments/012_citroen_traction_avant_1934/run.py --stage full
Reuses experiment 006's exporter, MuJoCo adapter, simulation loop and build checks.
"""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
VELO = HERE.parent/'006_benz_velo'
spec = importlib.util.spec_from_file_location('velo_run', VELO/'run.py')
velo = importlib.util.module_from_spec(spec)
spec.loader.exec_module(velo)          # puts 006 on sys.path and imports its vehicle adapter
vehicle = velo.vehicle
vehicle.CONFIG = json.loads((HERE/'vehicle.json').read_text())
sys.path.insert(0, str(HERE))          # this car's control/dynamics come first
SPRUNG = ('lower_arm_left', 'lower_arm_right', 'rear_axle')   # bodies whose joints carry the springs


def export(stage):
    out = HERE/'output'/stage
    subprocess.run(['blender', '--background', str(out/'traction_avant.blend'), '--python-exit-code', '1', '--python',
                    str(VELO/'export_blender.py'), '--', str(out/'physics')], check=True)
    return out, json.loads((out/'physics/geometry.json').read_text())


def apply_springs(geometry):
    """Put the spring rates and calibrated preloads from vehicle.json on the axle joints."""
    for body in SPRUNG:
        for j in geometry['bodies'][body]['joint']:
            j.update(vehicle.CONFIG['suspension'][f'{body}_{j["name"]}'])


def calibrate_springs(geometry, physics, iterations=4):
    """Choose each spring's preload so the loaded car settles at the design ride height
    (the joints' zero), as a car's springs are made. Returns preloads and static deflections."""
    names = [f'{b}_{j["name"]}' for b in SPRUNG for j in geometry['bodies'][b]['joint']]
    for _ in range(iterations):
        model = vehicle.make_model(geometry, physics, 0, path_name='calibrate.xml')
        data = mujoco.MjData(model)
        for _ in range(4000):
            mujoco.mj_step(model, data)
        settled = {n: float(data.qpos[model.joint(n).qposadr[0]]) for n in names}
        for b in SPRUNG:
            for j in geometry['bodies'][b]['joint']:
                j['springref'] = j.get('springref', 0.0)-settled[f'{b}_{j["name"]}']
    refs = {f'{b}_{j["name"]}': j['springref'] for b in SPRUNG for j in geometry['bodies'][b]['joint']}
    p = geometry['parameters']      # hinges: the preload angle times the arm is the deflection at the wheel
    return dict(residual_after_calibration=settled, springref=refs,
                static_deflection_front_m=(abs(refs['lower_arm_left_swing'])+abs(refs['lower_arm_right_swing']))/2*p['arm_length'],
                static_deflection_rear_m=abs(refs['rear_axle_swing'])*p['trailing_arm_m'])


def check_controls(geometry, physics):
    """Loose join, same-body intersection and sweep intrusion must each be caught."""
    model = vehicle.make_model(geometry, physics, path_name='controls.xml')
    ids = lambda prefix: [g for g in range(model.ngeom) if model.geom(g).name.startswith(prefix+':')]

    def shifted(prefix, delta):
        g = ids(prefix)
        saved = model.geom_pos[g].copy()
        model.geom_pos[g] += delta
        try:
            return vehicle.build_checks(model, geometry, step_deg=15)
        finally:
            model.geom_pos[g] = saved
    out = {}
    loose = shifted('Lower pin 1', [0, 0, .1])   # (box parts' colliders carry no ':n' suffix; a rod does)
    out['loose_join_detected'] = any('Lower pin 1' in k for k in map(tuple, loose['loose_joins']))
    clash = shifted('Steering arm 1', [.035, .07, .06])      # onto the stub axle, both on the left knuckle
    out['same_body_intersection_detected'] = any(set(f['pair']) == {'Steering arm 1', 'Stub axle 1'}
                                                 for f in clash['failures'])
    limit = math.radians(geometry['steering']['column_limit_deg'])
    lock = {'knuckle_left': vehicle.suspended_pose(geometry, limit, {})['knuckle_left_joint']}
    s = geometry['steering']
    k, a0 = np.array(s['kingpins']['left']), np.array(s['arm_balls']['left'])
    ball = k+vehicle._rot(a0-k, lock['knuckle_left'])
    blocked = vehicle.make_model(geometry, physics, extra=[('chassis', dict(
        name='sweep_blocker', type='box', size='.01 .01 .01',
        pos=vehicle.numbers([*ball, geometry['parameters']['track_rod_z']])))], path_name='controls_blocked.xml')
    hits = [f for f in vehicle.build_checks(blocked, geometry, step_deg=1)['failures'] if 'sweep_blocker' in f['pair']]
    out['sweep_intrusion_detected'] = bool(hits) and all(f['angle_deg'] != 0 for f in hits)
    return out


def steer_from_suspension(geometry):
    """Change in left road-wheel angle caused by each suspension extreme, over the
    steering range (each track rod runs from the frame-mounted drop arm to its moving wheel)."""
    out = {}
    limit = geometry['steering']['column_limit_deg']
    for pose in geometry['suspension']['check_poses']:
        label = ','.join(f'{k}={v:+.3f}' for k, v in pose.items())
        devs = []
        for d in np.arange(-limit, limit+.1, 1.0):
            base = vehicle.suspended_pose(geometry, math.radians(d), {})['knuckle_left_joint']
            devs.append((abs(math.degrees(vehicle.suspended_pose(geometry, math.radians(d), pose)['knuckle_left_joint']
                                          - base)), float(d)))
        worst = max(devs)
        straight = [v for v, d in devs if d == 0][0]
        out[label] = dict(at_straight_ahead_deg=straight, worst_deg=worst[0], worst_at_column_deg=worst[1])
    return out


def travel_used(model, geometry, seconds, slope, control=None):
    """Largest fraction of each axle joint's travel used during a run (1.0 = on the stop)."""
    data = mujoco.MjData(model)
    names = [f'{b}_{j["name"]}' for b in SPRUNG for j in geometry['bodies'][b]['joint']]
    used = dict.fromkeys(names, 0.0)
    for _ in range(round(seconds/model.opt.timestep)):
        if control:
            control(model, data)
        mujoco.mj_step(model, data)
        for n in names:
            j = model.joint(n)
            q = data.qpos[j.qposadr[0]]
            used[n] = max(used[n], abs(q)/j.range[1])
    return used


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', choices=('chassis', 'full', 'shocks'), required=True)
    parser.add_argument('--viewer', action='store_true')
    args = parser.parse_args()
    out, geometry = export(args.stage)
    physics = out/'physics'
    report = dict(stage=args.stage, source=geometry['source'], source_sha256=geometry['source_sha256'],
                  mujoco_version=mujoco.__version__, limits=velo.LIMITS, vehicle=vehicle.CONFIG)
    apply_springs(geometry)
    report['springs'] = calibrate_springs(geometry, physics)
    report['build'] = vehicle.build_checks(vehicle.make_model(geometry, physics), geometry)
    report['bump_and_roll_steer'] = steer_from_suspension(geometry)
    report['build_controls'] = check_controls(geometry, physics)

    slope = math.radians(5)
    wheels = [n for n, b in geometry['bodies'].items() if 'wheel' in b]
    gate, common, rows = velo.simulate(vehicle.make_model(geometry, physics, slope), geometry, 4, slope,
                                       viewer=args.viewer)
    velo.write_csv(physics/'downhill.csv', rows, wheels)
    springs = report['springs']
    gate['checks'] = common|dict(downhill_progress=gate['forward_m'] > velo.LIMITS['progress_m'],
                                 # springs neither rock hard nor sagging: 20-150 mm static deflection
                                 plausible_springs=all(.02 < springs[k] < .15 for k in
                                                       ('static_deflection_front_m', 'static_deflection_rear_m')),
                                 wheels_turn_forward=all(v > 1 for v in gate['wheel_rolled_m'].values()),
                                 straight=abs(gate['lateral_m']) < .05)
    wb, fr, ft = (geometry['parameters'][k] for k in ('wheelbase', 'front_radius', 'front_track'))
    blocked = vehicle.make_model(geometry, physics, slope, extra=[('chassis', dict(
        name='deliberate_wheel_blocker', type='capsule', size='.02',
        fromto=vehicle.numbers([wb-.1, ft/2, fr*1.7, wb+.1, ft/2, fr*1.7])))], path_name='blocked.xml')
    negative, _, _ = velo.simulate(blocked, geometry, .1, slope)
    gate['negative_control_detected'] = any('deliberate_wheel_blocker' in p for p in negative['forbidden_contacts'])
    travel = travel_used(vehicle.make_model(geometry, physics, slope), geometry, 4, slope)
    gate['suspension_travel_used'] = travel
    gate['checks']['within_suspension_travel'] = all(v < .9 for v in travel.values())
    gate['passed'] = all(gate['checks'].values()) and gate['negative_control_detected']
    report['downhill_gate'] = gate
    report['passed'] = report['build']['passed'] and all(report['build_controls'].values()) and gate['passed']

    ratios = None
    if args.stage in ('full', 'shocks'):
        from control import ENGINE, MAX_TORQUE
        ratios = {g: n*ENGINE['final_drive_ratio'] for g, n in ENGINE['gearbox_ratios'].items()}
    if args.stage in ('full', 'shocks') and gate['passed']:
        import dynamics

        def make(slope, engine, spool, timestep, extra=()):
            m = vehicle.make_model(geometry, physics, slope, spool=spool, path_name='scenario.xml', extra=extra,
                                   engine=dict(torque_nm=MAX_TORQUE, ratios=ratios) if engine else None)
            if timestep:
                m.opt.timestep = timestep
            return m
        summary, scenarios, runs = dynamics.run_all(
            make, lambda m, seconds, slope, ctrl: velo.simulate(m, geometry, seconds, slope, ctrl), geometry, ratios,
            travel_used=lambda m, g, seconds, slope, ctrl: travel_used(m, g, seconds, slope, ctrl))
        (physics/'runs').mkdir(exist_ok=True)
        for name, data in runs.items():
            velo.write_csv(physics/'runs'/f'{name}.csv', data, wheels)
        report['dynamics'] = summary|dict(scenarios=scenarios)
        report['ackermann'] = [vehicle.ackermann(geometry, math.radians(d))|dict(column_deg=d) for d in range(-30, 31, 5)]
        report['passed'] &= summary['passed']
        print(json.dumps({k: dict(passed=v['passed'], failed=[c for c, ok in v.get('checks', {}).items() if not ok])
                          for k, v in scenarios.items()}, indent=1, default=lambda o: o.item()))
    vehicle.make_model(geometry, physics, engine=dict(torque_nm=MAX_TORQUE, ratios=ratios) if ratios else None)
    (physics/'report.json').write_text(json.dumps(report, indent=2, default=lambda o: o.item())+'\n')
    print(json.dumps(dict(passed=report['passed'], build={k: v for k, v in report['build'].items() if k != 'closest'},
                          controls=report['build_controls'],
                          gate=dict(passed=gate['passed'], failed=[k for k, v in gate['checks'].items() if not v],
                                    forward_m=gate['forward_m'], lateral_m=gate['lateral_m'],
                                    contacts=gate['forbidden_contacts'], negative=gate['negative_control_detected'])),
                     indent=2, default=lambda o: o.item()))
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
