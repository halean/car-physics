"""Benz Velo validation: build checks and the unpowered downhill gate.

.venv/bin/python experiments/006_benz_velo/run.py --stage chassis   # gate before bodywork
.venv/bin/python experiments/006_benz_velo/run.py --stage full
"""
import argparse
import csv
import json
import math
from pathlib import Path
import subprocess
import sys
import time
import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import vehicle

LIMITS = dict(progress_m=1.0, rolling_error=.10, tilt_deg=10.0, max_loop_violation_m=.002)


def export(stage):
    out = HERE/'output'/stage
    blend = out/'benz_velo.blend'
    subprocess.run(['blender', '--background', str(blend), '--python-exit-code', '1', '--python',
                    str(HERE/'export_blender.py'), '--', str(out/'physics')], check=True)
    return out, json.loads((out/'physics/geometry.json').read_text())


def contacts(model, data):
    bad, ground = set(), set()
    for c in data.contact:
        if c.dist > 0:
            continue
        a, b = (model.geom(g).name for g in (c.geom1, c.geom2))
        if 'slope' in (a, b):
            other = b if a == 'slope' else a
            if other.endswith('_rolling'):
                ground.add(other.removesuffix('_rolling'))
                continue
        bad.add(tuple(sorted((a, b))))
    return bad, ground


def loop_violation(model, data):
    rows = data.efc_type[:data.nefc] == mujoco.mjtConstraint.mjCNSTR_EQUALITY
    return float(np.abs(data.efc_pos[:data.nefc][rows]).max()) if rows.any() else 0.0


def simulate(model, geometry, seconds, slope_rad, control=None, viewer=False):
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    tangent = np.array([math.cos(slope_rad), 0, -math.sin(slope_rad)])
    normal = np.array([math.sin(slope_rad), 0, math.cos(slope_rad)])
    initial = data.qpos[:3].copy()
    wheels = {n: (model.joint(f'{n}_joint'), b['wheel']['radius'])
              for n, b in geometry['bodies'].items() if 'wheel' in b}
    last = {n: data.body(n).xpos.copy() for n in wheels}
    path = {n: 0.0 for n in wheels}
    bad, touched, rows = set(), set(), []
    peak = dict(tilt=0.0, loop=0.0, engine=0.0, power=0.0)
    kinematic_yaw = 0.0
    wb = geometry['parameters']['wheelbase']
    handle = None
    if viewer:
        from mujoco import viewer as mjviewer
        handle = mjviewer.launch_passive(model, data)
        handle.opt.geomgroup[3] = 0
    try:
        for step in range(round(seconds/model.opt.timestep)):
            start = time.monotonic()
            log = control(model, data) if control else {}
            # Double-pivot steering: equivalent angle from the harmonic mean of tangents.
            tl, tr = (math.tan(data.joint(f'knuckle_{s}_joint').qpos[0]) for s in ('left', 'right'))
            tan_eq = 2*tl*tr/(tl+tr) if abs(tl+tr) > 1e-9 else 0.0
            v = data.qvel[:3]
            kinematic_yaw += float(np.linalg.norm(v-(v@normal)*normal))*tan_eq/wb*model.opt.timestep
            peak['engine'] = max(peak['engine'], log.get('engine_rpm', 0.0))
            peak['power'] = max(peak['power'], log.get('torque', 0.0)*log.get('engine_rpm', 0.0)*math.tau/60)
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            if not (np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()):
                raise RuntimeError('Non-finite simulation')
            pairs, ground = contacts(model, data)
            bad |= pairs
            touched |= ground
            for n in wheels:
                pos = data.body(n).xpos.copy()
                d = pos-last[n]
                path[n] += float(np.linalg.norm(d-(d@normal)*normal))
                last[n] = pos
            rot = data.body('chassis').xmat.reshape(3, 3)
            peak['tilt'] = max(peak['tilt'], math.degrees(math.acos(np.clip(rot[:, 2]@normal, -1, 1))))
            peak['loop'] = max(peak['loop'], loop_violation(model, data))
            if step % 20 == 0:
                d = data.qpos[:3]-initial
                rows.append([float(data.time), float(d@tangent), float(d[1]),
                             math.degrees(math.atan2(rot[1, 0], rot[:, 0]@tangent)),
                             float(data.qvel[:3]@tangent)]+
                            [math.degrees(data.joint(f'{k}_joint').qpos[0]) for k in
                             ('column', 'knuckle_left', 'knuckle_right')]+
                            [float(data.qvel[j.dofadr[0]]) for j, _ in wheels.values()]+
                            [float(log.get(k, 0.0)) for k in ('engine_rpm', 'torque', 'brake', 'gear', 'steer_nm')])
            if handle:
                if not handle.is_running():
                    raise RuntimeError('Viewer closed before the test completed')
                handle.cam.lookat[:] = data.body('chassis').xpos+[.5, 0, .3]
                handle.sync()
                time.sleep(max(0, model.opt.timestep-(time.monotonic()-start)))
    finally:
        if handle:
            handle.close()
    rows = np.array(rows)
    rows[:, 3] = np.degrees(np.unwrap(np.radians(rows[:, 3])))
    rolled = {n: float(data.qpos[j.qposadr[0]]*r) for n, (j, r) in wheels.items()}
    error = {n: abs(abs(rolled[n])-path[n])/max(path[n], .001) for n in wheels}
    warnings = {mujoco.mjtWarning(i).name: int(w.number) for i, w in enumerate(data.warning) if w.number}
    summary = dict(timestep_s=model.opt.timestep, forward_m=float((data.qpos[:3]-initial)@tangent),
                   lateral_m=float(data.qpos[1]-initial[1]), yaw_deg=float(rows[-1, 3]),
                   kinematic_yaw_deg=math.degrees(kinematic_yaw), max_engine_rpm=peak['engine'],
                   max_engine_power_w=peak['power'],
                   final_speed_mps=float(data.qvel[:3]@tangent), max_tilt_deg=peak['tilt'],
                   max_loop_violation_m=peak['loop'], wheel_paths_m=path, wheel_rolled_m=rolled,
                   rolling_error=error, forbidden_contacts=sorted(bad), wheels_touching=sorted(touched),
                   solver_warnings=warnings)
    common = dict(no_interference=not bad, all_wheels_contact=set(wheels) <= touched,
                  rolls_not_slides=max(error.values()) < LIMITS['rolling_error'],
                  upright=peak['tilt'] < LIMITS['tilt_deg'], no_solver_warnings=not warnings,
                  linkage_closed=peak['loop'] < LIMITS['max_loop_violation_m'])
    return summary, common, rows


HEADER = ['time_s', 'forward_m', 'lateral_m', 'yaw_deg', 'speed_mps', 'column_deg', 'knuckle_left_deg',
          'knuckle_right_deg']


def write_csv(path, rows, wheels):
    with path.open('w') as f:
        w = csv.writer(f)
        w.writerow(HEADER+[f'{n}_radps' for n in wheels]+['engine_rpm', 'engine_torque_nm', 'brake', 'gear',
                                                            'steering_torque_nm'])
        w.writerows(rows.tolist())


def check_controls(geometry, physics):
    """The build checks must catch a loose join, a same-body intersection and a sweep intrusion."""
    results = {}
    model = vehicle.make_model(geometry, physics, path_name='controls.xml')

    def shifted(prefix, delta):
        ids = [g for g in range(model.ngeom) if model.geom(g).name.startswith(prefix+':')]
        saved = model.geom_pos[ids].copy()
        model.geom_pos[ids] += delta
        try:
            return vehicle.build_checks(model, geometry, step_deg=15)
        finally:
            model.geom_pos[ids] = saved
    loose = shifted('Front hanger 1', [0, 0, .1])   # lifted clear of the beam
    results['loose_join_detected'] = any('Front hanger 1' in k for k in map(tuple, loose['loose_joins']))
    clash = shifted('Drag arm', [0, 0, .034])
    results['same_body_intersection_detected'] = any(set(f['pair']) == {'Drag arm', 'Knuckle lower ear 1'}
                                                     for f in clash['failures'])
    lock = vehicle.kinematics(geometry, math.radians(geometry['bodies']['column']['joint']['range_deg'][1]))
    s = geometry['steering']
    K, A0 = np.array(s['kingpins']['left']), np.array(s['arm_balls']['left'])
    ball = K+vehicle._rot(A0-K, lock['knuckle_left'])
    z = geometry['bodies']['tie_rod']['origin'][2]
    blocked = vehicle.make_model(geometry, physics, extra=[('chassis', dict(
        name='sweep_blocker', type='box', size='.01 .01 .01', pos=vehicle.numbers([*ball, z])))],
        path_name='controls_blocked.xml')
    sweep = vehicle.build_checks(blocked, geometry, step_deg=1)
    hits = [f for f in sweep['failures'] if 'sweep_blocker' in f['pair']]
    results['sweep_intrusion_detected'] = bool(hits) and all(f['angle_deg'] != 0 for f in hits)
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', choices=('chassis', 'full'), required=True)
    parser.add_argument('--viewer', action='store_true')
    args = parser.parse_args()
    out, geometry = export(args.stage)
    physics = out/'physics'
    report = dict(stage=args.stage, source=geometry['source'], source_sha256=geometry['source_sha256'],
                  mujoco_version=mujoco.__version__, limits=LIMITS, vehicle=vehicle.CONFIG)

    model = vehicle.make_model(geometry, physics)
    report['build'] = vehicle.build_checks(model, geometry)
    report['build_controls'] = check_controls(geometry, physics)

    # Unpowered downhill gate: 5 degrees, 4 s, brakes released, column held straight.
    slope = math.radians(5)
    model = vehicle.make_model(geometry, physics, slope)
    wheels = [n for n, b in geometry['bodies'].items() if 'wheel' in b]
    gate, common, rows = simulate(model, geometry, 4, slope, viewer=args.viewer)
    write_csv(physics/'downhill.csv', rows, wheels)
    gate['checks'] = common|dict(downhill_progress=gate['forward_m'] > LIMITS['progress_m'],
                                 wheels_turn_forward=all(v > 1 for v in gate['wheel_rolled_m'].values()),
                                 straight=abs(gate['lateral_m']) < .05)
    blocked = vehicle.make_model(geometry, physics, slope, extra=[('chassis', dict(
        name='deliberate_wheel_blocker', type='capsule', size='.02',
        fromto=vehicle.numbers([geometry['parameters']['wheelbase']-.1, .5, .5,
                                geometry['parameters']['wheelbase']+.1, .5, .5])))], path_name='blocked.xml')
    negative, _, _ = simulate(blocked, geometry, .1, slope)
    gate['negative_control_detected'] = any('deliberate_wheel_blocker' in p for p in negative['forbidden_contacts'])
    gate['passed'] = all(gate['checks'].values()) and gate['negative_control_detected']
    report['downhill_gate'] = gate
    report['passed'] = report['build']['passed'] and all(report['build_controls'].values()) and gate['passed']

    if args.stage == 'full' and gate['passed']:
        import dynamics
        from control import MAX_TORQUE
        ratios = {g: v['overall_ratio'] for g, v in geometry['drive']['gears'].items()}

        def make(slope, engine, spool, timestep):
            model = vehicle.make_model(geometry, physics, slope, spool=spool, path_name='scenario.xml',
                                       engine=dict(torque_nm=MAX_TORQUE, ratios=ratios) if engine else None)
            if timestep:
                model.opt.timestep = timestep
            return model
        summary, scenarios, rows = dynamics.run_all(
            make, lambda m, seconds, slope, ctrl: simulate(m, geometry, seconds, slope, ctrl), geometry, ratios)
        (physics/'runs').mkdir(exist_ok=True)
        for name, data in rows.items():
            write_csv(physics/'runs'/f'{name}.csv', data, wheels)
        report['dynamics'] = summary|dict(scenarios=scenarios)
        report['ackermann'] = [vehicle.ackermann(geometry, math.radians(d)) | dict(column_deg=d)
                               for d in range(-30, 31, 5)]
        report['passed'] &= summary['passed']
    # Leave the deliverable XML level; powered (belts idle) on the full build.
    vehicle.make_model(geometry, physics, engine=dict(torque_nm=__import__('control').MAX_TORQUE, ratios={
        g: v['overall_ratio'] for g, v in geometry['drive']['gears'].items()}) if geometry.get('drive') else None)
    (physics/'report.json').write_text(json.dumps(report, indent=2, default=lambda o: o.item())+'\n')
    if 'dynamics' in report:
        print(json.dumps({k: dict(passed=v['passed'], failed=[c for c, ok in v.get('checks', {}).items() if not ok])
                          for k, v in report['dynamics']['scenarios'].items()}, indent=1, default=lambda o: o.item()))
    print(json.dumps(dict(passed=report['passed'],
                          build={k: v for k, v in report['build'].items() if k != 'closest'},
                          controls=report['build_controls'],
                          gate=dict(passed=gate['passed'], failed=[k for k, v in gate['checks'].items() if not v],
                                    forward_m=gate['forward_m'], lateral_m=gate['lateral_m'],
                                    rolling_error=gate['rolling_error'], contacts=gate['forbidden_contacts'],
                                    loop=gate['max_loop_violation_m'], negative=gate['negative_control_detected'])),
                     indent=2))
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
