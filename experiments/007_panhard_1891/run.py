"""Panhard et Levassor (1891) validation: build checks, downhill gate, scenarios.

.venv/bin/python experiments/007_panhard_1891/run.py --stage chassis   # gate before drivetrain/bodywork
.venv/bin/python experiments/007_panhard_1891/run.py --stage full
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


def export(stage):
    out = HERE/'output'/stage
    subprocess.run(['blender', '--background', str(out/'panhard_1891.blend'), '--python-exit-code', '1', '--python',
                    str(VELO/'export_blender.py'), '--', str(out/'physics')], check=True)
    return out, json.loads((out/'physics/geometry.json').read_text())


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
    loose = shifted('Front hanger 1', [0, 0, .1])
    out['loose_join_detected'] = any('Front hanger 1' in k for k in map(tuple, loose['loose_joins']))
    ear = model.geom_pos[ids('Knuckle lower ear 1')[0]][2]+model.body_pos[model.body('knuckle_left').id][2]
    arm = model.geom_pos[ids('Drag arm')[0]][2]+model.body_pos[model.body('knuckle_left').id][2]
    clash = shifted('Drag arm', [0, 0, ear-arm])
    out['same_body_intersection_detected'] = any(set(f['pair']) == {'Drag arm', 'Knuckle lower ear 1'}
                                                 for f in clash['failures'])
    lock = vehicle.kinematics(geometry, math.radians(geometry['bodies']['column']['joint']['range_deg'][1]))
    s = geometry['steering']
    k, a0 = np.array(s['kingpins']['left']), np.array(s['arm_balls']['left'])
    ball = k+vehicle._rot(a0-k, lock['knuckle_left'])
    blocked = vehicle.make_model(geometry, physics, extra=[('chassis', dict(
        name='sweep_blocker', type='box', size='.01 .01 .01',
        pos=vehicle.numbers([*ball, geometry['bodies']['tie_rod']['origin'][2]])))], path_name='controls_blocked.xml')
    hits = [f for f in vehicle.build_checks(blocked, geometry, step_deg=1)['failures'] if 'sweep_blocker' in f['pair']]
    out['sweep_intrusion_detected'] = bool(hits) and all(f['angle_deg'] != 0 for f in hits)
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', choices=('chassis', 'full'), required=True)
    parser.add_argument('--viewer', action='store_true')
    args = parser.parse_args()
    out, geometry = export(args.stage)
    physics = out/'physics'
    report = dict(stage=args.stage, source=geometry['source'], source_sha256=geometry['source_sha256'],
                  mujoco_version=mujoco.__version__, limits=velo.LIMITS, vehicle=vehicle.CONFIG)
    report['build'] = vehicle.build_checks(vehicle.make_model(geometry, physics), geometry)
    report['build_controls'] = check_controls(geometry, physics)

    slope = math.radians(5)
    wheels = [n for n, b in geometry['bodies'].items() if 'wheel' in b]
    gate, common, rows = velo.simulate(vehicle.make_model(geometry, physics, slope), geometry, 4, slope,
                                       viewer=args.viewer)
    velo.write_csv(physics/'downhill.csv', rows, wheels)
    gate['checks'] = common|dict(downhill_progress=gate['forward_m'] > velo.LIMITS['progress_m'],
                                 wheels_turn_forward=all(v > 1 for v in gate['wheel_rolled_m'].values()),
                                 straight=abs(gate['lateral_m']) < .05)
    wb, fr, ft = (geometry['parameters'][k] for k in ('wheelbase', 'front_radius', 'front_track'))
    blocked = vehicle.make_model(geometry, physics, slope, extra=[('chassis', dict(
        name='deliberate_wheel_blocker', type='capsule', size='.02',
        fromto=vehicle.numbers([wb-.1, ft/2, fr*1.5, wb+.1, ft/2, fr*1.5])))], path_name='blocked.xml')
    negative, _, _ = velo.simulate(blocked, geometry, .1, slope)
    gate['negative_control_detected'] = any('deliberate_wheel_blocker' in p for p in negative['forbidden_contacts'])
    gate['passed'] = all(gate['checks'].values()) and gate['negative_control_detected']
    report['downhill_gate'] = gate
    report['passed'] = report['build']['passed'] and all(report['build_controls'].values()) and gate['passed']

    ratios = None
    if args.stage == 'full':
        from control import ENGINE, MAX_TORQUE
        ratios = {g: n*geometry['drive']['chain_ratio'] for g, n in ENGINE['gearbox_ratios'].items()}
    if args.stage == 'full' and gate['passed']:
        import dynamics

        def make(slope, engine, spool, timestep):
            m = vehicle.make_model(geometry, physics, slope, spool=spool, path_name='scenario.xml',
                                   engine=dict(torque_nm=MAX_TORQUE, ratios=ratios) if engine else None)
            if timestep:
                m.opt.timestep = timestep
            return m
        summary, scenarios, runs = dynamics.run_all(
            make, lambda m, seconds, slope, ctrl: velo.simulate(m, geometry, seconds, slope, ctrl), geometry, ratios)
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
