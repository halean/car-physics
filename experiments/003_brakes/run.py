"""Compare coasting, braking/holding, and brake release on a five-degree slope."""
import argparse
import csv
import json
import math
from pathlib import Path
import subprocess
import sys
import numpy as np
import mujoco

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.path.insert(0,str(HERE.parent/'002_downhill'))
from run import make_model, simulate
from control import BrakeControl


def stopping_metrics(rows,apply_at=2.0):
    a=np.asarray(rows)
    onset=int(np.searchsorted(a[:,0],apply_at))
    stops=np.flatnonzero((a[:,0]>apply_at+.25)&(np.abs(a[:,5])<.02))
    if not len(stops):
        return dict(stopped=False)
    stop=int(stops[0])
    hold=a[a[:,0]>=a[stop,0]+.25]
    return dict(stopped=True,speed_at_application_mps=float(a[onset,5]),
        stopping_distance_m=float(a[stop,1]-a[onset,1]),
        stopping_time_s=float(a[stop,0]-apply_at),
        hold_duration_s=float(a[-1,0]-a[stop,0]-.25),
        hold_max_speed_mps=float(np.abs(hold[:,5]).max()) if len(hold) else None,
        hold_drift_m=float(np.ptp(hold[:,1])) if len(hold) else None)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--viewer',action='store_true')
    parser.add_argument('--blender',default='blender')
    args=parser.parse_args()
    out=HERE/'output'
    blend=ROOT/'experiments/001_patent_motorwagen/output/patent_motorwagen.blend'
    subprocess.run([args.blender,'--background',str(blend),'--python-exit-code','1',
        '--python',str(HERE.parent/'002_downhill/export_blender.py'),'--',str(out)],check=True)
    geometry=json.loads((out/'geometry.json').read_text())
    if not all(geometry.get('wheel_colliders',{}).get(name) for name in ('rear_left','rear_right')):
        raise RuntimeError('Rebuild experiment 001 first: rear brake drums are missing')
    angle=math.radians(5)
    reports={}
    trajectories={}
    for name,seconds,control in [('coast',6,None),('brake',6,BrakeControl()),
                                 ('release',7,BrakeControl(release_at=5.0))]:
        model=make_model(geometry,out,angle)
        reports[name],trajectories[name]=simulate(model,geometry,angle,seconds,
            viewer=args.viewer and name=='brake',control=control)
        with (out/f'{name}.csv').open('w') as file:
            writer=csv.writer(file)
            writer.writerow(['time_s','downhill_m','rear_left_rad','rear_right_rad','front_rad',
                             'speed_mps','brake_command'])
            writer.writerows(trajectories[name])
    stop=stopping_metrics(trajectories['brake'])
    coast_stop=stopping_metrics(trajectories['coast'])
    release=np.asarray(trajectories['release'])
    before_release=release[(release[:,0]>4.0)&(release[:,0]<5.0)]
    finer_model=make_model(geometry,out,angle)
    finer_model.opt.timestep=.0005
    finer_report,finer_rows=simulate(finer_model,geometry,angle,6,control=BrakeControl())
    finer_stop=stopping_metrics(finer_rows)
    checks=dict(timestep_check=finer_report['passed'] and finer_stop['stopped']
        and abs(finer_stop['stopping_distance_m']-stop['stopping_distance_m'])<.03
        and finer_stop['hold_drift_m']<.01,all_physics_runs_pass=all(r['passed'] for r in reports.values()),
        stops=stop['stopped'],
        holds=stop.get('hold_duration_s',0)>1 and stop.get('hold_max_speed_mps',1)<.02
              and stop.get('hold_drift_m',1)<.01,
        travels_less_than_coast=reports['brake']['travel_m']<reports['coast']['travel_m']*.5,
        holds_before_release=float(np.abs(before_release[:,5]).max())<.02,
        rolls_after_release=reports['release']['final_speed_mps']>1,
        no_brake_negative_control=not coast_stop['stopped'])
    report=dict(passed=all(checks.values()),checks=checks,stopping=stop,
        half_timestep_stopping=finer_stop,runs=reports,
        mujoco_version=mujoco.__version__,slope_deg=5,apply_at_s=2,ramp_seconds=.25,
        rear_torque_limit_nm_each=75,release_test_at_s=5,
        model='Equivalent joint dry friction; no explicit shoe contact, cable actuation or thermal model',
        mass_assumption='135 kg total, including brakes; unchanged for controlled comparison')
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    if not report['passed']:
        raise SystemExit(1)


if __name__=='__main__':
    main()
