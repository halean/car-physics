"""Steering range clearance, downhill left/right turns, and braking in a turn."""
import argparse
import copy
import csv
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import time
import mujoco
import numpy as np
from control import SteeringControl

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
spec=importlib.util.spec_from_file_location('downhill_physics',HERE.parent/'002_downhill/run.py')
physics=importlib.util.module_from_spec(spec)
spec.loader.exec_module(physics)


def forbidden_contacts(model,data):
    pairs=set()
    ground=set()
    for contact in data.contact:
        if contact.dist>0:
            continue
        a,b=[mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,i) for i in (contact.geom1,contact.geom2)]
        if 'slope' in (a,b):
            other=b if a=='slope' else a
            if other.endswith('_rolling'):
                ground.add(other.removesuffix('_rolling'))
                continue
        pairs.add(tuple(sorted((a,b))))
    return pairs,ground


def sweep(model,limit):
    data=mujoco.MjData(model)
    moving={model.body('steering').id,model.body('front').id}
    pairs=[]
    for a in range(model.ngeom):
        if model.geom_bodyid[a]==0 or not model.geom_contype[a]:
            continue
        for b in range(a):
            if model.geom_bodyid[b]==0 or model.geom_bodyid[a]==model.geom_bodyid[b]:
                continue
            if not {model.geom_bodyid[a],model.geom_bodyid[b]} & moving:
                continue
            if (model.geom_contype[a]&model.geom_conaffinity[b] or
                    model.geom_contype[b]&model.geom_conaffinity[a]):
                pairs.append((a,b))
    minimum=.05
    closest=None
    failures=[]
    q=model.joint('steering_yaw').qposadr[0]
    # Every .5 degree, with a conservative bound for motion between samples.
    angles=np.linspace(-limit,limit,int(math.ceil(2*limit/.5))+1)
    mujoco.mj_forward(model,data)
    pivot=data.body('steering').xpos
    radius=max(np.linalg.norm(data.geom_xpos[i]-pivot)+model.geom_rbound[i]
               for i in range(model.ngeom) if model.geom_bodyid[i] in moving and model.geom_contype[i])
    bound=radius*math.radians(float(angles[1]-angles[0]))/2
    for angle in angles:
        data.qpos[q]=math.radians(float(angle))
        mujoco.mj_forward(model,data)
        for a,b in pairs:
            gap=mujoco.mj_geomDistance(model,data,a,b,.05,None)
            if gap<minimum:
                minimum=float(gap)
                closest=[model.geom(a).name,model.geom(b).name,float(angle)]
            if gap-bound<.005:
                failures.append([model.geom(a).name,model.geom(b).name,float(angle),float(gap)])
    return dict(passed=not failures,samples=len(angles),range_deg=[-limit,limit],
        minimum_sample_gap_m=minimum,between_samples_bound_m=bound,
        conservative_gap_m=minimum-bound,closest_pair_and_angle=closest,
        failures=failures[:20])


def drive(model,geometry,direction,viewer=False,seconds=6,timestep=None):
    if timestep:
        model.opt.timestep=timestep
    data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    tangent=np.array([math.cos(math.radians(5)),0,-math.sin(math.radians(5))])
    normal=np.array([math.sin(math.radians(5)),0,math.cos(math.radians(5))])
    initial=data.qpos[:3].copy()
    wheels={name:(model.joint(name+'_axle').qposadr[0],r) for name,_,r in geometry['wheels']}
    last={name:data.body(name).xpos.copy() for name in wheels}
    lengths={name:0.0 for name in wheels}
    touched=set()
    bad=set()
    max_tilt=0
    max_angle=0
    kinematic_yaw=0.0
    tracking_error=0.0
    brake_onset=None
    rows=[]
    control=SteeringControl(direction)
    handle=None
    if viewer:
        from mujoco import viewer as mjviewer
        handle=mjviewer.launch_passive(model,data)
        handle.opt.geomgroup[3]=0
        handle.cam.distance=5
        handle.cam.elevation=-30
    try:
        for step in range(round(seconds/model.opt.timestep)):
            start=time.monotonic()
            brake=control(model,data)
            if brake and brake_onset is None:
                brake_onset=float((data.qpos[:3]-initial)@tangent)
            # Rear-axle tricycle kinematics: yaw rate = v tan(steer) / wheelbase.
            v=data.qvel[:3]
            kinematic_yaw+=(float(np.linalg.norm(v-(v@normal)*normal))*
                math.tan(data.joint('steering_yaw').qpos[0])/geometry['parameters']['wheelbase']*model.opt.timestep)
            mujoco.mj_step(model,data)
            mujoco.mj_forward(model,data)
            if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
                raise RuntimeError('Non-finite steering simulation')
            pairs,ground=forbidden_contacts(model,data)
            bad.update(pairs)
            touched.update(ground)
            for name in wheels:
                pos=data.body(name).xpos.copy()
                delta=pos-last[name]
                lengths[name]+=float(np.linalg.norm(delta-(delta@normal)*normal))
                last[name]=pos
            rotation=data.body('chassis').xmat.reshape(3,3)
            tilt=math.degrees(math.acos(np.clip(rotation[:,2]@normal,-1,1)))
            max_tilt=max(max_tilt,tilt)
            yaw=math.degrees(math.atan2(rotation[1,0],rotation[:,0]@tangent))
            angle=math.degrees(data.joint('steering_yaw').qpos[0])
            max_angle=max(max_angle,abs(angle))
            v=data.qvel[:3]
            speed=float(np.linalg.norm(v-(v@normal)*normal))
            if data.time>1.5 and speed>.1:
                tracking_error=max(tracking_error,abs(angle-direction*15))
            if step%20==0:
                delta=data.qpos[:3]-initial
                rows.append([float(data.time),float(delta@tangent),float(delta[1]),yaw,angle,speed,brake])
            if handle:
                if not handle.is_running():
                    raise RuntimeError('Viewer closed before the test completed')
                handle.cam.lookat[:]=data.body('chassis').xpos+[.3,0,.5]
                handle.sync()
                time.sleep(max(0,model.opt.timestep-(time.monotonic()-start)))
    finally:
        if handle:
            handle.close()
    rolling={name:float(data.qpos[q]*r) for name,(q,r) in wheels.items()}
    error={name:abs(rolling[name]-lengths[name])/max(lengths[name],.001) for name in wheels}
    warnings={mujoco.mjtWarning(i).name:int(w.number) for i,w in enumerate(data.warning) if w.number}
    end=np.array(rows)
    hold=end[end[:,0]>seconds-1]
    lateral=float(data.qpos[1]-initial[1])
    predicted=math.degrees(kinematic_yaw)
    downhill=float((data.qpos[:3]-initial)@tangent)
    checks=dict(turns_correct_way=direction*lateral>.25 and direction*yaw>5,
        follows_tiller=abs(angle-direction*15)<2,within_limits=max_angle<=geometry['steering_limit_deg']+.5,
        downhill_progress=downhill>1,
        kinematic_yaw=abs(yaw-predicted)<max(1,.05*abs(predicted)),
        rolls_not_slides=max(error.values())<.10,no_interference=not bad,
        all_wheels_contact=set(wheels)<=touched,upright=max_tilt<10,no_solver_warnings=not warnings,
        stops_and_holds=float(np.max(hold[:,5]))<.02,
        hold_drift=float(np.linalg.norm(np.ptp(hold[:,1:3],axis=0)))<.01)
    return dict(passed=all(checks.values()),checks=checks,timestep_s=model.opt.timestep,
        downhill_m=downhill,lateral_m=lateral,yaw_deg=yaw,kinematic_yaw_deg=predicted,
        stopping_distance_m=downhill-brake_onset if brake_onset is not None else None,
        max_tiller_tracking_error_while_moving_deg=tracking_error,
        final_speed_mps=speed,max_tilt_deg=max_tilt,max_steering_deg=max_angle,
        wheel_path_lengths_m=lengths,wheel_rolling_distance_m=rolling,rolling_error=error,
        forbidden_contacts=sorted(bad),solver_warnings=warnings),rows


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--viewer',action='store_true')
    args=parser.parse_args()
    out=HERE/'output'
    source=ROOT/'experiments/001_patent_motorwagen/output/patent_motorwagen.blend'
    subprocess.run(['blender','--background',str(source),'--python-exit-code','1',
        '--python',str(HERE.parent/'002_downhill/export_blender.py'),'--',str(out)],check=True)
    geometry=json.loads((out/'geometry.json').read_text())
    if not geometry.get('steering_pivot'):
        raise RuntimeError('Rebuild the car with its steering pivot first')
    model=physics.make_model(geometry,out,math.radians(5))
    clearance=sweep(model,geometry['steering_limit_deg'])
    runs={}
    for direction,name in ((1,'left'),(-1,'right')):
        model=physics.make_model(geometry,out,math.radians(5))
        runs[name],rows=drive(model,geometry,direction,args.viewer)
        with (out/f'{name}.csv').open('w') as f:
            writer=csv.writer(f)
            writer.writerow(['time_s','downhill_m','lateral_m','yaw_deg','steering_deg','speed_mps','brake'])
            writer.writerows(rows)
    # Half-timestep sensitivity of the steered, braked left run.
    finer,_=drive(physics.make_model(geometry,out,math.radians(5)),geometry,1,timestep=.0005)
    left=runs['left']
    sensitivity=dict(yaw_change_deg=abs(finer['yaw_deg']-left['yaw_deg']),
        lateral_change_m=abs(finer['lateral_m']-left['lateral_m']),
        stopping_distance_change_m=abs(finer['stopping_distance_m']-left['stopping_distance_m']))
    sensitivity['passed']=(finer['passed'] and sensitivity['yaw_change_deg']<1 and
        sensitivity['lateral_change_m']<.03 and sensitivity['stopping_distance_change_m']<.03)
    sensitivity['half_timestep_run']=finer
    negative_geometry=copy.deepcopy(geometry)
    negative_geometry['colliders'].append(dict(name='steering_sweep_blocker',type='box',
        pos=[geometry['parameters']['wheelbase']+.29,.14,geometry['parameters']['front_radius']],
        size=[.02,.02,.02],role='chassis',axle=False))
    negative=physics.make_model(negative_geometry,out,math.radians(5))
    negative_sweep=sweep(negative,geometry['steering_limit_deg'])
    detected=any('steering_sweep_blocker' in pair[:2] for pair in negative_sweep['failures'])
    # Restore the exported deliverable to the valid assembly.
    physics.make_model(geometry,out,math.radians(5))
    report=dict(passed=clearance['passed'] and all(r['passed'] for r in runs.values())
        and sensitivity['passed'] and detected,
        source=str(source),source_sha256=geometry['source_sha256'],clearance=clearance,runs=runs,
        half_timestep=sensitivity,sweep_obstruction_detected=detected,mujoco_version=mujoco.__version__,
        conditions=dict(slope_deg=5,seconds=6,timestep_s=.001,tiller_command_deg=15,
            tiller_ramp_s=[.75,1.5],rear_brake_apply_s=3.0,rear_torque_limit_nm_each=75,
            tiller_actuator='position servo kp=180 kv=12, force limit 20 N m'),
        limitations=['Steering column and fork are one rigid body on a yaw hinge; no caster, trail, '
            'kingpin inclination or tire slip-angle model','Rigid chassis and suspension-free wheels',
            'Primitive colliders and conservative wheel envelopes; mass and friction are assumptions',
            'Clearance sweep is kinematic at rest pose (no suspension travel); structural strength not assessed'])
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    if not report['passed']:
        raise SystemExit(1)


if __name__=='__main__':
    main()
