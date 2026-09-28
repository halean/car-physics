"""Engine drive: drivetrain clearance, flat launch, climbing, grade limit,
powered turns through the differential, and braking against the engine."""
import argparse
import csv
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
import mujoco
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.path.insert(0,str(HERE))
from control import ENGINE, GOVERNED, NO_LOAD, MAX_TORQUE, EngineControl, Tiller, engine_torque
from visual import add_force_arrows


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


physics=load('downhill_physics',HERE.parent/'002_downhill/run.py')
BrakeControl=load('rear_brake_control',HERE.parent/'003_brakes/control.py').BrakeControl

# Acceptance thresholds, fixed before the first powered run.
LIMITS=dict(model_speed_rel=.03,model_distance_rel=.03,top_speed_fraction=.95,
    engine_overspeed=1.01,straight_lateral_m=.05,straight_yaw_deg=1.0,wheel_speed_match=.01,
    rolling_error=.10,differential_ratio_rel=.03,rollback_m=.05,climb_m=5.0,
    grade_limit_rollback_m=.5,hold_speed_mps=.02,hold_drift_m=.01,half_timestep_rel=.01,
    sweep_gap_m=.005,tilt_deg=10.0)


def contacts(model,data):
    bad,ground=set(),set()
    for contact in data.contact:
        if contact.dist>0:
            continue
        a,b=[mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,i) for i in (contact.geom1,contact.geom2)]
        if 'slope' in (a,b):
            other=b if a=='slope' else a
            if other.endswith('_rolling'):
                ground.add(other.removesuffix('_rolling'))
                continue
        bad.add(tuple(sorted((a,b))))
    return bad,ground


def simulate(model,geometry,control,seconds,angle,timestep=None,viewer=False):
    if timestep:
        model.opt.timestep=timestep
    data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    tangent=np.array([math.cos(angle),0,-math.sin(angle)])
    normal=np.array([math.sin(angle),0,math.cos(angle)])
    initial=data.qpos[:3].copy()
    wheels={name:(model.joint(name+'_axle'),r) for name,_,r in geometry['wheels']}
    last={name:data.body(name).xpos.copy() for name in wheels}
    lengths={name:0.0 for name in wheels}
    bad,touched=set(),set()
    rows=[]
    peak=dict(tilt=0.0,engine_speed=0.0,power=0.0)
    kinematic_yaw=0.0
    wheelbase=geometry['parameters']['wheelbase']
    handle=None
    if viewer:
        from mujoco import viewer as mjviewer
        handle=mjviewer.launch_passive(model,data)
        handle.opt.geomgroup[3]=0
        handle.cam.distance=5
        handle.cam.elevation=-25
    try:
        for step in range(round(seconds/model.opt.timestep)):
            start=time.monotonic()
            log=control(model,data)
            v=data.qvel[:3]
            planar=float(np.linalg.norm(v-(v@normal)*normal))
            kinematic_yaw+=planar*math.tan(data.joint('steering_yaw').qpos[0])/wheelbase*model.opt.timestep
            mujoco.mj_step(model,data)
            mujoco.mj_forward(model,data)
            if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
                raise RuntimeError('Non-finite simulation')
            pairs,ground=contacts(model,data)
            bad.update(pairs)
            touched.update(ground)
            for name in wheels:
                pos=data.body(name).xpos.copy()
                delta=pos-last[name]
                lengths[name]+=float(np.linalg.norm(delta-(delta@normal)*normal))
                last[name]=pos
            rotation=data.body('chassis').xmat.reshape(3,3)
            peak['tilt']=max(peak['tilt'],math.degrees(math.acos(np.clip(rotation[:,2]@normal,-1,1))))
            peak['engine_speed']=max(peak['engine_speed'],log['engine_speed'])
            peak['power']=max(peak['power'],log['torque']*log['engine_speed'])
            if step%20==0:
                delta=data.qpos[:3]-initial
                yaw=math.degrees(math.atan2(rotation[1,0],rotation[:,0]@tangent))
                rows.append([float(data.time),float(delta@tangent),float(delta[1]),yaw,
                    math.degrees(data.joint('steering_yaw').qpos[0]),float(data.qvel[:3]@tangent),
                    log['engine_speed']*60/math.tau,log['torque'],log['engaged'],log['brake']]+
                    [float(data.qvel[j.dofadr[0]]) for j,_ in wheels.values()])
            if handle:
                if not handle.is_running():
                    raise RuntimeError('Viewer closed before the test completed')
                handle.cam.lookat[:]=data.body('chassis').xpos+[.3,0,.4]
                with handle.lock():
                    handle.user_scn.ngeom=0
                    add_force_arrows(model,data,handle.user_scn)
                handle.sync()
                time.sleep(max(0,model.opt.timestep-(time.monotonic()-start)))
    finally:
        if handle:
            handle.close()
    rolled={name:abs(float(data.qpos[j.qposadr[0]]*r)) for name,(j,r) in wheels.items()}
    error={name:abs(rolled[name]-lengths[name])/max(lengths[name],.001) for name in wheels}
    warnings={mujoco.mjtWarning(i).name:int(w.number) for i,w in enumerate(data.warning) if w.number}
    rows=np.array(rows)
    # Powered turns can exceed 180 degrees; report continuous heading.
    rows[:,3]=np.degrees(np.unwrap(np.radians(rows[:,3])))
    summary=dict(timestep_s=model.opt.timestep,forward_m=float((data.qpos[:3]-initial)@tangent),
        lateral_m=float(data.qpos[1]-initial[1]),yaw_deg=float(rows[-1,3]),kinematic_yaw_deg=math.degrees(kinematic_yaw),
        final_speed_mps=float(data.qvel[:3]@tangent),max_tilt_deg=peak['tilt'],
        max_engine_rpm=peak['engine_speed']*60/math.tau,max_engine_power_w=peak['power'],
        wheel_path_lengths_m=lengths,wheel_rolled_m=rolled,rolling_error=error,
        forbidden_contacts=sorted(bad),wheels_touching=sorted(touched),solver_warnings=warnings)
    common=dict(no_interference=not bad,all_wheels_contact=set(wheels)<=touched,
        upright=peak['tilt']<LIMITS['tilt_deg'],no_solver_warnings=not warnings,
        rolls_not_slides=max(error.values())<LIMITS['rolling_error'],
        engine_within_governor=peak['engine_speed']<=NO_LOAD*LIMITS['engine_overspeed'],
        power_within_rating=peak['power']<=ENGINE['rated_power_w']*1.001)
    return summary,common,rows


def reference(model,geometry,control,seconds,angle,dt=.001):
    """One-dimensional longitudinal model with the same masses, engine, belt and brake."""
    r=geometry['parameters']['rear_radius']
    ratio=geometry['drive']['overall_ratio']
    mass=float(model.body_mass[1:].sum())
    wheels=[(model.body(name).id,radius) for name,_,radius in geometry['wheels']]
    inertia=sum(model.body_inertia[i][1]/radius**2 for i,radius in wheels)
    brake=control.brake
    v=x=0.0
    out=[]
    for step in range(round(seconds/dt)):
        t=step*dt
        engaged=0.0
        if control.engage_at is not None and t>=control.engage_at:
            engaged=min(1.0,(t-control.engage_at)/ENGINE['belt_shift_seconds'])
        if control.disengage_at is not None and t>=control.disengage_at:
            engaged=0.0
        force=engaged*ratio*engine_torque(ratio*v/r)/r+mass*9.81*math.sin(angle)
        capacity=0.0
        if brake:
            command=max(0.0,min(1.0,(t-brake.apply_at)/brake.ramp_seconds))
            if brake.release_at is not None and t>=brake.release_at:
                command=0.0
            capacity=2*brake.torque_nm*command/r
        if abs(v)<1e-9 and abs(force)<=capacity:
            v=0.0
        else:
            direction=math.copysign(1,v) if abs(v)>=1e-9 else math.copysign(1,force)
            new=v+(force-capacity*direction)/(mass+inertia)*dt
            v=0.0 if capacity and new*v<0 else new
        x+=v*dt
        out.append((t+dt,x,v))
    return np.array(out)


def at(rows,t,column):
    return float(np.interp(t,rows[:,0],rows[:,column]))


def rel(a,b):
    return abs(a-b)/max(abs(b),1e-6)


def drive_sweep(geometry,out,torque,blocker=False):
    """Swept rotating drive parts vs every other collider, including same-body parts."""
    physics.make_model(geometry,out,0,engine_torque_nm=torque)
    tree=ET.parse(out/'downhill.xml')
    chassis=tree.getroot().find(".//body[@name='chassis']")
    names=[]
    for part in geometry['drive']['sweeps']:
        for i,shape in enumerate(part['shapes']):
            name=f'sweep|{part["name"]}|{i}'
            ET.SubElement(chassis,'geom',name=name,type=shape['type'],fromto=physics.numbers(shape['fromto']),
                size=physics.numbers(shape['size']),contype='0',conaffinity='0',group='4')
            names.append((name,part))
    if blocker:
        belt=[s for p in geometry['drive']['sweeps'] if p['name']=='Drive belt' for s in p['shapes']]
        low=min((s['fromto'][:3] for s in belt),key=lambda p:p[2])
        ET.SubElement(chassis,'geom',name='drive_sweep_blocker',type='box',pos=physics.numbers(low),
            size='.01 .01 .01',contype='2',conaffinity='5')
    path=out/('drive_sweep_blocked.xml' if blocker else 'drive_sweep.xml')
    tree.write(path,encoding='unicode')
    model=mujoco.MjModel.from_xml_path(str(path))
    data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    own={'Horizontal flywheel':('Flywheel rim',)}
    minimum,closest,failures,pairs=math.inf,None,[],0
    for name,part in names:
        a=model.geom(name).id
        skip=(part['name'],)+own.get(part['name'],())+tuple(part['mates'])
        for b in range(model.ngeom):
            other=model.geom(b).name
            if model.geom_bodyid[b]==0 or not model.geom_contype[b] or other.split(':')[0] in skip:
                continue
            pairs+=1
            gap=mujoco.mj_geomDistance(model,data,a,b,.05,None)
            if gap<minimum:
                minimum,closest=float(gap),[part['name'],other]
            if gap<LIMITS['sweep_gap_m']:
                failures.append([part['name'],other,float(gap)])
    return dict(passed=not failures,pairs_checked=pairs,minimum_gap_m=minimum,closest_pair=closest,
                failures=failures[:20])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--viewer',action='store_true',help='watch the flat launch')
    args=parser.parse_args()
    out=HERE/'output'
    source=ROOT/'experiments/001_patent_motorwagen/output/patent_motorwagen.blend'
    subprocess.run(['blender','--background',str(source),'--python-exit-code','1',
        '--python',str(HERE.parent/'002_downhill/export_blender.py'),'--',str(out)],check=True)
    geometry=json.loads((out/'geometry.json').read_text())
    if not geometry.get('drive'):
        raise RuntimeError('Rebuild experiment 001 first: the drivetrain is missing')
    ratio=geometry['drive']['overall_ratio']
    r=geometry['parameters']['rear_radius']
    governed_speed=GOVERNED/ratio*r
    make=lambda angle,spool=False: physics.make_model(geometry,out,angle,engine_torque_nm=MAX_TORQUE,spool=spool)

    sweep=drive_sweep(geometry,out,MAX_TORQUE)
    blocked=drive_sweep(geometry,out,MAX_TORQUE,blocker=True)
    blocker_detected=any(f[1]=='drive_sweep_blocker' for f in blocked['failures'])

    results,trajectories={},{}
    def run(name,angle,control,seconds,spool=False,timestep=None,viewer=False):
        model=make(angle,spool)
        summary,common,rows=simulate(model,geometry,control,seconds,angle,timestep,viewer)
        ref=reference(model,geometry,control,seconds,angle)
        trajectories[name]=rows
        return model,summary,common,rows,ref

    # 1. Flat launch, belt engaged at 0.5 s, disengaged at 20 s.
    c=EngineControl(engage_at=.5,disengage_at=20)
    model,s,common,rows,ref=run('flat_launch',0,c,24,viewer=args.viewer)
    mass=float(model.body_mass[1:].sum())
    after=rows[rows[:,0]>20.05]
    speed_20=at(rows,20,5)
    rear=rows[(rows[:,0]>15)&(rows[:,0]<20),10:12]
    s.update(speed_at_5s_mps=at(rows,5,5),reference_speed_at_5s_mps=at(ref,5,2),
        distance_at_20s_m=at(rows,20,1),reference_distance_at_20s_m=at(ref,20,1),speed_at_20s_mps=speed_20,
        governed_speed_mps=governed_speed,rear_wheel_speed_mismatch=float(np.max(np.abs(rear[:,0]-rear[:,1])/np.abs(rear).mean(axis=1))))
    s['checks']=common|dict(
        matches_reference_speed=rel(s['speed_at_5s_mps'],s['reference_speed_at_5s_mps'])<LIMITS['model_speed_rel'],
        matches_reference_distance=rel(s['distance_at_20s_m'],s['reference_distance_at_20s_m'])<LIMITS['model_distance_rel'],
        reaches_governed_speed=speed_20>=LIMITS['top_speed_fraction']*governed_speed,
        clutch_cuts_drive=bool(np.all(after[:,7]==0)) and float(after[:,5].max())<=speed_20+1e-3,
        straight=abs(s['lateral_m'])<LIMITS['straight_lateral_m'] and abs(s['yaw_deg'])<LIMITS['straight_yaw_deg'],
        equal_rear_wheel_speeds=s['rear_wheel_speed_mismatch']<LIMITS['wheel_speed_match'])
    results['flat_launch']=s

    # 2. Hill start on a 3-degree climb: brakes held, belt engaged, brakes released.
    def hill(name,degrees,seconds):
        c=EngineControl(engage_at=.5,brake=BrakeControl(apply_at=0,release_at=1.5))
        _,s,common,rows,ref=run(name,-math.radians(degrees),c,seconds)
        after=rows[rows[:,0]>=1.5]
        s.update(slope_deg_uphill=degrees,rollback_after_release_m=float(at(rows,1.5,1)-after[:,1].min()),
            reference_forward_m=float(ref[-1,1]),final_speed_mps=float(rows[-1,5]))
        return s,common
    s,common=hill('climb_3deg',3,20)
    s['checks']=common|dict(no_rollback=s['rollback_after_release_m']<LIMITS['rollback_m'],
        climbs=s['forward_m']>LIMITS['climb_m'],
        matches_reference_distance=rel(s['forward_m'],s['reference_forward_m'])<LIMITS['model_distance_rel'])
    results['climb_3deg']=s

    # 3. Grade limit (negative control for a velocity-forced drive): 7 degrees must roll back.
    predicted=math.degrees(math.asin(min(1,ratio*MAX_TORQUE/r/(mass*9.81))))
    s,common=hill('grade_7deg',7,6)
    s['predicted_max_grade_deg']=predicted
    s['checks']=common|dict(rolls_back_under_full_torque=s['forward_m']<-LIMITS['grade_limit_rollback_m'],
        engine_at_full_torque=bool(np.all(trajectories['grade_7deg'][trajectories['grade_7deg'][:,0]>1.0,7]>=MAX_TORQUE*.999)),
        matches_reference_distance=rel(s['forward_m'],s['reference_forward_m'])<LIMITS['model_distance_rel'],
        prediction_brackets_tests=3<predicted<7)
    # Rolling back is the expected outcome here; the rolled/rolling-sign check is direction-free.
    results['grade_7deg']=s

    # 4. Powered turns through the open differential, and a locked-spool control.
    track=geometry['parameters']['rear_track']
    def turn(name,direction,spool=False):
        c=EngineControl(engage_at=.5,steering=Tiller(direction))
        _,s,common,rows,_=run(name,0,c,12,spool=spool)
        window=rows[rows[:,0]>6]
        delta=np.radians(np.abs(window[:,4])).mean()
        radius=geometry['parameters']['wheelbase']/math.tan(delta)
        expected=(radius+track/2)/(radius-track/2)
        outer,inner=(window[:,11],window[:,10]) if direction>0 else (window[:,10],window[:,11])
        s.update(mean_steer_deg=math.degrees(delta),turn_radius_m=radius,
            expected_outer_inner_ratio=expected,measured_outer_inner_ratio=float(outer.mean()/inner.mean()))
        s['checks']=common|dict(
            turns_correct_way=float((direction*rows[:,2]).max())>.25 and direction*s['yaw_deg']>5,
            differential_ratio=rel(s['measured_outer_inner_ratio'],expected)<LIMITS['differential_ratio_rel'],
            kinematic_yaw=abs(s['yaw_deg']-s['kinematic_yaw_deg'])<max(1,.05*abs(s['kinematic_yaw_deg'])))
        return s
    results['turn_left']=turn('turn_left',1)
    results['turn_right']=turn('turn_right',-1)
    spool=turn('turn_left_spool',1,spool=True)
    spool_detected=not spool['checks']['differential_ratio']

    # 5. Brakes against the engaged engine on the flat.
    c=EngineControl(engage_at=.5,brake=BrakeControl(apply_at=8))
    _,s,common,rows,_=run('brake_vs_engine',0,c,15)
    hold=rows[rows[:,0]>14]
    s.update(hold_max_speed_mps=float(np.abs(hold[:,5]).max()),hold_drift_m=float(np.ptp(hold[:,1])),
        engine_torque_during_hold_nm=float(hold[:,7].min()))
    s['checks']=common|dict(stops=s['hold_max_speed_mps']<LIMITS['hold_speed_mps'],
        holds=s['hold_drift_m']<LIMITS['hold_drift_m'],engine_still_driving=s['engine_torque_during_hold_nm']>=MAX_TORQUE*.999)
    results['brake_vs_engine']=s

    # 6. Flat launch at half timestep.
    _,fine,_,fine_rows,_=run('flat_launch_half_dt',0,EngineControl(engage_at=.5,disengage_at=20),24,timestep=.0005)
    half=dict(speed_at_10s_change=rel(at(fine_rows,10,5),at(trajectories['flat_launch'],10,5)),
        distance_at_20s_change=rel(at(fine_rows,20,1),at(trajectories['flat_launch'],20,1)),
        lateral_m=fine['lateral_m'],forbidden_contacts=fine['forbidden_contacts'])
    half['passed']=(half['speed_at_10s_change']<LIMITS['half_timestep_rel'] and
        half['distance_at_20s_change']<LIMITS['half_timestep_rel'] and not fine['forbidden_contacts'])

    for name,rows in trajectories.items():
        with (out/f'{name}.csv').open('w') as f:
            writer=csv.writer(f)
            writer.writerow(['time_s','forward_m','lateral_m','yaw_deg','steering_deg','speed_mps','engine_rpm',
                'engine_torque_nm','belt_engaged','brake','rear_left_radps','rear_right_radps','front_radps'])
            writer.writerows(rows.tolist())
    # Planes collide as infinite; the drawn size is visual only.
    # Leave the deliverable XML as the powered, open-differential model.
    make(0)
    for r_ in results.values():
        r_['passed']=all(r_['checks'].values())
    report=dict(passed=all(r_['passed'] for r_ in results.values()) and sweep['passed'] and blocker_detected
                and spool_detected and half['passed'],
        source=str(source),source_sha256=geometry['source_sha256'],mujoco_version=mujoco.__version__,
        drive_sweep=sweep,drive_sweep_blocker_detected=blocker_detected,runs=results,
        locked_differential_control=dict(detected=spool_detected,measured_outer_inner_ratio=spool['measured_outer_inner_ratio'],
            expected_outer_inner_ratio=spool['expected_outer_inner_ratio'],rolling_error=spool['rolling_error'],
            forbidden_contacts=spool['forbidden_contacts']),
        half_timestep=half,limits=LIMITS,
        drive=dict(overall_ratio=ratio,belt_ratio=geometry['drive']['belt_ratio'],chain_ratio=geometry['drive']['chain_ratio'],
            engine_torque_nm=MAX_TORQUE,governed_speed_mps=governed_speed,governed_speed_kmh=governed_speed*3.6,
            predicted_max_grade_deg=predicted,mass_kg=mass),
        engine=ENGINE,
        limitations=['Drivetrain is massless: flywheel, belt, countershaft and chain inertia and friction are not modeled',
            'Belt slip is idealized as passing rated torque below governed speed; no engine braking',
            'No rolling resistance or aerodynamic drag; top speed is set by the governor',
            '135 kg total mass assumption; the historical car is reported at 270 kg without occupants',
            'Chain and belt are rigid visual loops on the chassis; their running motion is not simulated'])
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(passed=report['passed'],sweep={k:v for k,v in sweep.items() if k!='failures'},
        blocker=blocker_detected,spool=spool_detected,half=half,
        runs={k:dict(passed=v['passed'],failed=[c for c,ok in v['checks'].items() if not ok]) for k,v in results.items()}),indent=2))
    if not report['passed']:
        raise SystemExit(1)


if __name__=='__main__':
    main()
