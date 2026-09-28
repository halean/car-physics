"""Headless downhill integration test; optional live MuJoCo viewer."""
import argparse
import csv
import json
import math
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET
import mujoco
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]


def numbers(values):
    return ' '.join(str(v) for v in values)


def make_model(geometry,out,angle,blocked=False,engine_torque_nm=0.0,spool=False):
    """engine_torque_nm bounds the drive actuator; the default 0 is belt disengaged."""
    root=ET.Element('mujoco',model='Motorwagen downhill')
    ET.SubElement(root,'compiler',angle='radian',inertiafromgeom='false')
    option=ET.SubElement(root,'option',timestep='.001',gravity='0 0 -9.81',integrator='implicitfast')
    ET.SubElement(option,'flag',filterparent='disable')
    defaults=ET.SubElement(root,'default')
    ET.SubElement(defaults,'geom',friction='.8 .005 .0001',condim='3',solref='.005 1')
    asset=ET.SubElement(root,'asset')
    for name in (['chassis','rear_left','rear_right','front']+(['steering'] if geometry.get('steering_pivot') else [])):
        ET.SubElement(asset,'mesh',name=name,file=str(out/f'{name}.obj'))
    ET.SubElement(asset,'texture',name='road_grid',type='2d',builtin='checker',
        rgb1='.25 .3 .34',rgb2='.45 .5 .54',width='256',height='256')
    ET.SubElement(asset,'material',name='road',texture='road_grid',texrepeat='1 1',texuniform='true')
    world=ET.SubElement(root,'worldbody')
    quat=numbers([math.cos(angle/2),0,math.sin(angle/2),0])
    ET.SubElement(world,'light',pos='0 -3 6',dir='0 0 -1',directional='true')
    ET.SubElement(world,'geom',name='slope',type='plane',size='150 40 .1',quat=quat,
                  material='road',rgba='1 1 1 1',contype='1',conaffinity='126')
    body=ET.SubElement(world,'body',name='chassis',pos='0 0 .003',quat=quat)
    ET.SubElement(body,'freejoint',name='root')
    ET.SubElement(body,'inertial',pos='.3 0 .65',mass='118' if geometry.get('steering_pivot') else '120',diaginertia='15 45 50')
    ET.SubElement(body,'geom',name='visual_chassis',type='mesh',mesh='chassis',
                  contype='0',conaffinity='0',group='2',rgba='.12 .3 .22 1')
    def add_collider(parent,c):
        attributes={k:numbers(c[k]) for k in ('fromto','pos','quat','size') if k in c}
        # Only axle/hub and shaft/bearing mating interfaces are exempt.
        # Chassis=2, wheels=4, axle=8, fork=16, bearing=32, shaft=64, chain=128,
        # wheel sprocket=256: each chain is exempt only from its own sprocket.
        masks={'chassis':(2,85),'steering':(16,39),'bearing':(32,21),
               'shaft':(64,7),'axle':(8,1),'chain':(128,117)}
        ct,ca=masks[c.get('role','axle' if c['axle'] else 'chassis')]
        ET.SubElement(parent,'geom',name=c['name'],type=c['type'],**attributes,
            contype=str(ct),conaffinity=str(ca),group='3',rgba='.2 .6 .3 .35')
    for c in geometry['colliders']:
        add_collider(body,c)
    steering=None
    if geometry.get('steering_pivot'):
        pivot=geometry['steering_pivot']
        limit=math.radians(geometry['steering_limit_deg'])
        steering=ET.SubElement(body,'body',name='steering',pos=numbers(pivot))
        ET.SubElement(steering,'joint',name='steering_yaw',type='hinge',axis='0 0 1',
            limited='true',range=numbers([-limit,limit]),damping='1',armature='.02')
        ET.SubElement(steering,'inertial',pos='0 0 -.1',mass='2',diaginertia='.06 .06 .02')
        ET.SubElement(steering,'geom',name='visual_steering',type='mesh',mesh='steering',
            contype='0',conaffinity='0',group='2',rgba='.35 .3 .12 1')
        for c in geometry['steering_colliders']:
            add_collider(steering,c)
        actuator=ET.SubElement(root,'actuator')
        ET.SubElement(actuator,'position',name='tiller',joint='steering_yaw',kp='180',kv='12',
            ctrllimited='true',ctrlrange=numbers([-limit,limit]),forcelimited='true',forcerange='-20 20')
    if blocked:
        # Negative control: put a chassis member through the front wheel crown.
        wb=geometry['parameters']['wheelbase']
        r=geometry['parameters']['front_radius']
        ET.SubElement(body,'geom',name='deliberate_wheel_blocker',type='capsule',
            fromto=numbers([wb-.10,0,r*1.7,wb+.10,0,r*1.7]),size='.025',contype='2',conaffinity='5')
    for name,pos,r in geometry['wheels']:
        parent=steering if name=='front' and steering is not None else body
        wheel_pos=(np.array(pos)-np.array(geometry['steering_pivot'])) if parent is steering else pos
        wheel=ET.SubElement(parent,'body',name=name,pos=numbers(wheel_pos))
        ET.SubElement(wheel,'joint',name=name+'_axle',type='hinge',axis='0 1 0',damping='.01',frictionloss='0',
            solreffriction='.004 1',solimpfriction='.999 .999 .001 .5 2')
        ET.SubElement(wheel,'inertial',pos='0 0 0',mass='5',
                      diaginertia=numbers([2.5*r*r,5*r*r,2.5*r*r]))
        ET.SubElement(wheel,'geom',name=name+'_rolling',type='cylinder',
            size=numbers([r,max(.030,geometry['parameters']['tire_radius'])]),
            quat='.7071067811865476 .7071067811865476 0 0',
            contype='4',conaffinity='115',group='3',rgba='.2 .2 .2 .4')
        for c in geometry.get('wheel_colliders',{}).get(name,[]):
            ET.SubElement(wheel,'geom',name=c['name'],type=c['type'],
                fromto=numbers(c['fromto']),size=numbers(c['size']),
                contype='256' if c.get('role')=='sprocket' else '4',conaffinity='115',
                group='3',rgba='.6 .4 .1 .5')
        ET.SubElement(wheel,'geom',name='visual_'+name,type='mesh',mesh=name,
                      contype='0',conaffinity='0',group='2',rgba='.18 .18 .18 1')
    if geometry.get('drive'):
        # Open differential: the actuator acts on the mean rear-axle angle, so
        # each wheel gets half the torque and their speeds are free to differ.
        # Its velocity is the engine speed (overall ratio x mean wheel speed).
        tendon=ET.SubElement(root,'tendon')
        fixed=ET.SubElement(tendon,'fixed',name='differential')
        for name in ('rear_left','rear_right'):
            ET.SubElement(fixed,'joint',joint=name+'_axle',coef='.5')
        if engine_torque_nm>0:
            # Belt on the fixed pulley. Without it (the default) there is no drive path.
            actuator=root.find('actuator')
            if actuator is None:
                actuator=ET.SubElement(root,'actuator')
            ET.SubElement(actuator,'motor',name='drive',tendon='differential',
                gear=str(geometry['drive']['overall_ratio']),ctrllimited='true',
                ctrlrange=numbers([0,engine_torque_nm]))
        if spool:
            # Negative control: a locked differential forces equal wheel speeds.
            equality=ET.SubElement(root,'equality')
            ET.SubElement(equality,'joint',joint1='rear_left_axle',joint2='rear_right_axle',polycoef='0 1 0 0 0')
    path=out/('blocked.xml' if blocked else 'downhill.xml')
    ET.indent(root)
    ET.ElementTree(root).write(path,encoding='unicode')
    return mujoco.MjModel.from_xml_path(str(path))


def simulate(model,geometry,angle,seconds,viewer=False,control=None):
    data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    downhill=np.array([math.cos(angle),0,-math.sin(angle)])
    normal=np.array([math.sin(angle),0,math.cos(angle)])
    initial=data.qpos[:3].copy()
    wheels={name:(model.joint(name+'_axle').qposadr[0],r) for name,_,r in geometry['wheels']}
    forbidden=set()
    wheel_ground={name:0 for name in wheels}
    rows=[]
    max_tilt=0.0
    handle=None
    if viewer:
        from mujoco import viewer as mjviewer
        handle=mjviewer.launch_passive(model,data)
        handle.opt.geomgroup[3]=0
        handle.cam.distance=5
        handle.cam.elevation=-20
    try:
        for step in range(round(seconds/model.opt.timestep)):
            start=time.monotonic()
            command=control(model,data) if control else 0.0
            mujoco.mj_step(model,data)
            if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
                raise RuntimeError('Simulation became non-finite')
            for contact in data.contact:
                a=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,contact.geom1)
                b=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,contact.geom2)
                if contact.dist > 0:
                    continue
                if 'slope' in (a,b):
                    other=b if a=='slope' else a
                    if other.endswith('_rolling'):
                        wheel_ground[other.removesuffix('_rolling')]+=1
                    else:
                        forbidden.add(('ground_drag',other))
                else:
                    forbidden.add(tuple(sorted((a,b))))
            up=data.body('chassis').xmat.reshape(3,3)[:,2]
            max_tilt=max(max_tilt,math.degrees(math.acos(np.clip(up@normal,-1,1))))
            travel=float((data.qpos[:3]-initial)@downhill)
            if step%20==0:
                rows.append([float(data.time),travel]+[float(data.qpos[q]) for q,r in wheels.values()]+[float(data.qvel[:3]@downhill),float(command)])
            if handle:
                if not handle.is_running():
                    raise RuntimeError('Viewer closed before the test completed')
                handle.cam.lookat[:]=data.body('chassis').xpos
                handle.sync()
                time.sleep(max(0,model.opt.timestep-(time.monotonic()-start)))
    finally:
        if handle:
            handle.close()
    rolling={name:float(data.qpos[q]*r) for name,(q,r) in wheels.items()}
    errors={name:abs(distance-travel)/max(abs(travel),.001) for name,distance in rolling.items()}
    warnings={mujoco.mjtWarning(i).name:int(w.number) for i,w in enumerate(data.warning) if w.number}
    checks=dict(downhill_progress=travel>1,all_wheels_rotate=all(v>1 for v in rolling.values()),
        rolling_not_sliding=max(errors.values())<.10,no_forbidden_contacts=not forbidden,
        all_wheels_touch_ground=all(v>0 for v in wheel_ground.values()),upright=max_tilt<10,
        no_solver_warnings=not warnings)
    return dict(passed=all(checks.values()),checks=checks,travel_m=travel,
        final_speed_mps=float(data.qvel[:3]@downhill),
        wheel_rolling_distance_m=rolling,relative_rolling_error=errors,
        forbidden_contacts=sorted(forbidden),max_tilt_deg=max_tilt,
        wheel_ground_contact_counts=wheel_ground,solver_warnings=warnings),rows


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--blend',type=Path,default=ROOT/'experiments/001_patent_motorwagen/output/patent_motorwagen.blend')
    parser.add_argument('--output',type=Path,default=HERE/'output')
    parser.add_argument('--blender',default='blender')
    parser.add_argument('--slope-deg',type=float,default=5)
    parser.add_argument('--seconds',type=float,default=4)
    parser.add_argument('--viewer',action='store_true')
    args=parser.parse_args()
    if not (0<args.slope_deg<=15 and 2<=args.seconds<=30):
        parser.error('Use a slope in (0,15] degrees and a duration from 2 to 30 seconds')
    out=args.output.resolve()
    subprocess.run([args.blender,'--background',str(args.blend.resolve()),'--python-exit-code','1',
        '--python',str(HERE/'export_blender.py'),'--',str(out)],check=True)
    geometry=json.loads((out/'geometry.json').read_text())
    angle=math.radians(args.slope_deg)
    model=make_model(geometry,out,angle)
    report,rows=simulate(model,geometry,angle,args.seconds,args.viewer)
    negative,_=simulate(make_model(geometry,out,angle,blocked=True),geometry,angle,.1)
    report['negative_control_detected']=any('deliberate_wheel_blocker' in pair for pair in negative['forbidden_contacts'])
    report['passed'] &= report['negative_control_detected']
    report.update(mujoco_version=mujoco.__version__,slope_deg=args.slope_deg,seconds=args.seconds,
        assumed_mass_kg=135,source=geometry['source'],source_sha256=geometry['source_sha256'],source_objects=geometry['source_objects'],
        limitations=['Rigid chassis connections assumed, not verified','Steering commanded straight ahead for this baseline',
                    'Conservative primitive colliders; mass and friction are assumptions'])
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    with (out/'trajectory.csv').open('w') as file:
        writer=csv.writer(file)
        writer.writerow(['time_s','downhill_m','rear_left_rad','rear_right_rad','front_rad','speed_mps','brake_command'])
        writer.writerows(rows)
    print(json.dumps(report,indent=2))
    if not report['passed']:
        raise SystemExit(1)


if __name__=='__main__':
    main()
