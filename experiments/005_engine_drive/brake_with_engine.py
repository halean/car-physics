"""Engine and rear brakes at the same time on the flat, with force arrows.

Belt engaged throughout; brakes applied at 5 s:
  engine only | engine + full brakes (75 N m per wheel) | engine + light brakes (20 N m per wheel)
Writes brake_with_engine.json, brake_with_engine_<case>.csv and brake_with_engine.mp4.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import csv
import json
import math
from pathlib import Path
import subprocess
import sys
import numpy as np
import mujoco

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from control import GOVERNED, NO_LOAD, MAX_TORQUE, EngineControl
from visual import LEGEND, add_force_arrows
import run as drive_test

APPLY=5.0
SECONDS=14
out=HERE/'output'
geometry=json.loads((out/'geometry.json').read_text())
ratio=geometry['drive']['overall_ratio']
r=geometry['parameters']['rear_radius']
cases=[('engine_only','ENGINE ONLY',None),
       ('full_brake','ENGINE + FULL BRAKES',75.0),
       ('light_brake','ENGINE + LIGHT BRAKES',20.0)]


def control(torque):
    brake=drive_test.BrakeControl(apply_at=APPLY,torque_nm=torque) if torque else None
    return EngineControl(engage_at=.5,brake=brake)


def dissipated(rows,torque):
    """Energy turned into brake heat after application (J), from wheel speeds."""
    if not torque:
        return 0.0
    t=rows[:,0]
    power=rows[:,9]*torque*(np.abs(rows[:,10])+np.abs(rows[:,11]))
    return float(np.sum(power[1:]*np.diff(t)))


results={}
for key,_,torque in cases:
    model=drive_test.physics.make_model(geometry,out,0,engine_torque_nm=MAX_TORQUE)
    summary,common,rows=drive_test.simulate(model,geometry,control(torque),SECONDS,0)
    end=rows[rows[:,0]>SECONDS-1]
    results[key]=dict(brake_torque_nm_per_wheel=torque,
        speed_at_application_mps=float(np.interp(APPLY,rows[:,0],rows[:,5])),
        final_speed_mps=float(rows[-1,5]),last_second_speed_range_mps=float(np.ptp(end[:,5])),
        last_second_drift_m=float(np.ptp(end[:,1])),
        final_engine_rpm=float(rows[-1,6]),final_engine_torque_nm=float(rows[-1,7]),
        brake_heat_after_application_j=dissipated(rows,torque),
        distance_m=summary['forward_m'],checks=common)
    with (out/f'brake_with_engine_{key}.csv').open('w') as f:
        writer=csv.writer(f)
        writer.writerow(['time_s','forward_m','lateral_m','yaw_deg','steering_deg','speed_mps','engine_rpm',
            'engine_torque_nm','belt_engaged','brake','rear_left_radps','rear_right_radps','front_radps'])
        writer.writerows(rows.tolist())

# Light brake: the governor settles where engine torque x ratio = total brake torque.
light=2*20.0
settled_torque=light/ratio
settled_speed=NO_LOAD-(NO_LOAD-GOVERNED)*settled_torque/MAX_TORQUE
prediction=dict(engine_torque_nm=settled_torque,engine_rpm=settled_speed*60/math.tau,
    road_speed_mps=settled_speed/ratio*r,brake_power_w=light*settled_speed/ratio)
L=results['light_brake']
F=results['full_brake']
verdict=dict(
    full_brake_stops_against_engine=F['final_speed_mps']<.02 and F['last_second_drift_m']<.01
        and F['final_engine_torque_nm']>=MAX_TORQUE*.999,
    light_brake_matches_governor_prediction=abs(L['final_speed_mps']-prediction['road_speed_mps'])<.02*prediction['road_speed_mps']
        and L['last_second_speed_range_mps']<.01,
    engine_only_unaffected=results['engine_only']['brake_heat_after_application_j']==0,
    all_standard_checks=all(all(v['checks'].values()) for v in results.values()))
report=dict(passed=all(verdict.values()),verdict=verdict,light_brake_prediction=prediction,cases=results,
    source_sha256=geometry['source_sha256'],brakes_applied_s=APPLY)
(out/'brake_with_engine.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(dict(verdict=verdict,prediction=prediction,
    cases={k:{x:v for x,v in c.items() if x!='checks'} for k,c in results.items()}),indent=2))

# Video: three tracking side views with force arrows.
models=[drive_test.physics.make_model(geometry,out,0,engine_torque_nm=MAX_TORQUE) for _ in cases]
drive_test.physics.make_model(geometry,out,0,engine_torque_nm=MAX_TORQUE)  # restore deliverable XML
controls=[control(torque) for _,_,torque in cases]
states=[mujoco.MjData(model) for model in models]
options=mujoco.MjvOption()
options.geomgroup[3]=0
width,height,fps=560,420,30
esc=lambda text: text.replace(':','\\:').replace(',','\\,')
labels=[f"drawtext=text='{title}':x={20+i*width}:y=16:fontcolor=white:fontsize=22"
        for i,(_,title,_) in enumerate(cases)]
labels+=[f"drawtext=text='BRAKES ON':x={20+i*width}:y=46:fontcolor=red:fontsize=20:enable='gte(t,{APPLY})'"
         for i in (1,2)]
labels+=[f"drawtext=text='{esc(LEGEND)}':x=20:y={height-56}:fontcolor=white:fontsize=17",
         f"drawtext=text='{esc('Belt engaged throughout (engine driving); brakes applied at 5 s; flat road')}':"
         f"x=20:y={height-30}:fontcolor=white:fontsize=17"]
renderers=[mujoco.Renderer(model,height,width) for model in models]
process=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24',
    '-s',f'{width*3}x{height}','-r',str(fps),'-i','-','-vf',','.join(labels),'-an','-c:v','libx264',
    '-pix_fmt','yuv420p',str(out/'brake_with_engine.mp4')],stdin=subprocess.PIPE)
try:
    for frame in range(SECONDS*fps+1):
        images=[]
        for model,data,ctrl,renderer in zip(models,states,controls,renderers):
            while data.time<frame/fps:
                ctrl(model,data)
                mujoco.mj_step(model,data)
            mujoco.mj_forward(model,data)
            camera=mujoco.MjvCamera()
            camera.distance,camera.azimuth,camera.elevation=4.5,90,-12
            camera.lookat[:]=data.body('chassis').xpos+[.5,0,.3]
            renderer.update_scene(data,camera=camera,scene_option=options)
            add_force_arrows(model,data,renderer.scene)
            images.append(renderer.render())
        process.stdin.write(np.concatenate(images,axis=1).tobytes())
finally:
    process.stdin.close()
    status=process.wait()
    for renderer in renderers:
        renderer.close()
if status or not report['passed']:
    raise SystemExit(status or 1)
print(out/'brake_with_engine.mp4')
