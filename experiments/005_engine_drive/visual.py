"""Per-wheel contact-force arrows for the recorder and live viewer (display only).

MuJoCo's built-in contact arrows share one scale, so a tire's ~57 N drive force
is drawn at a tenth of its ~530 N load. Here each wheel's contacts are summed
with mj_contactForce and split into a load arrow (along the contact normal)
and a tire-force arrow (in the road plane: drive, braking and cornering),
each with its own scale.
"""
import numpy as np
import mujoco

LOAD_M_PER_N=.001      # 1 m per 1000 N
TIRE_M_PER_N=.005      # 1 m per 200 N
LOAD_RGBA=np.array([.2,.5,1,.9],dtype=np.float32)
TIRE_RGBA=np.array([1,.55,0,1],dtype=np.float32)
LEGEND=('Arrows: blue = tire load (1 m per 1000 N), '
        'orange = road force on tire (1 m per 200 N)')


def wheel_forces(model,data):
    """{wheel: (contact point, load vector, tire-force vector)} in world frame."""
    totals={}
    for i,contact in enumerate(data.contact):
        if contact.dist>0:
            continue
        names=[model.geom(g).name for g in (contact.geom1,contact.geom2)]
        if 'slope' not in names:
            continue
        wheel=next((n for n in names if n.endswith('_rolling')),None)
        if not wheel:
            continue
        local=np.zeros(6)
        mujoco.mj_contactForce(model,data,i,local)
        frame=contact.frame.reshape(3,3)
        # Force on geom2 along the contact frame; flip it if the wheel is geom1.
        sign=1 if names[1]==wheel else -1
        normal=sign*local[0]*frame[0]
        tangent=sign*(local[1]*frame[1]+local[2]*frame[2])
        point,load,tire,count=totals.get(wheel,(np.zeros(3),np.zeros(3),np.zeros(3),0))
        totals[wheel]=(point+contact.pos,load+normal,tire+tangent,count+1)
    return {w.removesuffix('_rolling'):(p/n,l,t) for w,(p,l,t,n) in totals.items()}


def _arrow(scene,start,vector,rgba,width=.025):
    if scene.ngeom>=scene.maxgeom or np.linalg.norm(vector)<1e-4:
        return
    geom=scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_ARROW,np.zeros(3),np.zeros(3),np.zeros(9),rgba)
    mujoco.mjv_connector(geom,mujoco.mjtGeom.mjGEOM_ARROW,width,start,start+vector)
    # MuJoCo 3.14 draws an arrow at half the size[2] mjv_connector sets
    # (measured against markers 1 m apart), so double it for true scale.
    geom.size[2]*=2
    scene.ngeom+=1


def add_force_arrows(model,data,scene):
    for point,load,tire in wheel_forces(model,data).values():
        lifted=point+.01*load/max(np.linalg.norm(load),1e-9)
        _arrow(scene,lifted,load*LOAD_M_PER_N,LOAD_RGBA)
        _arrow(scene,lifted,tire*TIRE_M_PER_N,TIRE_RGBA)
