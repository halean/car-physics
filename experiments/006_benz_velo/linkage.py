"""Planar double-pivot steering kinematics and rest-pose transforms (numpy only,
so Blender can pose the saved model with the same maths the tests use)."""
import math
import numpy as np

def _rot(v, a):
    c, s = math.cos(a), math.sin(a)
    return np.array([c*v[0]-s*v[1], s*v[0]+c*v[1]])


def _solve(f, guess):
    x = guess
    for _ in range(60):
        fx = f(x)
        if abs(fx) < 1e-12:
            return x
        d = (f(x+1e-7)-fx)/1e-7
        x -= fx/d
    if abs(f(x)) > 1e-9:
        raise RuntimeError('Steering linkage does not close at this angle')
    return x


def kinematics(geometry, phi):
    """Joint angles (rad) closing both loops for column angle phi."""
    s = geometry['steering']
    C, P0, D0 = map(np.array, (s['column'], s['pitman_ball'], s['drag_ball']))
    KL, KR = np.array(s['kingpins']['left']), np.array(s['kingpins']['right'])
    AL0, AR0 = np.array(s['arm_balls']['left']), np.array(s['arm_balls']['right'])
    drag, tie = np.linalg.norm(D0-P0), np.linalg.norm(AR0-AL0)
    P = C+_rot(P0-C, phi)
    left = _solve(lambda t: np.linalg.norm(KL+_rot(D0-KL, t)-P)-drag, 0.0)
    AL = KL+_rot(AL0-KL, left)
    right = _solve(lambda t: np.linalg.norm(KR+_rot(AR0-KR, t)-AL)-tie, left)
    AR = KR+_rot(AR0-KR, right)
    D = KL+_rot(D0-KL, left)
    angle = lambda v: math.atan2(v[1], v[0])
    tie_world = angle(AR-AL)-angle(AR0-AL0)
    drag_world = angle(D-P)-angle(D0-P0)
    return dict(column=phi, knuckle_left=left, knuckle_right=right,
                tie_rod=tie_world-left, drag_link=drag_world-phi)


def ackermann(geometry, phi):
    """Inner/outer road-wheel angles and the error from ideal Ackermann (deg)."""
    k = kinematics(geometry, phi)
    left, right = k['knuckle_left'], k['knuckle_right']
    if abs(left) < 1e-6:
        return dict(inner_deg=0.0, outer_deg=0.0, ideal_outer_deg=0.0, error_deg=0.0)
    inner, outer = (left, right) if left > 0 else (-right, -left)
    s = geometry['steering']
    ideal = math.atan(1/(1/math.tan(inner)+s['kingpin_track']/s['wheelbase']))
    return dict(inner_deg=math.degrees(inner), outer_deg=math.degrees(outer),
                ideal_outer_deg=math.degrees(ideal), error_deg=math.degrees(outer-ideal))



def world_point(geometry, body, point, angles):
    """Rest-pose world point on 'body' moved by vertical-hinge angles (wheel spin ignored)."""
    bodies = geometry['bodies']
    chain = []
    while body:
        chain.append(body)
        body = bodies[body]['parent']
    p = np.array(point, dtype=float)
    for name in chain:   # innermost first
        q = angles.get(name, 0.0)
        if q and bodies[name]['joint'] != 'free' and bodies[name]['joint']['axis'] == [0, 0, 1]:
            o = np.array(bodies[name]['origin'], dtype=float)
            p[:2] = o[:2]+_rot(p[:2]-o[:2], q)
    return p
