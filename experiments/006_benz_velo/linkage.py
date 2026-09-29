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


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _quat_between(a, b):
    """Shortest-arc unit quaternion (w, x, y, z) rotating vector a onto b."""
    a, b = a/np.linalg.norm(a), b/np.linalg.norm(b)
    w = 1+float(a@b)
    q = np.array([w, *np.cross(a, b)])
    return q/np.linalg.norm(q)


def _quat_mul(p, q):
    w1, x1, y1, z1 = p
    w2, x2, y2, z2 = q
    return np.array([w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2,
                     w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2])


def _solve_bracketed(f, near, span=math.radians(80), step=math.radians(.5)):
    """Root of f nearest 'near': scan for sign changes, refine by bisection. Raises if the
    linkage cannot close anywhere in the span (it would bind)."""
    ts = np.arange(-span, span+step/2, step)
    vals = [f(t) for t in ts]
    roots = []
    for t0, t1, v0, v1 in zip(ts, ts[1:], vals, vals[1:]):
        if v0 == 0 or v0*v1 < 0:
            a, b, fa = t0, t1, v0
            for _ in range(60):
                mid = (a+b)/2
                fm = f(mid)
                if fa*fm <= 0:
                    b = mid
                else:
                    a, fa = mid, fm
            roots.append((a+b)/2)
    if not roots:
        raise RuntimeError('Steering linkage cannot close at this pose (it would bind)')
    return min(roots, key=lambda r: abs(r-near))


def suspended_pose(geometry, phi, pose):
    """Joint positions for a sprung car: front axle heave/roll, rear swing/roll, and the
    steering loops closed in 3D (the drag link runs from the frame to the moving axle,
    so axle travel steers the wheels: bump steer). Returns {joint name: qpos}."""
    s, sus = geometry['steering'], geometry['suspension']
    heave, roll = pose.get('front_heave', 0.0), pose.get('front_roll', 0.0)
    swing = pose.get('front_swing', 0.0)
    o = np.array(sus['front_axle_origin'])
    R_roll = _axis_angle(sus.get('front_roll_axis', [1, 0, 0]), roll)
    if 'front_swing_joint' in sus:
        # Axle located by a wishbone to a ball behind it: it swings about the ball (joint order
        # swing, then roll, as in the MJCF body) instead of heaving.
        pivot = np.array(sus['front_pivot'])
        axle = lambda p: pivot+_axis_angle([0, 1, 0], swing)@(o+R_roll@(np.array(p)-o)-pivot)
    else:
        axle = lambda p: o+R_roll@(np.array(p)-o)+np.array([0, 0, heave])
    C, P0 = np.array(s['column_3d']), np.array(s['pitman_ball_3d'])
    D0, KL = np.array(s['drag_ball_3d']), np.array(s['kingpin_left_3d'])
    P = C+_rz(phi)@(P0-C)
    drag = np.linalg.norm(D0-P0)
    D = lambda t: axle(KL+_rz(t)@(D0-KL))
    left = _solve_bracketed(lambda t: np.linalg.norm(D(t)-P)-drag, phi)   # drag side is near 1:1
    # Tie rod: both knuckles ride on the axle, so this loop stays planar in the axle frame.
    KL2, KR2 = np.array(s['kingpins']['left']), np.array(s['kingpins']['right'])
    AL0, AR0 = np.array(s['arm_balls']['left']), np.array(s['arm_balls']['right'])
    AL = KL2+_rot(AL0-KL2, left)
    right = _solve_bracketed(lambda t: np.linalg.norm(KR2+_rot(AR0-KR2, t)-AL)-np.linalg.norm(AR0-AL0), left)
    AR = KR2+_rot(AR0-KR2, right)
    angle = lambda v: math.atan2(v[1], v[0])
    tie = angle(AR-AL)-angle(AR0-AL0)-left
    # Drag link ball joint: shortest rotation of its rest direction, in the pitman's frame.
    world = _quat_between(D0-P0, D(left)-P)
    in_pitman = _quat_mul(np.array([math.cos(-phi/2), 0, 0, math.sin(-phi/2)]), world)
    q = {'column_joint': phi, 'knuckle_left_joint': left, 'knuckle_right_joint': right, 'tie_rod_joint': tie,
         'drag_link_joint': in_pitman, sus['front_roll_joint']: roll,
         sus['rear_swing_joint']: pose.get('rear_swing', 0.0), sus['rear_roll_joint']: pose.get('rear_roll', 0.0)}
    if 'front_swing_joint' in sus:
        q[sus['front_swing_joint']] = swing
    else:
        q[sus['front_heave_joint']] = heave
    for c in geometry.get('couplings', []):
        if c['joint2'] == 'column_joint':
            q[c['joint1']] = c['ratio']*phi
    return q


def ackermann(geometry, phi):
    """Inner/outer road-wheel angles and the error from ideal Ackermann (deg)."""
    if geometry.get('suspension'):
        q = suspended_pose(geometry, phi, {})
        left, right = q['knuckle_left_joint'], q['knuckle_right_joint']
    else:
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


def _axis_angle(axis, angle):
    axis = np.array(axis, dtype=float)/np.linalg.norm(axis)
    x, y, z = axis
    c, s, t = math.cos(angle), math.sin(angle), 1-math.cos(angle)
    return np.array([[t*x*x+c, t*x*y-s*z, t*x*z+s*y], [t*x*y+s*z, t*y*y+c, t*y*z-s*x],
                     [t*x*z-s*y, t*y*z+s*x, t*z*z+c]])


def _quat_matrix(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)], [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def body_transforms(geometry, q):
    """Forward kinematics at rest-frame scale: {body: 4x4} mapping rest-pose world points to
    posed world points, for hinge, slide and ball joints (q: {joint name: value})."""
    bodies = geometry['bodies']
    out = {}

    def solve(name):
        if name in out:
            return out[name]
        spec = bodies[name]
        T = np.eye(4) if not spec['parent'] else solve(spec['parent']).copy()
        if spec['joint'] != 'free':
            joints = spec['joint'] if isinstance(spec['joint'], list) else [spec['joint']]
            for j in joints:
                jname = f'{name}_{j["name"]}' if isinstance(spec['joint'], list) else f'{name}_joint'
                value = q.get(jname)
                if value is None or 'wheel' in spec:
                    continue
                anchor = np.array(j.get('pos', spec['origin']), dtype=float)
                M = np.eye(4)
                kind = j.get('type', 'hinge')
                if kind == 'slide':
                    M[:3, 3] = np.array(j['axis'])*value
                else:
                    R = _quat_matrix(value) if kind == 'ball' else _axis_angle(j['axis'], value)
                    M[:3, :3] = R
                    M[:3, 3] = anchor-R@anchor
                T = T@M
        out[name] = T
        return T
    for n in bodies:
        solve(n)
    return out
