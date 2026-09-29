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
    """Joint positions for a sprung car: the axles (or independent pillars) at the given pose and
    the steering loops closed in 3D (the drag link runs from the frame to moving wheels, so
    travel steers them: bump steer). Returns {joint name: qpos}.
    pose keys: front_heave | front_swing, front_roll, rear_swing | rear_heave, rear_roll; or, with
    independent_front, front_left and front_right (each pillar's travel, up positive)."""
    s, sus = geometry['steering'], geometry['suspension']
    angle = lambda v: math.atan2(v[1], v[0])
    q = {'column_joint': phi}
    if s.get('type') != 'centre_drop_arm':    # pitman and drag link (double pivot)
        C, P0 = np.array(s['column_3d']), np.array(s['pitman_ball_3d'])
        D0, KL = np.array(s['drag_ball_3d']), np.array(s['kingpin_left_3d'])
        P = C+_rz(phi)@(P0-C)
        drag = np.linalg.norm(D0-P0)
    if sus.get('independent_front') and s.get('type') == 'centre_drop_arm':
        # Drop arm on a lengthwise sector shaft swings sideways; a track rod from its ball to each
        # knuckle (split track rods, so each wheel's travel acts on its own rod only).
        hl, hr = pose.get('front_left', 0.0), pose.get('front_right', 0.0)
        ups = {'left': _front_travel(sus, 'left', hl), 'right': _front_travel(sus, 'right', hr)}
        axis = np.array(s['sector_axis'], dtype=float)
        Csec, B0 = np.array(s['sector_point']), np.array(s['drop_ball_3d'])
        B = Csec+_axis_angle(axis, phi)@(B0-Csec)
        qc = np.array([math.cos(phi/2), *(math.sin(phi/2)*axis/np.linalg.norm(axis))])
        qc_inv = qc*np.array([1, -1, -1, -1])
        for side in ('left', 'right'):
            KP, A0 = np.array(s['kingpins_3d'][side]), np.array(s['arm_balls_3d'][side])
            up, _ = ups[side]
            A = lambda t: KP+up+_rz(t)@(A0-KP)
            L = np.linalg.norm(A0-B0)
            t = _solve_bracketed(lambda t: np.linalg.norm(A(t)-B)-L, phi)
            q[f'knuckle_{side}_joint'] = t
            q[f'track_rod_{side}_joint'] = _quat_mul(qc_inv, _quat_between(A0-B0, A(t)-B))
        q.update(_wishbone_joints(sus, ups))
        left, right = q['knuckle_left_joint'], q['knuckle_right_joint']
        for key in ('rear_swing', 'rear_heave', 'rear_roll'):
            if f'{key}_joint' in sus:
                q[sus[f'{key}_joint']] = pose.get(key, 0.0)
        for c in geometry.get('couplings', []):
            if c['joint2'] == 'column_joint':
                q[c['joint1']] = c['ratio']*phi
        return q
    if sus.get('independent_front'):
        # Sliding pillars: each wheel carrier rises on its own vertical pillar and steers about it.
        hl, hr = pose.get('front_left', 0.0), pose.get('front_right', 0.0)
        up_l, up_r = _front_travel(sus, 'left', hl)[0], _front_travel(sus, 'right', hr)[0]
        D = lambda t: KL+up_l+_rz(t)@(D0-KL)
        left = _solve_bracketed(lambda t: np.linalg.norm(D(t)-P)-drag, phi)
        KL3, KR3 = np.array(s['kingpins_3d']['left']), np.array(s['kingpins_3d']['right'])
        AL0, AR0 = np.array(s['arm_balls_3d']['left']), np.array(s['arm_balls_3d']['right'])
        AL = KL3+up_l+_rz(left)@(AL0-KL3)
        tie_len = np.linalg.norm(AR0-AL0)
        AR = lambda t: KR3+up_r+_rz(t)@(AR0-KR3)
        right = _solve_bracketed(lambda t: np.linalg.norm(AR(t)-AL)-tie_len, left)
        # The tie rod is a ball joint on the left arm's ball: shortest rotation of its rest
        # direction, expressed in the left knuckle's frame.
        tie_world = _quat_between(AR0-AL0, AR(right)-AL)
        q['tie_rod_joint'] = _quat_mul(np.array([math.cos(-left/2), 0, 0, math.sin(-left/2)]), tie_world)
        q[sus['front_left_joint']], q[sus['front_right_joint']] = hl, hr
        drag_end = D(left)
    else:
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
        D = lambda t: axle(KL+_rz(t)@(D0-KL))
        left = _solve_bracketed(lambda t: np.linalg.norm(D(t)-P)-drag, phi)   # drag side is near 1:1
        # Tie rod: both knuckles ride on the axle, so this loop stays planar in the axle frame.
        KL2, KR2 = np.array(s['kingpins']['left']), np.array(s['kingpins']['right'])
        AL0, AR0 = np.array(s['arm_balls']['left']), np.array(s['arm_balls']['right'])
        AL = KL2+_rot(AL0-KL2, left)
        right = _solve_bracketed(lambda t: np.linalg.norm(KR2+_rot(AR0-KR2, t)-AL)-np.linalg.norm(AR0-AL0), left)
        AR = KR2+_rot(AR0-KR2, right)
        q['tie_rod_joint'] = angle(AR-AL)-angle(AR0-AL0)-left
        q[sus['front_roll_joint']] = roll
        if 'front_swing_joint' in sus:
            q[sus['front_swing_joint']] = swing
        else:
            q[sus['front_heave_joint']] = heave
        drag_end = D(left)
    # Drag link ball joint: shortest rotation of its rest direction, in the pitman's frame.
    world = _quat_between(D0-P0, drag_end-P)
    q['drag_link_joint'] = _quat_mul(np.array([math.cos(-phi/2), 0, 0, math.sin(-phi/2)]), world)
    q['knuckle_left_joint'], q['knuckle_right_joint'] = left, right
    for key in ('rear_swing', 'rear_heave', 'rear_roll'):
        if f'{key}_joint' in sus:
            q[sus[f'{key}_joint']] = pose.get(key, 0.0)
    for c in geometry.get('couplings', []):
        if c['joint2'] == 'column_joint':
            q[c['joint1']] = c['ratio']*phi
    if sus.get('scissors'):
        q.update(_scissors(geometry, q))
    if sus.get('shafts'):
        q.update(_shafts(geometry, q))
    return q


def _front_travel(sus, side, h):
    """Displacement of an independent front upright for wheel travel h (up positive), and the arm
    angle. Sliding pillar: straight up. Equal parallel wishbones of length L (a parallelogram): the
    upright keeps its orientation and moves on an arc, inboard by L(1-cos a) as it rises."""
    arms = sus.get('wishbones')
    if not arms:
        return np.array([0.0, 0.0, h]), None
    L, s = arms['length'], (1 if side == 'left' else -1)
    a = math.asin(max(-1.0, min(1.0, h/L)))
    return np.array([0.0, s*L*(math.cos(a)-1), L*math.sin(a)]), a


def _wishbone_joints(sus, ups):
    """Joint values for parallel wishbones: lower and upper arms and drive shaft turn by a, the
    upright by -a relative to the lower arm (arm axes are set so that positive a raises the wheel)."""
    arms = sus.get('wishbones')
    if not arms:
        return {sus['front_left_joint']: ups['left'][0][2], sus['front_right_joint']: ups['right'][0][2]}
    out = {}
    for side, (_, a) in ups.items():
        for joint, ratio in arms['joints'][side].items():
            out[joint] = ratio*a
    return out


def _shafts(geometry, q):
    """Close each propeller shaft: a body on a ball joint at the front universal joint F
    (parent chassis) whose child slides along the shaft (a splined joint) to the rear universal
    joint R on the axle (a connect constraint). Returns the ball quaternion and the slide."""
    T = body_transforms(geometry, q)
    out = {}
    for s in geometry['suspension']['shafts']:
        F, R0 = np.array(s['F'], dtype=float), np.array(s['R'], dtype=float)
        R = (T[s['axle']]@np.append(R0, 1))[:3]
        out[f"{s['body']}_joint"] = _quat_between(R0-F, R-F)
        out[f"{s['slider']}_joint"] = float(np.linalg.norm(R-F)-np.linalg.norm(R0-F))
    return out


def _scissors(geometry, q):
    """Close each scissor shock absorber (e.g. Hartford friction type) to its posed axle.
    Arm A pivots on the frame at F about z (yaw) then y (pitch); arm B hinges on arm A at the
    knee E about y; arm B's end is tied to the axle at P (a ball, a connect constraint).
    Yaw keeps the arms' plane through the moved P; the knee stays on its rest side."""
    T = body_transforms(geometry, q)
    out = {}
    for s in geometry['suspension']['scissors']:
        F, E0, P0 = (np.array(s[k], dtype=float) for k in ('F', 'E', 'P'))
        P = (T[s['axle']]@np.append(P0, 1))[:3]
        a, b = np.linalg.norm(E0-F), np.linalg.norm(P0-E0)
        d0, d = P0-F, P-F
        yaw = math.atan2(d[1], d[0])-math.atan2(d0[1], d0[0])
        yaw = (yaw+math.pi) % (2*math.pi)-math.pi
        h = np.array([math.cos(math.atan2(d[1], d[0])), math.sin(math.atan2(d[1], d[0])), 0.0])
        u, w = float(d@h), float(d[2])
        r = math.hypot(u, w)
        if not abs(a-b) < r < a+b:
            raise RuntimeError(f"Shock {s['name']} cannot reach its axle at this pose")
        # Knee in the arms' plane: on the same side of F->P as at rest.
        h0 = np.array([math.cos(math.atan2(d0[1], d0[0])), math.sin(math.atan2(d0[1], d0[0])), 0.0])
        ue0, we0 = float((E0-F)@h0), float((E0-F)[2])
        side = math.copysign(1.0, float(d0@h0)*we0-float(d0[2])*ue0)
        along = (a*a-b*b+r*r)/(2*r)
        up = math.sqrt(max(0.0, a*a-along*along))
        eu = along*u/r-side*up*w/r
        ew = along*w/r+side*up*u/r
        E = F+eu*h+np.array([0, 0, ew])
        Rz = _axis_angle([0, 0, 1], yaw)
        v0, v1 = E0-F, Rz.T@(E-F)
        pitch = math.atan2(v0[2]*v1[0]-v0[0]*v1[2], v0[0]*v1[0]+v0[2]*v1[2])
        RA = Rz@_axis_angle([0, 1, 0], pitch)
        w0, w1 = P0-E0, RA.T@(P-E)
        knee = math.atan2(w0[2]*w1[0]-w0[0]*w1[2], w0[0]*w1[0]+w0[2]*w1[2])
        out[f"{s['arm_a']}_yaw"], out[f"{s['arm_a']}_pitch"], out[f"{s['arm_b']}_joint"] = yaw, pitch, knee
    return out


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
