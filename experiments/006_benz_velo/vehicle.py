"""MuJoCo adapter for body-tagged cars (built for the Benz Velo study).

make_model  : MJCF from geometry.json and vehicle.json (linkage loops as connect constraints)
kinematics  : planar double-pivot linkage pose for a column angle
build_checks: declared joins touch, every other collider pair stays apart,
              through the full steering range (includes same-body pairs)
"""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from linkage import _rot, ackermann, kinematics, world_point  # noqa: F401

HERE = Path(__file__).resolve().parent
CONFIG = json.loads((HERE/'vehicle.json').read_text())
STATIC_GAP = .001    # undeclared parts on one rigid body must not intersect
MOVING_GAP = .005    # parts that move relative to each other keep 5 mm


def numbers(values):
    return ' '.join(f'{v:.9g}' for v in values)


def body_mass(name, spec):
    m = CONFIG['masses_kg']
    if 'wheel' in spec:
        return m['rear_wheel' if name.startswith('rear') else 'front_wheel']
    if name == 'chassis':
        others = sum(body_mass(n, s) for n, s in GEOMETRY_BODIES.items() if n != 'chassis')
        return CONFIG['total_mass_kg']-others
    return m['knuckle' if name.startswith('knuckle') else name]


GEOMETRY_BODIES = {}


def part_inertia(colliders, mass):
    """Centre of mass and inertia of a small body from its own colliders (volume-weighted
    points along each primitive). Linkage mass must not sit at a joint: a tie rod
    lumped on one ball joint pushes the steering under acceleration."""
    points, weights = [], []
    for c in colliders:
        if 'fromto' in c:
            a, b = np.array(c['fromto'][:3]), np.array(c['fromto'][3:])
            volume = math.pi*c['size'][0]**2*max(np.linalg.norm(b-a), 1e-6)
            for t in np.linspace(0, 1, 7):
                points.append(a+(b-a)*t)
                weights.append(volume/7)
        else:
            points.append(np.array(c['pos']))
            weights.append(8*np.prod(c['size']))
    p, w = np.array(points), np.array(weights)/sum(weights)*mass
    com = (p*w[:, None]).sum(axis=0)
    d = p-com
    inertia = np.zeros((3, 3))
    for vec, m in zip(d, w):
        inertia += m*(vec@vec*np.eye(3)-np.outer(vec, vec))
    inertia += np.eye(3)*1e-5*mass      # section inertia of thin rods, keeps it positive definite
    return com, [inertia[0, 0], inertia[1, 1], inertia[2, 2], inertia[0, 1], inertia[0, 2], inertia[1, 2]]


def excluded_pairs(geometry):
    """Bodies joined by a joint or loop: their relative pose is set by the
    joints alone, so the kinematic sweep checks them instead of contacts."""
    pairs = {tuple(sorted((name, b['parent']))) for name, b in geometry['bodies'].items() if b['parent']}
    pairs |= {tuple(sorted((l['body1'], l['body2']))) for l in geometry['loops']}
    return sorted(pairs)


def make_model(geometry, out, slope_rad=0.0, *, engine=None, spool=False, extra=(), path_name='velo.xml'):
    """engine: dict(torque_nm, ratios={gear: overall ratio}) adds the belts (None = disengaged)."""
    global GEOMETRY_BODIES
    GEOMETRY_BODIES = geometry['bodies']
    bodies = geometry['bodies']
    root = ET.Element('mujoco', model='Benz Velo')
    ET.SubElement(root, 'compiler', angle='radian', inertiafromgeom='false')
    # No-slip friction pass: compliant tyre contacts otherwise let braked or cornering tyres creep.
    option = ET.SubElement(root, 'option', timestep='.001', gravity='0 0 -9.81', integrator='implicitfast',
                           noslip_iterations='10')
    ET.SubElement(option, 'flag', filterparent='disable')
    ET.SubElement(root, 'visual').append(ET.Element('global', offwidth='1920', offheight='1080'))
    defaults = ET.SubElement(root, 'default')
    ET.SubElement(defaults, 'geom', friction=f'{CONFIG["tire_friction"]} .005 .0001', condim='3', solref='.005 1')
    asset = ET.SubElement(root, 'asset')
    for name in bodies:
        if (Path(out)/f'{name}.obj').stat().st_size > 1:
            ET.SubElement(asset, 'mesh', name=name, file=str(Path(out)/f'{name}.obj'))
    ET.SubElement(asset, 'texture', name='road_grid', type='2d', builtin='checker',
                  rgb1='.25 .3 .34', rgb2='.45 .5 .54', width='256', height='256')
    ET.SubElement(asset, 'material', name='road', texture='road_grid', texrepeat='1 1', texuniform='true')
    world = ET.SubElement(root, 'worldbody')
    quat = numbers([math.cos(slope_rad/2), 0, math.sin(slope_rad/2), 0])
    ET.SubElement(world, 'light', pos='0 -3 6', dir='0 0 -1', directional='true')
    # Planes collide as infinite; the drawn size is visual only.
    # Collision classes: road 1, parts 2, tyre crowns 4, wheel clearance envelopes 8.
    # The road touches parts (drag detection) and tyres, never the envelopes.
    ET.SubElement(world, 'geom', name='slope', type='plane', size='150 40 .1', quat=quat, material='road',
                  contype='1', conaffinity='6')
    elements = {}

    def add(name, parent_el):
        spec = bodies[name]
        parent = spec['parent']
        pos = np.array(spec['origin'])-(np.array(bodies[parent]['origin']) if parent else np.zeros(3))
        attrs = dict(name=name, pos=numbers(pos+([0, 0, .003] if not parent else 0)))
        if not parent:
            attrs['quat'] = quat
        el = ET.SubElement(parent_el, 'body', **attrs)
        elements[name] = el
        joint = spec['joint']
        if joint == 'free':
            ET.SubElement(el, 'freejoint', name='root')
        else:
            j = dict(name=f'{name}_joint', type='hinge', axis=numbers(joint['axis']))
            if 'wheel' in spec:
                j.update(damping='.01', solreffriction='.004 1', solimpfriction='.999 .999 .001 .5 2')
            else:
                j.update(damping='.5', armature='.01')
            if 'range_deg' in joint:
                j.update(limited='true', range=numbers(np.radians(joint['range_deg'])))
            ET.SubElement(el, 'joint', **j)
        mass = body_mass(name, spec)
        if name == 'chassis':
            ET.SubElement(el, 'inertial', pos=numbers(CONFIG['chassis_com']), mass=f'{mass:.6g}',
                          diaginertia=numbers(CONFIG['chassis_inertia']))
        elif 'wheel' in spec:
            r = spec['wheel']['radius']
            ET.SubElement(el, 'inertial', pos='0 0 0', mass=f'{mass}',
                          diaginertia=numbers([mass*r*r/2, mass*r*r, mass*r*r/2]))
            w = spec['wheel']['half_width']
            tire = CONFIG['tire']
            ET.SubElement(el, 'geom', name=f'{name}_envelope', type='cylinder', fromto=numbers([0, -w, 0, 0, w, 0]),
                          size=f'{r}', contype='8', conaffinity='2', group='3', rgba='.2 .2 .2 .2')
            ET.SubElement(el, 'geom', name=f'{name}_rolling', type='ellipsoid',
                          size=numbers([r, tire['section_half_width_m'], r]), solref=numbers(tire['contact_solref']),
                          solimp=numbers(tire.get('contact_solimp', [.9, .95, .001, .5, 2])),
                          contype='4', conaffinity='3', group='3', rgba='.1 .1 .1 .5')
        else:
            com, full = part_inertia(geometry['colliders'][name], mass)
            ET.SubElement(el, 'inertial', pos=numbers(com), mass=f'{mass}', fullinertia=numbers(full))
        if f'{name}.obj' in [Path(m.get('file')).name for m in asset.findall('mesh')]:
            ET.SubElement(el, 'geom', name=f'visual_{name}', type='mesh', mesh=name, contype='0',
                          conaffinity='0', group='2', rgba='.25 .22 .2 1')
        for c in geometry['colliders'][name]:
            attrs = {k: numbers(c[k]) for k in ('fromto', 'pos', 'quat', 'size') if k in c}
            ET.SubElement(el, 'geom', name=c['name'], type=c['type'], contype='2', conaffinity='15',
                          group='3', rgba='.2 .6 .3 .35', **attrs)
        for child in [n for n, b in bodies.items() if b['parent'] == name]:
            add(child, el)

    add('chassis', world)
    for body, geom in extra:   # e.g. negative-control blockers: (body, attribute dict)
        ET.SubElement(elements[body], 'geom', contype='2', conaffinity='15', group='1', **geom)
    contact = ET.SubElement(root, 'contact')
    for a, b in excluded_pairs(geometry):
        ET.SubElement(contact, 'exclude', body1=a, body2=b)
    equality = ET.SubElement(root, 'equality')
    for loop in geometry['loops']:
        anchor = np.array(loop['point'])-np.array(bodies[loop['body1']]['origin'])
        ET.SubElement(equality, 'connect', body1=loop['body1'], body2=loop['body2'], anchor=numbers(anchor),
                      solref='.002 1')
    servo = CONFIG['steering_servo']
    actuator = ET.SubElement(root, 'actuator')
    limit = np.radians(bodies['column']['joint']['range_deg'])
    ET.SubElement(actuator, 'position', name='column', joint='column_joint', kp=str(servo['kp']),
                  kv=str(servo['kv']), ctrllimited='true', ctrlrange=numbers(limit), forcelimited='true',
                  forcerange=numbers([-servo['force_limit_nm'], servo['force_limit_nm']]))
    driven = [n for n, b in bodies.items() if b.get('wheel', {}).get('driven')]
    if driven:
        tendon = ET.SubElement(root, 'tendon')
        # frictionloss on this tendon is the countershaft band brake (set at run time).
        fixed = ET.SubElement(tendon, 'fixed', name='differential', frictionloss='0',
                              solreffriction='.004 1', solimpfriction='.999 .999 .001 .5 2')
        for n in driven:
            ET.SubElement(fixed, 'joint', joint=f'{n}_joint', coef=f'{1/len(driven)}')
        if engine:
            # One belt per speed; the controller engages at most one.
            for gear, ratio in engine['ratios'].items():
                ET.SubElement(actuator, 'motor', name=f'drive_{gear}', tendon='differential', gear=f'{ratio}',
                              ctrllimited='true', ctrlrange=numbers([0, engine['torque_nm']]))
        if spool:
            ET.SubElement(equality, 'joint', joint1=f'{driven[0]}_joint', joint2=f'{driven[1]}_joint',
                          polycoef='0 1 0 0 0')
    ET.indent(root)
    path = Path(out)/path_name
    ET.ElementTree(root).write(path, encoding='unicode')
    return mujoco.MjModel.from_xml_path(str(path))


def set_steering(model, data, geometry, phi):
    for body, q in kinematics(geometry, phi).items():
        data.qpos[model.joint(f'{body}_joint').qposadr[0]] = q


# ---------- spatial checks on the actual colliders ----------

def build_checks(model, geometry, step_deg=.5):
    """Joins touch; all other collider pairs apart; repeated over the steering range."""
    data = mujoco.MjData(model)
    geoms = [g for g in range(model.ngeom) if model.geom_bodyid[g] and model.geom_contype[g]]
    obj = {g: model.geom(g).name.split(':')[0].removesuffix('_rolling').removesuffix('_envelope') for g in geoms}
    body = {g: model.body(model.geom_bodyid[g]).name for g in geoms}
    joins = {tuple(sorted(j)) for j in geometry['joins']}
    mates, spinning = set(), set()
    for items in geometry['colliders'].values():
        for c in items:
            mates |= {tuple(sorted((c['object'], m))) for m in c['mates']}
            if c.get('sweep'):
                spinning.add(c['object'])   # rotating drive part: checked as its swept solid
    # An axle joined to a wheel's hub runs through that wheel's swept envelope.
    for a, b in list(joins):
        for hub, axle in ((a, b), (b, a)):
            if hub.endswith('.hub'):
                mates.add(tuple(sorted((axle, hub.removesuffix('.hub')))))
    steering = {'knuckle_left', 'knuckle_right', 'tie_rod', 'column', 'drag_link', 'front_left', 'front_right'}
    wheel_bodies = {n for n, b in geometry['bodies'].items() if 'wheel' in b}
    pairs = []
    for i, a in enumerate(geoms):
        for b in geoms[:i]:
            key = tuple(sorted((obj[a], obj[b])))
            if obj[a] == obj[b]:
                continue
            if key not in joins and (key in mates or (body[a] == body[b] and body[a] in wheel_bodies)):
                continue   # moving mates; wheel parts sit inside their own wheel envelope
            pairs.append((a, b, key, key in joins, body[a] != body[b] or bool({obj[a], obj[b]} & spinning)))
    limit = geometry['bodies']['column']['joint']['range_deg'][1]
    angles = np.arange(-limit, limit+1e-9, step_deg)
    join_gap, failures, closest = {}, [], {}
    for angle in angles:
        touching = {}   # object pair -> closest collider pieces at this angle
        mujoco.mj_resetData(model, data)
        set_steering(model, data, geometry, math.radians(angle))
        mujoco.mj_forward(model, data)
        for a, b, key, joined, moving in pairs:
            if angle != 0 and not ({body[a], body[b]} & steering):
                continue   # rigid relation: rest pose covers it
            gap = mujoco.mj_geomDistance(model, data, a, b, .05, None)
            if joined:
                touching[key] = min(touching.get(key, 1), gap)
                continue
            need = MOVING_GAP if moving else STATIC_GAP
            closest[key] = min(closest.get(key, (1, 0)), (gap, angle))
            if gap < need:
                failures.append(dict(pair=list(key), angle_deg=float(angle), gap_m=float(gap), required_m=need))
        for key, gap in touching.items():
            join_gap[key] = max(join_gap.get(key, -1), gap)   # must touch at every angle
    loose = sorted(k for k in joins if join_gap.get(k, 1) > 5e-4)
    join_worst = max(join_gap.values()) if join_gap else None
    # Connectivity over declared, verified joins.
    graph = {o: set() for o in set(obj.values())-wheel_bodies}
    for a, b in joins:
        if a in graph and b in graph and (a, b) not in loose:
            graph[a].add(b)
            graph[b].add(a)
    reached, todo = set(), ['Frame rail 1']
    while todo:
        n = todo.pop()
        if n not in reached:
            reached.add(n)
            todo += graph[n]-reached
    unknown = sorted({j for pair in joins for j in pair}-set(graph))
    worst = sorted(((g, a, k) for k, (g, a) in closest.items()))[:8]
    by_pair = {}
    for f in failures:
        by_pair.setdefault(tuple(f['pair']), f)
    return dict(passed=not failures and not loose and reached == set(graph) and not unknown,
                pairs=len(pairs), angles=len(angles), joins_checked=len(joins), worst_join_gap_m=join_worst,
                loose_joins=loose, disconnected=sorted(set(graph)-reached), unknown_join_parts=unknown,
                failures=list(by_pair.values())[:30],
                closest=[dict(pair=list(k), gap_m=float(g), angle_deg=float(a)) for g, a, k in worst])
