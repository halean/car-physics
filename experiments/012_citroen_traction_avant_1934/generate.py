"""Citroën Traction Avant (1934, 7A). Run inside Blender. X forward, Y left, Z up; meters.

Front-wheel drive: the driven wheels are also the steered ones. Each front wheel hangs on
wishbones (a torsion bar in the lower one), and its half-shaft ends in a double-Cardan joint
centred on the kingpin, so the wheel can steer while it is driven. The gearbox sits ahead of the
front axle and the engine behind it. Welded unitary body with a flat floor. Rear: a tube axle on
trailing arms, sprung by transverse torsion bars. Hydraulic brakes on all four drums. The 1934
cars steer through a worm-and-roller box (rack and pinion from 1936).
--stage chassis: hull, front towers and wishbones, uprights, knuckles, steering box, drop arm and
  track rods, rear axle on its trailing arms, wheels (the gate).
--stage full: engine, clutch housing, final drive and gearbox, half-shafts with their joints,
  brakes, controls and the saloon body.
"""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import sys

import bpy
from mathutils import Euler, Vector

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('velo_generate', HERE.parent/'006_benz_velo/generate.py')
vg = importlib.util.module_from_spec(spec)
spec.loader.exec_module(vg)
mw, part = vg.mw, vg.part
material, box, rod, tube, torus = mw.material, mw.box, mw.rod, mw.tube, mw.torus

spec = importlib.util.spec_from_file_location('bodywork', HERE.parent/'006_benz_velo/bodywork.py')
bw = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bw)
spec = importlib.util.spec_from_file_location('traction_body', HERE/'body.py')
body = importlib.util.module_from_spec(spec)
spec.loader.exec_module(body)
body_y = bw.Profile(*body.WING['y'])

COLLECTION = 'VEHICLE | 1934 Citroën Traction Avant'


def ring_on_axis(name, center, radius, minor, direction, mat, V, body, joins=()):
    """Torus whose axis is 'direction', with a polyline centreline for checks."""
    obj = torus(name, center, radius, minor, mat, V, wheel=False)
    obj.rotation_mode = 'QUATERNION'
    obj.rotation_quaternion = Vector(direction).to_track_quat('Z', 'Y')
    obj['centerline'] = [[radius*math.cos(math.tau*i/32), radius*math.sin(math.tau*i/32), 0] for i in range(33)]
    obj['section_radius'] = minor
    obj['capsule'] = True
    return part(obj, body, joins)


def slab(name, x0, x1, y0, y1, z0, z1, mat, V, bevel=.003):
    """Box from its extents."""
    return box(name, ((x0+x1)/2, (y0+y1)/2, (z0+z1)/2), (x1-x0, y1-y0, z1-z0), mat, V, bevel)


def mudguard(name, center, radius, y, half_width, a0, a1, m, V, joins_first, joins_last):
    """Curved wing as short overlapping boards on an arc."""
    n = 10
    names = [f'{name} {i}' for i in range(n)]
    for i in range(n):
        a = math.radians(a0+(a1-a0)*(i+.5)/n)
        chord = 2*radius*math.sin(math.radians(a1-a0)/n/2)*1.08
        obj = box(names[i], (center[0]+radius*math.cos(a), y, center[1]+radius*math.sin(a)),
                  (chord, 2*half_width, .006), m['paint'], V, .002)
        obj.rotation_euler = Euler((0, -(a-math.pi/2), 0))
        joins = [names[i-1]] if i else []
        joins += joins_first if i == 0 else []
        joins += joins_last if i == n-1 else []
        part(obj, joins=joins)
    return names


def front_end(V, p, m, kp):
    """Towers, wishbones, uprights, knuckles; worm-and-roller box with a drop arm swinging
    sideways and a track rod from its ball to each knuckle."""
    wb, ft, fr = p['wheelbase'], p['front_track'], p['front_radius']
    L, xt, leg = p['arm_length'], p['tower_dx'], p['arm_leg_dx']
    zla, zua, zt = p['lower_arm_z'], p['upper_arm_z'], p['track_rod_z']
    yi = kp-L
    arm = p['steering_arm']
    alpha = math.atan2(kp, wb)
    balls = {}
    for s, side in ((1, 'left'), (-1, 'right')):
        y, yl = s*kp, s*yi
        part(slab(f'Front longeron {s}', 1.975, 3.56, s*.385, s*.415, .58, .78, m['paint'], V) if s > 0 else
             slab(f'Front longeron {s}', 1.975, 3.56, -.415, -.385, .58, .78, m['paint'], V), joins=['Front bulkhead'])
        for tx, where in ((wb-xt, 'rear'), (wb+xt, 'front')):
            y0, y1 = sorted((s*(yi-.015), s*p['tower_outer_y']))
            part(slab(f'Suspension tower {where} {s}', tx-.01, tx+.01, y0, y1, .16, .60, m['frame'], V),
                 joins=[f'Tower bracket {where} {s}'])
            y0, y1 = sorted((s*(p['tower_outer_y']-.01), s*.40))
            part(slab(f'Tower bracket {where} {s}', tx-.01, tx+.01, y0, y1, .575, .60, m['frame'], V),
                 joins=[f'Front longeron {s}'])   # above the tyre's reach at full lock
        for z, level in ((zla, 'Lower'), (zua, 'Upper')):
            part(rod(f'{level} pin {s}', (wb-xt-.01, yl, z), (wb+xt+.01, yl, z), .015, m['steel'], V),
                 joins=[f'Suspension tower rear {s}', f'Suspension tower front {s}'])
            body = f'{level.lower()}_arm_{side}'
            for dx, end in ((-leg, 'rear'), (leg, 'front')):
                part(rod(f'{level} arm {end} leg {s}', (wb+dx, yl, z), (wb, y, z), .018, m['frame'], V), body,
                     joins=[f'{level} pin {s}', f'{level} ball {s}']+([f'{level} arm front leg {s}'] if end == 'rear' else []))
            part(rod(f'{level} ball {s}', (wb, y, z-.03), (wb, y, z+.03), .025, m['steel'], V), body)
        # The torsion bar runs back from the lower pivot to an anchor at the bulkhead; its twist is
        # the lower arm's hinge spring.
        part(rod(f'Front torsion bar {s}', (wb-xt-.01, yl, zla), (2.03, yl, zla), .013, m['steel'], V),
             joins=[f'Lower pin {s}', f'Torsion bar anchor {s}', f'Suspension tower rear {s}'])
        part(slab(f'Torsion bar anchor {s}', 1.97, 2.03, yl-.03, yl+.03, .19, .30, m['iron'], V), joins=['Front bulkhead'])
        # Upright: bosses on the lower and upper balls, a spine ahead of the kingpin (inboard of the
        # wheel's sweep) and two webs. The knuckle's housing turns between the bosses.
        up = f'upright_{side}'
        part(rod(f'Upright lower boss {s}', (wb, y, zla+.028), (wb, y, fr-.045), .015, m['steel'], V), up,
             joins=[f'Lower ball {s}', f'Knuckle housing {s}'])
        part(rod(f'Upright upper boss {s}', (wb, y, fr+.055), (wb, y, zua-.028), .015, m['steel'], V), up,
             joins=[f'Upper ball {s}', f'Knuckle housing {s}'])
        spine = (wb+.09, s*(kp-.05))
        for z, name in ((zla+.05, 'lower'), (zua-.055, 'upper')):   # clear of the balls and the housing
            part(rod(f'Upright {name} web {s}', (wb, y, z), (*spine, z), .012, m['steel'], V), up,
                 joins=[f'Upright {name} boss {s}', f'Upright spine {s}'])
        part(rod(f'Upright spine {s}', (*spine, zla+.025), (*spine, zua-.025), .015, m['steel'], V), up)
        kn = f'knuckle_{side}'
        part(rod(f'Knuckle housing {s}', (wb, y, fr-.05), (wb, y, fr+.06), .045, m['iron'], V), kn)
        part(rod(f'Stub axle {s}', (wb, y+s*.04, fr), (wb, s*(ft/2+.05), fr), .02, m['steel'], V), kn,
             role='axle', joins=[f'Knuckle housing {s}'])
        ball = (wb-arm*math.cos(alpha), s*(kp-arm*math.sin(alpha)), zt)
        part(rod(f'Steering arm {s}', (wb-.035, y, zt), ball, .011, m['steel'], V), kn, joins=[f'Knuckle housing {s}'])
        balls[side] = ball
    # Worm-and-roller box on the left rear tower; its sector shaft runs lengthwise, so the drop arm
    # swings sideways and its ball moves across the car like a rack.
    by, R = p['box_y'], p['drop_arm']
    bx = balls['left'][0]
    sector = (bx, by, zt+R)
    part(slab('Steering box', wb-xt+.005, bx-.03, by-.05, by+.05, sector[2]-.05, sector[2]+.05, m['iron'], V),
         joins=['Suspension tower rear 1'])
    part(rod('Sector shaft', (bx-.035, by, sector[2]), (bx+.012, by, sector[2]), .018, m['steel'], V), 'column',
         joins=['Steering box'])
    drop_ball = (bx, by, zt)
    part(rod('Drop arm', sector, (bx, by, zt+.012), .014, m['steel'], V), 'column', joins=['Sector shaft', 'Drop arm ball'])
    part(rod('Drop arm ball', (bx-.018, by, zt), (bx+.018, by, zt), .016, m['steel'], V), 'column')
    for s, side in ((1, 'left'), (-1, 'right')):
        part(rod(f'Track rod {s}', drop_ball, balls[side], .011, m['steel'], V), f'track_rod_{side}',
             joins=['Drop arm ball', f'Steering arm {s}'], mates=[f'Track rod {-s}', 'Drop arm'])   # both on one ball
    return dict(balls=balls, drop_ball=drop_ball, sector=sector, yi=yi)


def build_full(V, p, m, kp, yi):
    wb, fr, rr, ft, rt = p['wheelbase'], p['front_radius'], p['rear_radius'], p['front_track'], p['rear_track']
    # Engine behind the front axle, clutch housing over the track rods, then the final drive and the
    # gearbox ahead of the axle.
    part(slab('Crankcase', 2.05, 2.55, -.15, .15, .34, .58, m['alu'], V, .01))
    part(slab('Cylinder block', 2.07, 2.53, -.13, .13, .58, .82, m['iron'], V, .01), joins=['Crankcase'])
    part(slab('Cylinder head', 2.08, 2.52, -.12, .12, .82, .88, m['alu'], V, .01), joins=['Cylinder block'])
    for x in (2.12, 2.42):
        for s in (-1, 1):
            part(rod(f'Engine bearer {x}.{s}', (x, s*.15, .52), (x, s*.39, .60), .018, m['frame'], V),
                 joins=['Crankcase', f'Front longeron {s}'])
    part(slab('Clutch housing', 2.55, 2.86, -.12, .12, .40, .62, m['alu'], V, .01), joins=['Crankcase', 'Final drive case'])
    part(slab('Final drive case', 2.85, 3.36, -.13, .13, .24, .50, m['alu'], V, .01))
    part(rod('Gear lever', (1.99, 0, .88), (1.80, 0, .84), .012, m['steel'], V), joins=['Front bulkhead'])   # through the dash
    # Half-shafts: inner universal joint at the final drive, outer double-Cardan joint centred on the
    # kingpin; each shaft is parallel to its wishbones and swings with them.
    for s, side in ((1, 'left'), (-1, 'right')):
        y0 = s*.13
        part(rod(f'Final drive flange {s}', (wb, s*.12, fr), (wb, s*.15, fr), .04, m['steel'], V), joins=['Final drive case'])
        sh = f'halfshaft_{side}'
        pieces = [part(rod(f'Inner joint {s}', (wb, s*.145, fr), (wb, s*(yi+.02), fr), .045, m['steel'], V), sh,
                       joins=[f'Final drive flange {s}', f'Half shaft {s}']),
                  part(rod(f'Half shaft {s}', (wb, s*(yi+.02), fr), (wb, s*(kp-.04), fr), .02, m['steel'], V), sh,
                       joins=[f'Outer joint {s}'], mates=[f'Knuckle housing {s}']),
                  # Only the joint's inner half is drawn on the shaft; its outer half turns with the knuckle.
                  part(rod(f'Outer joint {s}', (wb, s*(kp-.04), fr), (wb, s*(kp+.02), fr), .035, m['steel'], V), sh,
                       mates=[f'Knuckle housing {s}', f'Stub axle {s}'])]
        for o in pieces:
            o['sweep'] = 'solid'
        del y0
    # Brakes: a drum in each wheel, the front anchors on the knuckles, the rear on the axle.
    for s, side in ((1, 'left'), (-1, 'right')):
        wheel = f'front_{side}'
        part(rod(f'{wheel}.brake_drum', (wb, s*(kp+.055), fr), (wb, s*(kp+.115), fr), .12, m['steel'], V), wheel,
             joins=[f'{wheel}.hub'], mates=[f'Stub axle {s}', f'Front brake anchor {s}'])
        part(rod(f'Front brake anchor {s}', (wb, s*(kp+.047), fr), (wb, s*(kp+.049), fr), .125, m['iron'], V),
             f'knuckle_{side}', joins=[f'Stub axle {s}'], mates=[f'{wheel}.brake_drum'])
        wheel = f'rear_{side}'
        part(rod(f'{wheel}.brake_drum', (0, s*(rt/2-.068), rr), (0, s*(rt/2-.03), rr), .12, m['steel'], V), wheel,
             joins=[f'{wheel}.hub'], mates=['Rear axle', f'Rear brake anchor {s}'])
        part(rod(f'Rear brake anchor {s}', (0, s*(rt/2-.072), rr), (0, s*(rt/2-.070), rr), .125, m['iron'], V),
             'rear_axle', joins=['Rear axle'], mates=[f'{wheel}.brake_drum'])
    for i, y in enumerate((.16, .26, .36)):
        part(rod(f'Pedal {i+1}', (1.80, y, .29), (1.72, y, .50), .012, m['steel'], V), joins=['Floor'])
    # Radiator on a cross member between the longerons, bonnet, saloon body.
    part(slab('Front cross member', 3.46, 3.54, -.415, .415, .60, .70, m['frame'], V), joins=['Front longeron -1', 'Front longeron 1'])
    part(slab('Radiator', 3.46, 3.54, -.30, .30, .47, .99, m['nickel'], V, .01), joins=['Front cross member'])
    part(slab('Scuttle', 1.85, 1.98, -.72, .72, .95, 1.03, m['paint'], V), joins=['Front bulkhead'], mates=['Body shell'])
    # Procedural bodywork (bodywork.py, designed in body.py): cabin and boot, bonnet, wings.
    bodywork = body.build(bw, m, V, part)
    for label, xb, z0, floor in (('Front', 1.40, .29, 'Floor'), ('Rear', .15, .535, 'Rear floor')):
        for x in (xb-.14, xb+.14):
            for s in (-1, 1):
                part(rod(f'{label} seat support {x:.2f}.{s}', (x, s*.34, z0), (x, s*.34, .62), .016, m['frame'], V),
                     joins=[floor, f'{label} seat base'])
        part(slab(f'{label} seat base', xb-.20, xb+.20, -.46, .46, .62, .66, m['wood'], V))
        part(slab(f'{label} seat cushion', xb-.19, xb+.19, -.45, .45, .66, .74, m['leather'], V, .03), joins=[f'{label} seat base'])
        part(slab(f'{label} seat back', xb-.26, xb-.185, -.45, .45, .66, 1.06, m['leather'], V, .025),
             joins=[f'{label} seat base', f'{label} seat cushion'])
    # Wing stays: front from the longerons, rear from the wheel arches, each to the wing's crown.
    w = body.WING
    for s_ in (-1, 1):
        for a, end in ((140, 'rear'), (40, 'front')):
            R, (cx, cz) = w['front_radius'], w['front_centre']
            ax, az = cx+R*math.cos(math.radians(a)), cz+R*math.sin(math.radians(a))
            part(rod(f'Front wing stay {end} {s_}', (ax, s_*.41, az), (ax, s_*float(body_y(ax)), az), .012, m['frame'], V),
                 joins=[f'Front longeron {s_}', f'Wing {s_}'])
        for a, end in ((150, 'rear'), (30, 'front')):
            R, (cx, cz) = w['rear_radius'], w['rear_centre']
            ax, az = cx+R*math.cos(math.radians(a)), cz+R*math.sin(math.radians(a))
            part(rod(f'Rear wing stay {end} {s_}', (ax, s_*.52, az), (ax, s_*float(body_y(ax)), az), .012, m['frame'], V),
                 joins=[f'Rear arch {s_}', f'Wing {s_}'])
    for label, x, z in (('Front', 3.72, .45), ('Rear', -1.14, .40)):
        part(rod(f'{label} bumper', (x, -.72, z), (x, .72, z), .03, m['nickel'], V))
        for s_ in (-1, 1):
            start = (3.555, s_*.40, .62) if label == 'Front' else (-.90, s_*.40, .47)
            part(rod(f'{label} bumper iron {s_}', start, (x, s_*.40, z), .015, m['frame'], V),
                 joins=[f'{label} bumper', f'Front longeron {s_}' if label == 'Front' else 'Boot floor'])
    return dict(final_drive='gearbox ahead of the front axle, open differential, half-shafts with double-Cardan outer joints',
                bodywork=bodywork)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--stage', choices=('chassis', 'full'), required=True)
    parser.add_argument('--render', action='store_true')
    args = parser.parse_args(sys.argv[sys.argv.index('--')+1:])
    p = json.loads(args.config.read_text())
    wb, ft, rt = p['wheelbase'], p['front_track'], p['rear_track']
    fr, rr, tire = p['front_radius'], p['rear_radius'], p['tire_radius']
    kp = ft/2-p['kingpin_inset']

    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    scene = bpy.context.scene
    V = bpy.data.collections.new(COLLECTION)
    studio = bpy.data.collections.new('STUDIO | excluded from export')
    scene.collection.children.link(V)
    scene.collection.children.link(studio)
    m = dict(frame=material('Hull | grey primer', (.30, .32, .33), .3, .5),
             paint=material('Body | black', (.015, .015, .018), .2, .25),
             wood=material('Varnished ash', (.33, .16, .06), 0, .35),
             steel=material('Machined steel', (.38, .43, .45), .8, .25),
             nickel=material('Chrome', (.70, .70, .72), .95, .12),
             alu=material('Cast aluminium', (.55, .57, .58), .6, .45),
             iron=material('Cast iron', (.07, .075, .08), .7, .42),
             rubber=material('Pneumatic tyre', (.017, .021, .023), 0, .7),
             leather=material('Cloth upholstery', (.25, .20, .15), 0, .6))

    # --- Unitary hull: sills, flat floor (no tunnel: nothing drives the rear), bulkhead, the
    # riser to the raised rear floor, the rear wheel arches and the boot floor.
    for s in (-1, 1):
        y0, y1 = sorted((s*.705, s*.735))
        part(slab(f'Frame rail {s}', .40, 2.00, y0, y1, .24, .42, m['paint'], V))   # unitary body: painted sills
    part(slab('Front bulkhead', 1.96, 1.98, -.71, .71, .20, .95, m['paint'], V), joins=['Frame rail -1', 'Frame rail 1'])
    # The floor starts behind the trailing arms' pivots (the rigid arms dip with the axle's roll).
    part(slab('Floor', .615, 1.965, -.705, .705, .27, .29, m['frame'], V), joins=['Frame rail -1', 'Frame rail 1', 'Front bulkhead'])
    part(slab('Seat riser', .63, .65, -.50, .50, .28, .53, m['frame'], V), joins=['Floor', 'Rear floor'])
    part(slab('Rear floor', -.35, .64, -.52, .52, .505, .535, m['frame'], V))
    for s in (-1, 1):
        y0, y1 = sorted((s*.514, s*.526))
        part(slab(f'Rear arch {s}', -.40, .42, y0, y1, .46, .79, m['frame'], V), joins=['Rear floor', 'Boot floor'])
    part(slab('Boot floor', -.92, -.34, -.52, .52, .46, .48, m['frame'], V))
    part(rod('Rear torsion tube', (.55, -.715, rr), (.55, .715, rr), .0375, m['steel'], V), joins=['Frame rail -1', 'Frame rail 1'])

    front = front_end(V, p, m, kp)
    # Column from the box, raked back past the engine to the wheel (left-hand drive).
    top = Vector((front['sector'][0]-.06, p['box_y'], front['sector'][2]+.05))
    hub = Vector(p['steering_wheel_hub'])
    d = (hub-top).normalized()
    part(rod('Steering column', tuple(top), tuple(hub), .02, m['steel'], V), 'steering_wheel', joins=['Steering box'],
         mates=['Suspension tower rear 1', 'Front bulkhead', 'Scuttle'])   # through the tower and the dash
    ring_on_axis('Steering wheel rim', tuple(hub), .20, .012, d, m['iron'], V, 'steering_wheel')
    part(rod('Steering wheel boss', tuple(hub-d*.02), tuple(hub+d*.02), .03, m['steel'], V), 'steering_wheel',
         joins=['Steering column'])
    e1 = d.orthogonal().normalized()
    e2 = d.cross(e1)
    for i in range(3):
        a = math.tau*i/3
        v = e1*math.cos(a)+e2*math.sin(a)
        part(rod(f'Steering wheel spoke {i}', tuple(hub+v*.025), tuple(hub+v*.20), .008, m['steel'], V),
             'steering_wheel', joins=['Steering wheel boss', 'Steering wheel rim'])

    # --- Rear: tube axle, trailing arms pivoting on the torsion tube (their twist is the axle's swing
    # spring). The Panhard rod is not drawn: the model's axle has no sideways freedom to locate.
    part(rod('Rear axle', (0, -(rt/2+.03), rr), (0, rt/2+.03, rr), .0375, m['steel'], V), 'rear_axle', role='axle')
    for s in (-1, 1):
        part(rod(f'Trailing arm {s}', (.55, s*.55, rr), (0, s*.55, rr), .02, m['steel'], V), 'rear_axle',
             joins=['Rear axle'], rest_joins=['Rear torsion tube'], mates=['Rear torsion tube'])

    # --- Wheels (steel disc wheels drawn with spokes for the shared helper).
    wheels = []
    for s, side in ((1, 'left'), (-1, 'right')):
        vg.wheel(f'rear_{side}', (0, s*rt/2, rr), rr, p['spokes'], tire, (m['rubber'], m['steel'], m['steel']), V, 'Rear axle')
        vg.wheel(f'front_{side}', (wb, s*ft/2, fr), fr, p['spokes'], tire, (m['rubber'], m['steel'], m['steel']), V,
                 f'Stub axle {s}')
        wheels += [dict(name=f'rear_{side}', center=[0, s*rt/2, rr], radius=rr, parent='rear_axle', driven=False),
                   dict(name=f'front_{side}', center=[wb, s*ft/2, fr], radius=fr, parent=f'knuckle_{side}', driven=True)]
    drive = build_full(V, p, m, kp, front['yi']) if args.stage == 'full' else None

    # --- Assembly for the physics adapter.
    L, yi = p['arm_length'], front['yi']
    zla, zua = p['lower_arm_z'], p['upper_arm_z']
    amax = math.asin(p['front_travel_m']/L)
    lr = p['trailing_arm_m']
    bodies = {'chassis': dict(parent=None, origin=[0, 0, 0], joint='free')}
    loops, couplings, excluded = [], [], [['track_rod_left', 'track_rod_right']]
    joints = {}
    for side, s in (('left', 1), ('right', -1)):
        axis = [s, 0, 0]          # positive raises the wheel on both sides
        bodies[f'lower_arm_{side}'] = dict(parent='chassis', origin=[wb, s*yi, zla], joint=[
            dict(name='swing', type='hinge', axis=axis, pos=[wb, s*yi, zla], range=[-amax, amax])])
        bodies[f'upper_arm_{side}'] = dict(parent='chassis', origin=[wb, s*yi, zua], joint=dict(type='hinge', axis=axis))
        bodies[f'upright_{side}'] = dict(parent=f'lower_arm_{side}', origin=[wb, s*kp, zla],
                                         joint=dict(type='hinge', axis=axis))
        bodies[f'knuckle_{side}'] = dict(parent=f'upright_{side}', origin=[wb, s*kp, fr],
                                         joint=dict(type='hinge', axis=[0, 0, 1]))
        bodies[f'track_rod_{side}'] = dict(parent='column', origin=list(front['drop_ball']), joint=dict(type='ball'))
        loops += [dict(body1=f'upper_arm_{side}', body2=f'upright_{side}', point=[wb, s*kp, zua]),
                  dict(body1=f'track_rod_{side}', body2=f'knuckle_{side}', point=list(front['balls'][side]))]
        joints[side] = {f'lower_arm_{side}_swing': 1, f'upper_arm_{side}_joint': 1, f'upright_{side}_joint': -1}
        if drive:
            bodies[f'halfshaft_{side}'] = dict(parent='chassis', origin=[wb, s*yi, fr], joint=dict(type='hinge', axis=axis))
            couplings.append(dict(joint1=f'halfshaft_{side}_joint', joint2=f'lower_arm_{side}_swing', ratio=1.0,
                                  body1=f'halfshaft_{side}', body2=f'lower_arm_{side}'))
            joints[side][f'halfshaft_{side}_joint'] = 1
            excluded += [[f'halfshaft_{side}', f'knuckle_{side}'], [f'halfshaft_{side}', f'upright_{side}'],
                         [f'halfshaft_{side}', f'front_{side}']]
    limit = p['column_limit_deg']
    bodies['column'] = dict(parent='chassis', origin=list(front['sector']),
                            joint=dict(type='hinge', axis=[-1, 0, 0], range_deg=[-limit, limit]))
    bodies['rear_axle'] = dict(parent='chassis', origin=[0, 0, rr], joint=[
        dict(name='swing', type='hinge', axis=[0, 1, 0], pos=[lr, 0, rr], range=[-p['rear_travel_m']/lr, p['rear_travel_m']/lr]),
        dict(name='roll', type='hinge', axis=[1, 0, 0], pos=[0, 0, rr], range_deg=[-p['roll_travel_deg'], p['roll_travel_deg']])])
    bodies['steering_wheel'] = dict(parent='chassis', origin=list(top), joint=dict(type='hinge', axis=list(d)))
    for w in wheels:
        bodies[w['name']] = dict(parent=w['parent'], origin=w['center'], joint=dict(type='hinge', axis=[0, 1, 0]),
                                 wheel=dict(radius=w['radius'], half_width=max(tire, .035), driven=w['driven'], braked=True))
    couplings.append(dict(joint1='steering_wheel_joint', joint2='column_joint', ratio=p['steering_box_ratio'],
                          body1='steering_wheel', body2='column'))
    balls = front['balls']
    steering = dict(type='centre_drop_arm', sector_axis=[-1, 0, 0], sector_point=list(front['sector']),
                    drop_ball_3d=list(front['drop_ball']),
                    kingpins_3d={'left': [wb, kp, p['track_rod_z']], 'right': [wb, -kp, p['track_rod_z']]},
                    arm_balls_3d={k: list(v) for k, v in balls.items()},
                    kingpins={'left': [wb, kp], 'right': [wb, -kp]}, arm_balls={k: list(v[:2]) for k, v in balls.items()},
                    column_limit_deg=limit, kingpin_track=2*kp, wheelbase=wb)
    t, roll = p['front_travel_m'], math.radians(p['roll_travel_deg'])
    rs = p['rear_travel_m']/lr
    suspension = dict(independent_front=True, wishbones=dict(length=L, joints=joints),
                      rear_swing_joint='rear_axle_swing', rear_roll_joint='rear_axle_roll', springs_on=True,
                      check_poses=[dict(front_left=t, front_right=t, rear_swing=-rs),
                                   dict(front_left=-t, front_right=-t, rear_swing=rs),
                                   dict(front_left=t, front_right=-t, rear_roll=roll),
                                   dict(front_left=-t, front_right=t, rear_roll=-roll)])
    bpy.context.view_layer.update()
    objects = [o for o in V.objects if o.type in {'MESH', 'CURVE'}]
    for obj in objects:
        assert 'body' in obj, obj.name
    print('SEATING_OK', mw.check_drive_seating(objects))
    lo = [min((o.matrix_world @ Vector(c))[i] for o in objects for c in o.bound_box) for i in range(3)]
    hi = [max((o.matrix_world @ Vector(c))[i] for o in objects for c in o.bound_box) for i in range(3)]
    for w in wheels:
        tobj = bpy.data.objects[f'{w["name"]}.tire']
        assert abs(min((tobj.matrix_world @ Vector(c)).z for c in tobj.bound_box)) < 1e-5, w['name']

    box('Ground', (wb/2, 0, -.055), (200, 200, .10), material('Studio limestone', (.19, .215, .22), 0, .8), studio, .001)
    bpy.ops.object.camera_add(location=(5.8, -6.2, 3.2))
    camera = bpy.context.object
    for c in list(camera.users_collection):
        c.objects.unlink(camera)
    studio.objects.link(camera)
    mw.aim(camera, (wb*.45, 0, .7))
    camera.data.type = 'ORTHO'
    camera.data.ortho_scale = max(hi[0]-lo[0], hi[1]-lo[1], hi[2]-lo[2])*1.4
    scene.camera = camera
    for name, loc, power in [('Key', (2, -4, 6), 1500), ('Fill', (0, 5, 4), 1000), ('Rim', (-4, -1, 5), 1500)]:
        light = bpy.data.lights.new(name, 'AREA')
        light.energy, light.shape, light.size = power, 'DISK', 4
        obj = bpy.data.objects.new(name, light)
        studio.objects.link(obj)
        obj.location = loc
        mw.aim(obj, (1.4, 0, .7))
    scene.world.color = (.22, .22, .22)
    scene.render.engine = 'CYCLES'
    scene.cycles.samples = 32
    scene.cycles.use_denoising = True
    scene.render.resolution_x, scene.render.resolution_y = 1400, 1050
    args.output.mkdir(parents=True, exist_ok=True)
    scene.render.filepath = str(args.output/'preview.png')
    bpy.ops.wm.save_as_mainfile(filepath=str(args.output/'traction_avant.blend'))
    report = dict(blender_version=bpy.app.version_string, stage=args.stage, parameters=p, units='meters',
                  collection=COLLECTION, bodies=bodies, loops=loops, steering=steering, drive=drive,
                  couplings=couplings, excluded_pairs=excluded, suspension=suspension,
                  joins=sorted({tuple(j) for j in vg.JOINS}), rest_joins=sorted({tuple(j) for j in vg.REST_JOINS}),
                  vehicle_objects=len(objects), bounds_min=lo, bounds_max=hi, dimensions=[b-a for a, b in zip(lo, hi)],
                  historical_accuracy='approximate study of a 1934 Citroën Traction Avant 7A; front-wheel drive with '
                                      'the gearbox ahead of the axle and the engine behind it, double-Cardan outer '
                                      'joints, wishbones with torsion bars, rear tube axle on trailing arms with '
                                      'torsion bars, welded unitary body with a flat floor, hydraulic brakes, worm-and-'
                                      'roller steering, 1303 cc 32 PS, three speeds, 2.91 m wheelbase and about 95 '
                                      'km/h are sourced; mass, track, wheels, rates, ratios, the wishbone and steering '
                                      'linkage geometry and all bodywork dimensions are assumptions')
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    if args.render:
        bpy.ops.render.render(write_still=True)
    print('BUILD_OK', args.stage, args.output, len(objects), 'objects')


if __name__ == '__main__':
    main()
