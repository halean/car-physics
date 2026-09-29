"""Lancia Lambda (1922, first series). Run inside Blender. X forward, Y left, Z up; meters.

The first car with a load-bearing unitary body and independent front suspension: each front
wheel carrier slides up and down its own pillar (with a coil spring and a hydraulic damper) and
steers about it. Rear: live axle on semi-elliptic springs with friction dampers, driven by a
propeller shaft in the hull's central tunnel. Four-wheel drum brakes. Narrow-angle V4.
--stage chassis: hull (sills, floor, tunnels, front cross member), sliding pillars, steering,
  rear axle on its springs, wheels (the gate).
--stage full: engine, gearbox, propeller shaft with two universal joints, four drums, body.
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

COLLECTION = 'VEHICLE | 1922 Lancia Lambda'


def ring_on_axis(name, center, radius, minor, direction, mat, V, body, joins=()):
    """Torus whose axis is 'direction', with a polyline centreline for checks."""
    obj = torus(name, center, radius, minor, mat, V, wheel=False)
    obj.rotation_mode = 'QUATERNION'
    obj.rotation_quaternion = Vector(direction).to_track_quat('Z', 'Y')
    obj['centerline'] = [[radius*math.cos(math.tau*i/32), radius*math.sin(math.tau*i/32), 0] for i in range(33)]
    obj['section_radius'] = minor
    obj['capsule'] = True
    return part(obj, body, joins)


def coil_spring(name, x, y, z0, z1, radius, m, V, body, joins, rest_joins, mates, turns=6):
    """A coil drawn as a helix (visual) with a cylinder as its collider (the envelope). The drawn
    coil is rigid; its flex is the pillar joint's, so its top seat is a rest join."""
    n = turns*16
    pts = [(x+radius*math.cos(math.tau*i/16), y+radius*math.sin(math.tau*i/16), z0+(z1-z0)*i/n) for i in range(n+1)]
    part(tube(f'{name} coil', pts, .006, m['steel'], V), body, collide=False)
    env = rod(name, (x, y, z0), (x, y, z1), radius+.006, m['steel'], V)
    env['envelope'] = True
    return part(env, body, joins=joins, rest_joins=rest_joins, mates=list(rest_joins)+list(mates))


def leaf_spring(name, x0, x1, y, z_end, z_mid, m, V, joins, rest_joins, mates):
    """Semi-elliptic spring drawn at ride height; its flex is the axle joints'."""
    pts = [(x0+(x1-x0)*t, y, z_mid+(z_end-z_mid)*(2*t-1)**2) for t in [i/8 for i in range(9)]]
    return part(tube(name, pts, .02, m['iron'], V), joins=joins, rest_joins=rest_joins,
                mates=list(rest_joins)+list(mates))


def mudguard(name, center, radius, y, half_width, a0, a1, m, V, joins_first, joins_last):
    """Curved wing as short overlapping boards on an arc."""
    n = 10
    names = [f'{name} {i}' for i in range(n)]
    for i in range(n):
        a = math.radians(a0+(a1-a0)*(i+.5)/n)
        chord = 2*radius*math.sin(math.radians(a1-a0)/n/2)*1.08
        obj = box(names[i], (center[0]+radius*math.cos(a), y, center[1]+radius*math.sin(a)),
                  (chord, 2*half_width, .006), m['black'], V, .002)
        obj.rotation_euler = Euler((0, -(a-math.pi/2), 0))
        joins = [names[i-1]] if i else []
        joins += joins_first if i == 0 else []
        joins += joins_last if i == n-1 else []
        part(obj, joins=joins)
    return names


def sliding_pillar_front(V, p, m, kp, cx, cy):
    """Independent front: a fixed pillar (kingpin) each side between the hull's upper and lower
    arms; a carrier slides on it (the coil spring above, the damper inside the pillar, not
    drawn); the knuckle turns on the carrier. Steering as the shared double-pivot layout:
    Jeantaud tie rod behind the wheels, drag link from a box on the left sill."""
    wb, ft, fr = p['wheelbase'], p['front_track'], p['front_radius']
    sy = p['sill_y']
    zt, zl, zd, zp = p['linkage_heights']
    arm = p['steering_arm']
    alpha = math.atan2(kp, wb)
    z_lo, z_hi = fr-.10, fr+.12         # carrier at ride height
    balls = {}
    hy = p['horn_y']
    part(box('Front cross member', (wb, 0, .50), (.06, 2*hy-.03, .28), m['frame'], V, .006),
         joins=['Front horn -1', 'Front horn 1'])
    for s, side in ((1, 'left'), (-1, 'right')):
        body, pillar = f'knuckle_{side}', f'pillar_{side}'
        y = s*kp
        part(rod(f'Pillar {s}', (wb, y, .13), (wb, y, .86), .016, m['steel'], V), joins=[f'Pillar upper arm {s}', f'Pillar lower arm {s}'])
        part(rod(f'Pillar upper arm {s}', (wb, s*(hy-.01), .62), (wb, y, .78), .022, m['frame'], V),
             joins=[f'Front horn {s}', 'Front cross member'])
        part(rod(f'Pillar lower arm {s}', (wb, s*(hy-.01), .38), (wb, y, .14), .022, m['frame'], V),
             joins=[f'Front horn {s}', 'Front cross member'])
        part(rod(f'Pillar cap {s}', (wb, y, .84), (wb, y, .88), .03, m['steel'], V), joins=[f'Pillar {s}'])   # above the spring at full bump
        # Carrier and spring ride with the pillar body (slide joint); the knuckle turns on the carrier.
        # Sliding and turning fits are declared joins: they touch at every pose.
        part(rod(f'Carrier {s}', (wb, y, z_lo), (wb, y, z_hi), .032, m['steel'], V), pillar, joins=[f'Pillar {s}'])
        coil_spring(f'Front spring {s}', wb, y, z_hi, .76, .042, m, V, pillar, joins=[f'Carrier {s}'],
                    rest_joins=[f'Pillar upper arm {s}'], mates=[f'Pillar {s}'])
        ring_on_axis(f'Knuckle collar {s}', (wb, y, fr), .0435, .0125, (0, 0, 1), m['steel'], V, body,
                     joins=[f'Carrier {s}'])
        part(rod(f'Stub axle {s}', (wb, y+s*.05, fr), (wb, s*(ft/2+.05), fr), .018, m['steel'], V), body,
             role='axle', joins=[f'Knuckle collar {s}'])
        ball = (wb-arm*math.cos(alpha), s*(kp-arm*math.sin(alpha)), zt)
        part(rod(f'Steering arm {s}', (wb-.06, y, fr), ball, .011, m['steel'], V), body,
             joins=[f'Knuckle collar {s}'])
        balls[side] = ball
    drag_ball = (wb+p['drag_arm_vector'][0], kp+p['drag_arm_vector'][1], zl)
    part(rod('Drag arm', (wb-.05, kp, fr), (drag_ball[0], drag_ball[1], zd), .011, m['steel'], V),
         'knuckle_left', joins=['Knuckle collar 1', 'Steering arm 1'])   # forged with the steering arm
    part(rod('Drag arm stud', (drag_ball[0], drag_ball[1], zd), drag_ball, .008, m['steel'], V), 'knuckle_left',
         joins=['Drag arm'])
    part(rod('Tie rod', balls['left'], balls['right'], .011, m['steel'], V), 'tie_rod',
         joins=['Steering arm 1', 'Steering arm -1'])
    pit_dx, pit_dy = p['pitman_arm_vector']
    pitman_ball = (cx+pit_dx, cy+pit_dy, zl)
    part(rod('Steering column', (cx, cy, zp), (cx, cy, .55), .016, m['steel'], V), 'column', joins=['Column bearing'])
    part(rod('Pitman arm', (cx, cy, zp), (cx+pit_dx, cy+pit_dy, zp), .011, m['steel'], V), 'column',
         joins=['Steering column'])
    part(rod('Pitman stud', (cx+pit_dx, cy+pit_dy, zp), pitman_ball, .008, m['steel'], V), 'column', joins=['Pitman arm'])
    part(rod('Drag link', pitman_ball, drag_ball, .011, m['steel'], V), 'drag_link', joins=['Pitman stud', 'Drag arm stud'])
    return dict(balls=balls, drag_ball=drag_ball, pitman_ball=pitman_ball, zd=zd, zl=zl)


def build_full(V, p, m):
    wb, rr, fr, rt, ft = p['wheelbase'], p['rear_radius'], p['front_radius'], p['rear_track'], p['front_track']
    sy = p['sill_y']
    # Narrow-angle V4 (13 degrees: one block) on the hull, gearbox in unit behind it.
    part(box('Crankcase', (2.75, 0, .58), (.60, .34, .22), m['alu'], V, .01))
    part(box('Cylinder block', (2.75, 0, .84), (.44, .26, .30), m['alu'], V, .01), joins=['Crankcase'])
    part(box('Cylinder head', (2.75, 0, 1.02), (.42, .24, .06), m['alu'], V, .01), joins=['Cylinder block'])
    for x in (2.50, 2.95):
        for s in (-1, 1):
            part(rod(f'Engine bearer {x}.{s}', (x, s*.17, .60), (x, s*(p['horn_y']-.01), .58), .02, m['frame'], V),
                 joins=['Crankcase', f'Front horn {s}'])
    part(box('Gearbox', (2.25, 0, .55), (.40, .28, .24), m['alu'], V, .01), joins=['Crankcase'])   # through the bulkhead's opening
    part(box('Radiator', (3.36, 0, .86), (.10, .72, .72), m['nickel'], V, .01), joins=['Front horn -1', 'Front horn 1'])
    # Propeller shaft: front universal joint behind the gearbox, sliding spline, rear universal
    # joint on the pinion nose. The shaft angles and lengthens as the axle rises (two bodies).
    F, R = Vector(p['front_uj']), Vector(p['rear_uj'])
    mid = (F+R)/2
    part(rod('Gearbox output', (2.10, 0, F.z), tuple(F+Vector((.01, 0, 0))), .028, m['steel'], V), joins=['Gearbox'])
    uj_f = part(rod('Front universal joint', tuple(F-Vector((.04, 0, 0))), tuple(F+Vector((.04, 0, 0))), .045,
                    m['steel'], V), 'propshaft', joins=['Gearbox output'])
    shaft = part(rod('Propeller shaft', tuple(F-Vector((.02, 0, 0))), tuple(mid), .03, m['steel'], V), 'propshaft',
                 joins=['Front universal joint', 'Propeller shaft rear'])
    rear = part(rod('Propeller shaft rear', tuple(mid+(F-mid)*.15), tuple(R+Vector((.02, 0, 0))), .024, m['steel'], V),
                'propshaft_rear', joins=['Rear universal joint'])
    uj_r = part(rod('Rear universal joint', tuple(R-Vector((.04, 0, 0))), tuple(R+Vector((.04, 0, 0))), .045,
                    m['steel'], V), 'propshaft_rear', joins=['Pinion nose'])
    for o in (uj_f, shaft, rear, uj_r):
        o['sweep'] = 'solid'
    part(rod('Pinion nose', (R.x-.01, 0, R.z), (.08, 0, R.z-.01), .04, m['iron'], V), 'rear_axle',
         joins=['Differential housing'])
    # Four-wheel brakes: a drum inside each wheel; front anchors on the knuckles, rear on the axle.
    for s, side in ((1, 'left'), (-1, 'right')):
        wheel = f'front_{side}'
        part(rod(f'{wheel}.brake_drum', (wb, s*.645, fr), (wb, s*.70, fr), .13, m['steel'], V), wheel,
             joins=[f'{wheel}.hub'], mates=[f'Stub axle {s}', f'Front brake anchor {s}'])
        part(rod(f'Front brake anchor {s}', (wb, s*.6375, fr), (wb, s*.6395, fr), .135, m['iron'], V), f'knuckle_{side}',
             joins=[f'Stub axle {s}'], mates=[f'{wheel}.brake_drum'])
        wheel = f'rear_{side}'
        part(rod(f'{wheel}.carrier', (0, s*.585, rr), (0, s*.67, rr), .045, m['iron'], V), wheel,
             joins=[f'{wheel}.hub'], mates=['Rear axle'])
        part(rod(f'{wheel}.brake_drum', (0, s*.57, rr), (0, s*.63, rr), .13, m['steel'], V), wheel,
             joins=[f'{wheel}.carrier'], mates=['Rear axle', f'Rear brake anchor {s}'])
        part(rod(f'Rear brake anchor {s}', (0, s*.560, rr), (0, s*.566, rr), .135, m['iron'], V), 'rear_axle',
             joins=['Rear axle'], mates=[f'{wheel}.brake_drum'])
    # Controls: three pedals through the floor, gear and brake levers on the tunnel.
    for i, y in enumerate((-.22, -.32, -.42)):
        part(rod(f'Pedal {i+1}', (1.62, y, .37), (1.56, y, .66), .012, m['steel'], V), joins=['Floor -1'])
    for name, x in (('Gear lever', 1.66), ('Hand brake lever', 1.58)):
        part(rod(name, (x, 0, .55), (x-.06, 0, 1.00), .014, m['steel'], V), joins=['Tunnel top'])
    # Torpedo body: scuttle, bonnet, seats, rear tonneau, wings and running boards.
    part(box('Scuttle', (1.78, 0, .95), (.02, 2*sy+.06, .62), m['paint'], V, .006), joins=['Frame rail -1', 'Frame rail 1'])
    for s in (-1, 1):
        part(box(f'Bonnet side {s}', (2.55, s*.36, .97), (1.52, .012, .50), m['paint'], V, .003),
             joins=['Scuttle', 'Radiator', 'Bonnet lid'])
        # Low-sided torpedo: the sides stop at the driver's elbow, clear of the steering wheel.
        part(box(f'Body side {s}', (.375, s*(sy+.01), .86), (1.89, .012, .44), m['paint'], V, .003),
             joins=[f'Frame rail {s}', f'Rear sill {s}', 'Tonneau back'])
    part(box('Bonnet lid', (2.55, 0, 1.22), (1.52, .72, .02), m['paint'], V, .004), joins=['Scuttle', 'Radiator'])
    part(box('Tonneau back', (-.56, 0, .86), (.02, 2*sy+.06, .44), m['paint'], V, .004),
         joins=['Rear sill -1', 'Rear sill 1', 'Rear cross member'])
    for label, xb, z0 in (('Front', 1.15, .37), ('Rear', .05, .57)):
        for x in (xb-.16, xb+.16):
            for s in (-1, 1):
                part(rod(f'{label} seat support {x:.2f}.{s}', (x, s*.30, z0), (x, s*.30, .70), .018, m['frame'], V),
                     joins=[f'Floor {s}' if label == 'Front' else 'Rear floor', f'{label} seat base'])
        part(box(f'{label} seat base', (xb, 0, .72), (.44, .90, .04), m['wood'], V))
        part(box(f'{label} seat cushion', (xb, 0, .78), (.40, .86, .08), m['leather'], V, .03), joins=[f'{label} seat base'])
        part(box(f'{label} seat back', (xb-.22, 0, .92), (.06, .86, .36), m['leather'], V, .025),
             joins=[f'{label} seat base', f'{label} seat cushion'])
    guard_r = p['wing_radius']
    for s in (-1, 1):
        for axle_x, zc, wy, label, (a0, a1), member, yy in (
                (wb, fr, s*ft/2, 'Front', (50, 150), 'Front horn', p['horn_y']), (0.0, rr, s*rt/2, 'Rear', (25, 155), 'Frame rail', sy)):
            base = f'{label} wing {s}'
            for a, end in ((a1, 'rear'), (a0, 'front')):
                ax, az = axle_x+guard_r*math.cos(math.radians(a)), zc+guard_r*math.sin(math.radians(a))
                anchor = f'{member} {s}' if not (label == 'Rear' and end == 'rear') else f'Rear sill {s}'
                if az > .66:   # the stay is above the member: a post takes it down to it
                    part(rod(f'{base} stay {end} post', (ax, s*yy, .62), (ax, s*yy, az), .012, m['frame'], V), joins=[anchor])
                    anchor = f'{base} stay {end} post'
                part(rod(f'{base} stay {end}', (ax, s*(yy+.01), az), (ax, wy-s*.08, az), .012, m['frame'], V),
                     joins=[anchor]+([f'Frame rail {s}'] if label == 'Front' and end == 'rear' else []))   # through the sill
            mudguard(base, (axle_x, zc), guard_r, wy, .10, a0, a1, m, V,
                     joins_first=[f'{base} stay front'], joins_last=[f'{base} stay rear'])
        part(box(f'Running board {s}', (1.40, s*.68, .42), (1.60, .26, .025), m['black'], V, .004))
        for x in (.80, 2.00):
            part(rod(f'Running board bracket {x}.{s}', (x, s*sy, .40), (x, s*.66, .43), .014, m['frame'], V),
                 joins=[f'Frame rail {s}', f'Running board {s}'])
    return dict(final_drive='propeller shaft with two universal joints to a live axle', shafts=['propshaft', 'propshaft_rear'])


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
    cx, cy, sy = p['column_x'], p['column_y'], p['sill_y']
    kp = ft/2-p['kingpin_inset']

    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    scene = bpy.context.scene
    V = bpy.data.collections.new(COLLECTION)
    studio = bpy.data.collections.new('STUDIO | excluded from export')
    scene.collection.children.link(V)
    scene.collection.children.link(studio)
    m = dict(frame=material('Hull | grey primer', (.30, .32, .33), .3, .5),
             paint=material('Body | Lancia blue', (.06, .10, .22), .1, .3),
             black=material('Wing | black', (.02, .02, .02), .2, .4),
             wood=material('Varnished ash', (.33, .16, .06), 0, .35),
             steel=material('Machined steel', (.38, .43, .45), .8, .25),
             nickel=material('Nickel shell', (.62, .60, .55), .9, .2),
             alu=material('Cast aluminium', (.55, .57, .58), .6, .45),
             iron=material('Cast iron', (.07, .075, .08), .7, .42),
             rubber=material('Pneumatic tyre', (.017, .021, .023), 0, .7),
             leather=material('Buttoned leather', (.12, .05, .03), 0, .45))

    # --- Unitary hull: deep pressed-steel sills (the frame rails of this body), a floor, the
    # central tunnel for the propeller shaft, the transverse tunnel over the rear axle.
    hy = p['horn_y']
    for s in (-1, 1):
        # The sills stop at the rear axle's transverse tunnel (the axle passes through the body
        # sides) and end behind the front wheels, which need room to steer.
        part(box(f'Frame rail {s}', (1.39, s*sy, .50), (2.54, .03, .28), m['frame'], V, .004))
        part(box(f'Rear sill {s}', (-.335, s*sy, .50), (.43, .03, .28), m['frame'], V, .004),
             joins=['Rear floor', 'Rear cross member'])
        part(box(f'Front horn {s}', (2.89, s*hy, .50), (1.14, .03, .28), m['frame'], V, .004), joins=[f'Front bulkhead {s}'])
        # The bulkhead is open in the middle for the gearbox; the floor is open under the tunnel.
        part(box(f'Front bulkhead {s}', (2.40, s*(sy+.17-.015)/2+s*.0, .50), (.05, sy-.015-.17, .28), m['frame'], V, .004),
             joins=[f'Frame rail {s}'])
        part(box(f'Floor {s}', (1.43, s*(sy-.015+.13)/2, .37), (1.74, sy-.015-.13, .02), m['frame'], V, .003),
             joins=[f'Frame rail {s}'])
    part(box('Rear floor', (-.13, 0, .575), (.82, 2*sy-.03, .03), m['frame'], V, .003),
         joins=['Frame rail -1', 'Frame rail 1'])      # the transverse tunnel over the rear axle
    part(box('Rear cross member', (-.55, 0, .50), (.05, 2*sy-.03, .28), m['frame'], V, .004),
         joins=['Rear floor'])
    part(box('Tunnel top', (1.17, 0, .55), (1.15, .26, .02), m['frame'], V, .003), joins=['Tunnel side -1', 'Tunnel side 1'])
    for s in (-1, 1):
        part(box(f'Tunnel side {s}', (1.17, s*.12, .46), (1.15, .02, .20), m['frame'], V, .003), joins=[f'Floor {s}'])
    # Steering box behind the front cross member (the 'Column bearing' the linkage expects).
    part(box('Column bearing', (cx, cy, .52), (.12, .10, .12), m['iron'], V, .01), joins=['Steering box bracket'])
    part(rod('Steering box bracket', (cx+.05, -(p['horn_y']-.01), .50), (cx+.05, cy-.03, .50), .02, m['frame'], V), joins=['Front horn -1'])

    # --- Front: sliding pillars, steering.
    front = sliding_pillar_front(V, p, m, kp, cx, cy)
    # Raked column and wheel: the box turns the pitman at the shared coupling ratio.
    top = Vector((cx, cy, .595))
    hub = Vector((1.55, cy, 1.15))   # right-hand drive; the column clears the engine and its bearers
    d = (hub-top).normalized()
    part(rod('Raked column', tuple(top), tuple(hub), .02, m['steel'], V), 'steering_wheel', joins=['Column bearing'],
         mates=['Scuttle'])   # passes through the scuttle
    ring_on_axis('Steering wheel rim', tuple(hub), .21, .014, d, m['wood'], V, 'steering_wheel')
    part(rod('Steering wheel boss', tuple(hub-d*.02), tuple(hub+d*.02), .035, m['steel'], V), 'steering_wheel',
         joins=['Raked column'])
    e1 = d.orthogonal().normalized()
    e2 = d.cross(e1)
    for i in range(4):
        a = math.tau*i/4
        v = e1*math.cos(a)+e2*math.sin(a)
        part(rod(f'Steering wheel spoke {i}', tuple(hub+v*.03), tuple(hub+v*.21), .01, m['steel'], V),
             'steering_wheel', joins=['Steering wheel boss', 'Steering wheel rim'])

    # --- Rear: live axle on semi-elliptic springs (their flex is the axle joints'; the friction
    # dampers are dry friction on the heave joint).
    part(rod('Rear axle', (0, -(rt/2+.03), rr), (0, rt/2+.03, rr), .04, m['iron'], V), 'rear_axle', role='axle')
    part(rod('Differential housing', (0, -.12, rr), (0, .12, rr), .09, m['iron'], V), 'rear_axle', joins=['Rear axle'])
    for s in (-1, 1):
        part(box(f'Rear spring pad {s}', (0, s*.45, rr+.05), (.10, .06, .02), m['iron'], V), 'rear_axle',
             joins=['Rear axle'])
        leaf_spring(f'Rear spring {s}', -.50, .50, s*.45, .35, rr+.08, m, V, joins=[f'Frame rail {s}', f'Rear sill {s}'],
                    rest_joins=[f'Rear spring pad {s}'], mates=['Rear axle'])

    # --- Wire wheels.
    wheels = []
    for s, side in ((1, 'left'), (-1, 'right')):
        vg.wheel(f'rear_{side}', (0, s*rt/2, rr), rr, p['spokes'], tire, (m['rubber'], m['steel'], m['steel']), V, 'Rear axle')
        vg.wheel(f'front_{side}', (wb, s*ft/2, fr), fr, p['spokes'], tire, (m['rubber'], m['steel'], m['steel']), V,
                 f'Stub axle {s}')
        wheels += [dict(name=f'rear_{side}', center=[0, s*rt/2, rr], radius=rr, parent='rear_axle', driven=True),
                   dict(name=f'front_{side}', center=[wb, s*ft/2, fr], radius=fr, parent=f'knuckle_{side}', driven=False)]
    drive = build_full(V, p, m) if args.stage == 'full' else None

    # --- Assembly for the physics adapter.
    bodies = {'chassis': dict(parent=None, origin=[0, 0, 0], joint='free')}
    steer_bodies, loops, steering = vg.steering_bodies(p, kp, cx, front, cy)
    bodies.update(steer_bodies)
    t, rt_travel, roll = p['front_travel_m'], p['rear_travel_m'], p['roll_travel_deg']
    for side, s in (('left', 1), ('right', -1)):
        bodies[f'pillar_{side}'] = dict(parent='chassis', origin=[wb, s*kp, fr], joint=[
            dict(name='heave', type='slide', axis=[0, 0, 1], pos=[wb, s*kp, fr], range=[-t, t])])
        bodies[f'knuckle_{side}']['parent'] = f'pillar_{side}'
    bodies['tie_rod']['joint'] = dict(type='ball')      # the wheels rise separately: the rod tilts
    bodies['drag_link']['joint'] = dict(type='ball')
    bodies['rear_axle'] = dict(parent='chassis', origin=[0, 0, rr], joint=[
        dict(name='heave', type='slide', axis=[0, 0, 1], pos=[0, 0, rr], range=[-rt_travel, rt_travel]),
        dict(name='roll', type='hinge', axis=[1, 0, 0], pos=[0, 0, rr], range_deg=[-roll, roll])])
    bodies['steering_wheel'] = dict(parent='chassis', origin=list(top), joint=dict(type='hinge', axis=list(d)))
    for w in wheels:
        bodies[w['name']] = dict(parent=w['parent'], origin=w['center'], joint=dict(type='hinge', axis=[0, 1, 0]),
                                 wheel=dict(radius=w['radius'], half_width=max(tire, .035), driven=w['driven'],
                                            braked=True))
    couplings = [dict(joint1='steering_wheel_joint', joint2='column_joint', ratio=p['steering_box_ratio'],
                      body1='steering_wheel', body2='column')]
    suspension = dict(independent_front=True, front_left_joint='pillar_left_heave', front_right_joint='pillar_right_heave',
                      rear_heave_joint='rear_axle_heave', rear_roll_joint='rear_axle_roll', springs_on=True,
                      check_poses=[dict(front_left=t, front_right=t, rear_heave=rt_travel),
                                   dict(front_left=-t, front_right=-t, rear_heave=-rt_travel),
                                   dict(front_left=t, front_right=-t, rear_roll=math.radians(roll)),
                                   dict(front_left=-t, front_right=t, rear_roll=-math.radians(roll))])
    if drive:
        F, R = p['front_uj'], p['rear_uj']
        axis = (Vector(R)-Vector(F)).normalized()
        bodies['propshaft'] = dict(parent='chassis', origin=list(F), joint=dict(type='ball'))
        bodies['propshaft_rear'] = dict(parent='propshaft', origin=[(F[i]+R[i])/2 for i in range(3)],
                                        joint=dict(type='slide', axis=list(axis), range=[-.04, .04]))
        loops.append(dict(body1='propshaft_rear', body2='rear_axle', point=list(R)))
        suspension['shafts'] = [dict(name='Propeller shaft', body='propshaft', slider='propshaft_rear', axle='rear_axle',
                                     F=list(F), R=list(R))]
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
    bpy.ops.object.camera_add(location=(5.6, -6.2, 3.4))
    camera = bpy.context.object
    for c in list(camera.users_collection):
        c.objects.unlink(camera)
    studio.objects.link(camera)
    mw.aim(camera, (wb*.5, 0, .7))
    camera.data.type = 'ORTHO'
    camera.data.ortho_scale = max(hi[0]-lo[0], hi[1]-lo[1], hi[2]-lo[2])*1.45
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
    bpy.ops.wm.save_as_mainfile(filepath=str(args.output/'lancia_lambda.blend'))
    report = dict(blender_version=bpy.app.version_string, stage=args.stage, parameters=p, units='meters',
                  collection=COLLECTION, bodies=bodies, loops=loops, steering=steering, drive=drive,
                  couplings=couplings, suspension=suspension,
                  joins=sorted({tuple(j) for j in vg.JOINS}), rest_joins=sorted({tuple(j) for j in vg.REST_JOINS}),
                  vehicle_objects=len(objects), bounds_min=lo, bounds_max=hi, dimensions=[b-a for a, b in zip(lo, hi)],
                  historical_accuracy='approximate study of a first-series Lancia Lambda; wheelbase, unitary body, '
                                      'sliding-pillar front suspension with coil springs and hydraulic dampers, rear '
                                      'semi-elliptics with friction dampers, four-wheel drum brakes, V4 rating and '
                                      'three speeds are sourced; track, wheels, masses, rates, ratios, the drive line '
                                      'and the steering layout are assumptions')
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    if args.render:
        bpy.ops.render.render(write_still=True)
    print('BUILD_OK', args.stage, args.output, len(objects), 'objects')


if __name__ == '__main__':
    main()
