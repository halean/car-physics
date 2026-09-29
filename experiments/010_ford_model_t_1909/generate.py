"""Ford Model T (1909 touring). Run inside Blender. X forward, Y left, Z up; meters.

Transverse semi-elliptic springs on both beam axles. Each axle is located by a
ball: the front by a wishbone to a socket under the crankcase, the rear by the
torque tube's ball behind the planetary transmission. So each axle swings and
rolls about its ball while its spring carries the load. Left-hand drive; the
drag link runs at the front axle's roll-axis height.
--stage chassis: frame, crankcase/transmission case (the ball sockets), sprung
  axles, wishbone, torque tube, steering, wheels (the gate).
--stage full: engine block, radiator, pedals, hand lever, hub brakes, body,
  mudguards and running boards.
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

COLLECTION = 'VEHICLE | 1909 Ford Model T touring'


def artillery_wheel(name, center, radius, spokes, tire, m, V, axle, hub_half):
    """Twelve-spoke wooden wheel with a clincher tyre."""
    x, y, z = center
    part(torus(f'{name}.tire', center, radius-tire, tire, m['rubber'], V), name, collide=False)
    felloe = radius-2*tire-.02
    part(torus(f'{name}.felloe', center, felloe, .022, m['wood'], V), name, collide=False)
    hub = part(rod(f'{name}.hub', (x, y-hub_half, z), (x, y+hub_half, z), .06, m['iron'], V), name, joins=[axle])
    hub['mounted_on'] = axle
    for i in range(spokes):
        a = math.tau*i/spokes+math.pi/spokes
        part(rod(f'{name}.spoke.{i:02}', (x+.055*math.cos(a), y, z+.055*math.sin(a)),
                 (x+felloe*math.cos(a), y, z+felloe*math.sin(a)), .016, m['wood'], V), name, collide=False)


def ring_on_axis(name, center, radius, minor, direction, mat, V, body, joins=()):
    """Torus whose axis is 'direction', with a polyline centreline for checks."""
    obj = torus(name, center, radius, minor, mat, V, wheel=False)
    obj.rotation_mode = 'QUATERNION'
    obj.rotation_quaternion = Vector(direction).to_track_quat('Z', 'Y')
    obj['centerline'] = [[radius*math.cos(math.tau*i/32), radius*math.sin(math.tau*i/32), 0] for i in range(33)]
    obj['section_radius'] = minor
    obj['capsule'] = True
    return part(obj, body, joins)


def transverse_spring(name, x, half_span, z_end, z_mid, m, V, joins, rest_joins, mates):
    """Transverse semi-elliptic spring drawn at ride height, clamped at its centre under a
    crossmember; its ends rest on perches on the axle. Its flex is the axle joints', so the
    rigid drawn spring mates the axle parts that close on it across the travel."""
    pts = [(x, half_span*(2*t-1), z_end+(z_mid-z_end)*(1-(2*t-1)**2)) for t in [i/10 for i in range(11)]]
    return part(tube(name, pts, .02, m['iron'], V), joins=joins, rest_joins=rest_joins,
                mates=list(rest_joins)+list(mates))


def mudguard(name, center, radius, y, half_width, a0, a1, m, V, joins_first, joins_last):
    """Curved guard as short overlapping boards on an arc."""
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


def build_full(V, p, m):
    wb, rr, fr, rt, ft = p['wheelbase'], p['rear_radius'], p['front_radius'], p['rear_track'], p['front_track']
    fw, h = p['frame_width'], p['rail_height']
    rail_top = h+.05
    # Engine: four cylinders in one block on the crankcase, detachable head.
    part(box('Cylinder block', (2.125, 0, .765), (.55, .26, .27), m['iron'], V, .01), joins=['Crankcase'])
    part(box('Cylinder head', (2.125, 0, .93), (.53, .24, .06), m['iron'], V, .01), joins=['Cylinder block'])
    part(box('Radiator', (2.635, 0, .945), (.10, fw-.04, .55), m['brass'], V, .01),
         joins=['Frame rail -1', 'Frame rail 1'])
    # Three pedals (low, reverse, transmission brake) from the transmission cover, through the floor.
    for i, y in enumerate((.08, 0.0, -.08)):
        part(rod(f'Pedal {i+1}', (1.55, y, .63), (1.50, y, .80), .012, m['steel'], V), joins=['Crankcase'],
             mates=['Floor'])
    # Hand lever (high-speed clutch and hub brakes) at the driver's left, outboard of the rail.
    part(box('Lever bracket', (1.25, fw/2+.075, .60), (.10, .10, .04), m['frame'], V, .004), joins=['Frame rail 1'])
    part(rod('Hand lever', (1.25, fw/2+.10, .62), (1.20, .60, 1.00), .014, m['steel'], V),
         joins=['Lever bracket'], mates=['Floor'])
    # Parking brake: a band on a drum at each rear hub, anchored to the axle housing.
    for s in (-1, 1):
        wheel = f'rear_{"left" if s == 1 else "right"}'
        part(rod(f'{wheel}.carrier', (0, s*.52, rr), (0, s*(rt/2-.055), rr), .05, m['iron'], V), wheel,
             joins=[f'{wheel}.hub'], mates=['Rear axle'])
        part(rod(f'{wheel}.brake_drum', (0, s*.52, rr), (0, s*.58, rr), .12, m['steel'], V), wheel,
             joins=[f'{wheel}.carrier'], mates=['Rear axle', f'Rear brake band {s}'])
        band = torus(f'Rear brake band {s}', (0, s*.55, rr), .126, .006, m['iron'], V)
        band['centerline'] = [[.126*math.cos(math.tau*i/32), .126*math.sin(math.tau*i/32), 0] for i in range(33)]
        band['section_radius'], band['capsule'] = .006, True
        part(band, 'rear_axle', joins=[f'Rear brake anchor {s}'], mates=[f'{wheel}.brake_drum'])
        part(tube(f'Rear brake anchor {s}', [(0, s*.55, rr-.135), (0, s*.55, rr-.15), (0, s*.49, rr-.15),
                                            (0, s*.49, rr-.02)], .008, m['iron'], V), 'rear_axle',
             joins=['Rear axle'])

    # Body: dash, floor, front seat, rear tonneau, bonnet.
    part(box('Dash', (1.80, 0, .92), (.02, 1.10, .50), m['wood'], V, .006), joins=['Frame rail -1', 'Frame rail 1'])
    part(box('Floor', (.9925, 0, rail_top+.0125), (1.585, 1.10, .025), m['wood'], V, .003),
         joins=['Frame rail -1', 'Frame rail 1'])
    for label, xb, xs in (('Front', 1.19, (1.03, 1.35)), ('Rear', .48, (.32, .64))):
        for x in xs:
            for s in (-1, 1):
                part(rod(f'{label} seat support {x}.{s}', (x, s*.30, rail_top+.025), (x, s*.30, .935), .018,
                         m['frame'], V), joins=['Floor', f'{label} seat base'])
        part(box(f'{label} seat base', (xb, 0, .95), (.42 if label == 'Front' else .45, 1.00, .03), m['wood'], V))
        part(box(f'{label} seat cushion', (xb+.005, 0, 1.005), (.38 if label == 'Front' else .41, .96, .08),
                 m['leather'], V, .03), joins=[f'{label} seat base'])
    part(box('Front seat back', (1.00, 0, 1.15), (.06, .96, .38), m['leather'], V, .025),
         joins=['Front seat base', 'Front seat cushion'])
    part(box('Rear seat back', (.27, 0, 1.17), (.06, .96, .42), m['leather'], V, .025),
         joins=['Rear seat base', 'Rear seat cushion'])
    for s in (-1, 1):
        part(box(f'Tonneau side {s}', (.575, s*.54, .8725), (.75, .02, .355), m['paint'], V, .004),
             joins=['Floor', 'Tonneau back'])
        part(box(f'Bonnet side {s}', (2.1975, s*(fw/2-.02), .90), (.775, .012, .46), m['paint'], V, .003),
             joins=[f'Frame rail {s}', 'Dash', 'Radiator', 'Bonnet lid', 'Front spring crossmember'])
    part(box('Tonneau back', (.21, 0, .8725), (.02, 1.10, .355), m['paint'], V, .004), joins=['Floor'])
    part(box('Bonnet lid', (2.1975, 0, 1.14), (.775, fw-.028, .02), m['paint'], V, .004),
         joins=['Dash', 'Radiator'])

    # Mudguards on stays from the rails, and running boards between them.
    guard_r = .50            # clears the tyre at full bump, roll and lock
    for s in (-1, 1):
        for axle_x, zc_w, wy, label, (a0, a1) in ((wb, fr, s*ft/2, 'Front', (32, 155)),
                                                 (0.0, rr, s*rt/2, 'Rear', (25, 155))):
            base = f'{label} mudguard {s}'
            for a, end in ((a1, 'rear'), (a0, 'front')):
                ax = axle_x+guard_r*math.cos(math.radians(a))
                az = zc_w+guard_r*math.sin(math.radians(a))
                part(rod(f'{base} stay {end}', (ax, s*(fw/2+.025), az), (ax, wy-s*.08, az), .012, m['frame'], V),
                     joins=[f'Frame rail {s}'])
            mudguard(base, (axle_x, zc_w), guard_r, wy, .09, a0, a1, m, V,
                     joins_first=[f'{base} stay front'], joins_last=[f'{base} stay rear'])
        part(box(f'Running board {s}', (1.26, s*.54, .45), (1.58, .28, .025), m['black'], V, .004))
        for x in (.80, 1.70):
            part(rod(f'Running board bracket {x}.{s}', (x, s*(fw/2), h-.045), (x, s*.55, .4625), .014,
                     m['frame'], V), joins=[f'Frame rail {s}', f'Running board {s}'])
    return dict(final_drive='torque tube and bevel to the live rear axle', enclosed_rotating_parts=[
        'crankshaft, flywheel magneto and planetary gears in the crankcase', 'drive shaft in the torque tube',
        'half-shafts in the axle housing'])


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
    fw, h, cx, cy = p['frame_width'], p['rail_height'], p['column_x'], p['column_y']
    kp = ft/2-p['kingpin_inset']
    rail_bottom = h-.05
    fpiv, rpiv = Vector(p['front_pivot']), Vector(p['rear_pivot'])

    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    scene = bpy.context.scene
    V = bpy.data.collections.new(COLLECTION)
    studio = bpy.data.collections.new('STUDIO | excluded from export')
    scene.collection.children.link(V)
    scene.collection.children.link(studio)
    m = dict(frame=material('Frame | black enamel', (.03, .03, .03), .2, .4),
             paint=material('Body | Brewster green', (.03, .12, .07), .1, .35),
             black=material('Mudguard | black', (.02, .02, .02), .2, .4),
             wood=material('Varnished hickory', (.36, .20, .08), 0, .35),
             steel=material('Machined steel', (.38, .43, .45), .8, .25),
             brass=material('Radiator brass', (.60, .42, .13), .8, .3),
             iron=material('Cast iron', (.07, .075, .08), .7, .42),
             rubber=material('Clincher tyre', (.017, .021, .023), 0, .7),
             leather=material('Buttoned leather', (.10, .04, .03), 0, .45))

    # --- Channel-steel frame: two rails, end crossmembers, and a crossmember over each spring.
    for s in (-1, 1):
        part(box(f'Frame rail {s}', (1.26, s*fw/2, h), (3.48, .05, .10), m['frame'], V, .006))
    for x in (-.45, 2.97):
        part(box(f'Frame crossmember {x}', (x, 0, h), (.05, fw-.05, .10), m['frame'], V, .006),
             joins=['Frame rail -1', 'Frame rail 1'])
    part(box('Front spring crossmember', (wb, 0, h), (.08, fw-.05, .10), m['frame'], V, .006),
         joins=['Frame rail -1', 'Frame rail 1'])
    # Rear crossmember dropped between the rails; the arched spring clamps under it.
    part(box('Rear spring crossmember', (0, 0, .62), (.08, fw-.05, .07), m['frame'], V, .006),
         joins=['Frame rail -1', 'Frame rail 1'])
    # Crankcase and transmission case: the structural centre carrying both balls' sockets.
    part(box('Crankcase', (1.875, 0, .55), (1.15, .34, .16), m['iron'], V, .01))
    part(rod('Engine front support', (2.44, 0, .60), (2.51, 0, .60), .03, m['iron'], V),
         joins=['Crankcase', 'Front spring crossmember'])
    for s in (-1, 1):
        part(rod(f'Engine arm {s}', (1.95, s*.16, .58), (1.95, s*(fw/2-.01), .60), .02, m['iron'], V),
             joins=['Crankcase', f'Frame rail {s}'])
    part(box('Wishbone socket', (fpiv.x, 0, .45), (.10, .08, .04), m['iron'], V, .004), joins=['Crankcase'])
    # Steering gear bracket outboard of the left rail (the 'Column bearing' the shared linkage expects).
    part(box('Column bearing', (cx, cy, .56), (.12, 2*(cy-(fw/2+.025)), .12), m['iron'], V, .01),
         joins=['Frame rail 1'])

    # --- Front: beam axle, knuckles and linkage (shared double-pivot front), wishbone, spring.
    front = vg.double_pivot_front(V, p, dict(frame=m['iron'], steel=m['steel'], brass=m['brass'], wood=m['wood']),
                                  kp, cx, top=lambda V, x, y, z: None, column_top=.56, cy=cy)
    for name in ('Front beam', 'Axle eye -1', 'Axle eye 1'):
        bpy.data.objects[name]['body'] = 'front_axle'
    part(rod('Wishbone ball', (fpiv.x-.03, 0, fpiv.z), (fpiv.x+.03, 0, fpiv.z), .03, m['steel'], V), 'front_axle',
         joins=['Wishbone socket'], mates=['Wishbone socket'])
    for s in (-1, 1):
        part(rod(f'Wishbone rod {s}', (wb, s*.35, fr), (fpiv.x, s*.01, fpiv.z), .015, m['steel'], V), 'front_axle',
             joins=['Front beam', 'Wishbone ball']+(['Wishbone rod -1'] if s == 1 else []))   # meet in the ball
        part(box(f'Front spring perch {s}', (wb, s*.50, .4165), (.06, .06, .027), m['iron'], V), 'front_axle',
             joins=['Front beam'])
    transverse_spring('Front spring', wb, .50, .45, rail_bottom-.02, m, V, joins=['Front spring crossmember'],
                      rest_joins=['Front spring perch -1', 'Front spring perch 1'], mates=['Front beam'])
    # Raked steering column from the gear bracket to the wheel (4:1 planetary under the wheel).
    top = Vector((cx, cy, .62))
    hub = Vector((1.50, cy, 1.27))
    d = (hub-top).normalized()
    part(rod('Raked column', tuple(top), tuple(hub), .02, m['frame'], V), 'steering_wheel', joins=['Column bearing'],
         mates=['Dash'])   # passes through a hole in the dash
    ring_on_axis('Steering wheel rim', tuple(hub), .19, .014, d, m['wood'], V, 'steering_wheel')
    part(rod('Planetary housing', tuple(hub-d*.03), tuple(hub+d*.02), .045, m['brass'], V), 'steering_wheel',
         joins=['Raked column'])
    e1 = d.orthogonal().normalized()
    e2 = d.cross(e1)
    for i in range(4):
        a = math.tau*i/4
        v = e1*math.cos(a)+e2*math.sin(a)
        part(rod(f'Steering wheel spoke {i}', tuple(hub+v*.04), tuple(hub+v*.19), .01, m['wood'], V),
             'steering_wheel', joins=['Planetary housing', 'Steering wheel rim'])

    # --- Rear: live axle housing, differential, torque tube to its ball, radius rods, spring.
    part(rod('Rear axle', (0, -(rt/2+.07), rr), (0, rt/2+.07, rr), .04, m['iron'], V), 'rear_axle', role='axle')
    part(rod('Differential housing', (0, -.12, rr), (0, .12, rr), .10, m['iron'], V), 'rear_axle',
         joins=['Rear axle'])
    # The ball (centred on the pivot) seats against the transmission case's rear face.
    part(rod('Torque tube', (.05, 0, rr+.004), (rpiv.x-.05, 0, rpiv.z), .035, m['iron'], V), 'rear_axle',
         joins=['Differential housing', 'Torque tube ball'])
    part(rod('Torque tube ball', (rpiv.x-.05, 0, rpiv.z), (rpiv.x+.05, 0, rpiv.z), .05, m['steel'], V), 'rear_axle',
         joins=['Crankcase'], mates=['Crankcase'])
    t_at = lambda x: Vector((.05, 0, rr+.004)).lerp(Vector((rpiv.x-.05, 0, rpiv.z)), (x-.05)/(rpiv.x-.10))
    for s in (-1, 1):
        end = t_at(1.05)
        part(rod(f'Radius rod {s}', (0, s*.40, rr), (end.x, s*.03, end.z), .015, m['steel'], V), 'rear_axle',
             joins=['Rear axle', 'Torque tube'])
        part(box(f'Rear spring perch {s}', (0, s*.45, .436), (.06, .05, .03), m['iron'], V), 'rear_axle',
             joins=['Rear axle'])
    transverse_spring('Rear spring', 0, .45, .471, .565, m, V, joins=['Rear spring crossmember'],
                      rest_joins=['Rear spring perch -1', 'Rear spring perch 1'], mates=['Rear axle'])

    # --- Wheels: 30 in wooden artillery wheels, clincher tyres.
    wheels = []
    for s, side in ((1, 'left'), (-1, 'right')):
        artillery_wheel(f'rear_{side}', (0, s*rt/2, rr), rr, p['rear_spokes'], tire, m, V, 'Rear axle', .06)
        artillery_wheel(f'front_{side}', (wb, s*ft/2, fr), fr, p['front_spokes'], tire, m, V, f'Stub axle {s}', .05)   # 10 mm clear of the knuckle web
        wheels += [dict(name=f'rear_{side}', center=[0, s*rt/2, rr], radius=rr, parent='rear_axle', driven=True),
                   dict(name=f'front_{side}', center=[wb, s*ft/2, fr], radius=fr, parent=f'knuckle_{side}',
                        driven=False)]
    drive = build_full(V, p, m) if args.stage == 'full' else None

    # --- Assembly for the physics adapter.
    bodies = {'chassis': dict(parent=None, origin=[0, 0, 0], joint='free')}
    steer_bodies, loops, steering = vg.steering_bodies(p, kp, cx, front, cy)
    bodies.update(steer_bodies)
    for side in ('left', 'right'):
        bodies[f'knuckle_{side}']['parent'] = 'front_axle'
    bodies['drag_link']['joint'] = dict(type='ball')   # it must tilt as the axle moves
    t, rt_travel, roll = p['front_travel_m'], p['rear_travel_m'], p['roll_travel_deg']
    lf = (Vector((wb, 0, fr))-fpiv).length
    lr = (Vector((0, 0, rr))-rpiv).length
    front_roll_axis = list((Vector((wb, 0, fr))-fpiv).normalized())
    rear_roll_axis = list((rpiv-Vector((0, 0, rr))).normalized())
    bodies['front_axle'] = dict(parent='chassis', origin=[wb, 0, fr], joint=[
        dict(name='swing', type='hinge', axis=[0, 1, 0], pos=list(fpiv), range=[-t/lf, t/lf]),
        dict(name='roll', type='hinge', axis=front_roll_axis, pos=[wb, 0, fr], range_deg=[-roll, roll])])
    bodies['rear_axle'] = dict(parent='chassis', origin=[0, 0, rr], joint=[
        dict(name='swing', type='hinge', axis=[0, 1, 0], pos=list(rpiv), range=[-rt_travel/lr, rt_travel/lr]),
        dict(name='roll', type='hinge', axis=rear_roll_axis, pos=[0, 0, rr], range_deg=[-roll, roll])])
    bodies['steering_wheel'] = dict(parent='chassis', origin=list(top), joint=dict(type='hinge', axis=list(d)))
    for w in wheels:
        bodies[w['name']] = dict(parent=w['parent'], origin=w['center'], joint=dict(type='hinge', axis=[0, 1, 0]),
                                 wheel=dict(radius=w['radius'], half_width=max(tire, .035),
                                            driven=w['driven'], braked=w['driven']))
    couplings = [dict(joint1='steering_wheel_joint', joint2='column_joint', ratio=p['steering_box_ratio'],
                      body1='steering_wheel', body2='column')]
    rad = math.radians(roll)
    suspension = dict(front_axle_origin=[wb, 0, fr], front_swing_joint='front_axle_swing', front_pivot=list(fpiv),
                      front_roll_axis=front_roll_axis, front_roll_joint='front_axle_roll',
                      rear_swing_joint='rear_axle_swing', rear_roll_joint='rear_axle_roll',
                      front_radius_rod_m=lf, radius_rod_m=lr, springs_on=True,
                      # swing > 0 lowers the front axle (it is ahead of its ball) and raises the rear
                      check_poses=[dict(front_swing=-t/lf, rear_swing=rt_travel/lr),
                                   dict(front_swing=t/lf, rear_swing=-rt_travel/lr),
                                   dict(front_roll=rad, rear_roll=rad), dict(front_roll=-rad, rear_roll=-rad)])
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

    box('Ground', (wb/2, 0, -.055), (200, 200, .10), material('Studio limestone', (.19, .215, .22), 0, .8),
        studio, .001)
    bpy.ops.object.camera_add(location=(5.2, -5.6, 3.2))
    camera = bpy.context.object
    for c in list(camera.users_collection):
        c.objects.unlink(camera)
    studio.objects.link(camera)
    mw.aim(camera, (wb*.5, 0, .65))
    camera.data.type = 'ORTHO'
    camera.data.ortho_scale = max(hi[0]-lo[0], hi[1]-lo[1], hi[2]-lo[2])*1.45
    scene.camera = camera
    for name, loc, power in [('Key', (2, -4, 6), 1500), ('Fill', (0, 5, 4), 1000), ('Rim', (-4, -1, 5), 1500)]:
        light = bpy.data.lights.new(name, 'AREA')
        light.energy, light.shape, light.size = power, 'DISK', 4
        obj = bpy.data.objects.new(name, light)
        studio.objects.link(obj)
        obj.location = loc
        mw.aim(obj, (1.1, 0, .7))
    scene.world.color = (.22, .22, .22)
    scene.render.engine = 'CYCLES'
    scene.cycles.samples = 32
    scene.cycles.use_denoising = True
    scene.render.resolution_x, scene.render.resolution_y = 1400, 1050
    args.output.mkdir(parents=True, exist_ok=True)
    scene.render.filepath = str(args.output/'preview.png')
    bpy.ops.wm.save_as_mainfile(filepath=str(args.output/'model_t.blend'))
    report = dict(blender_version=bpy.app.version_string, stage=args.stage, parameters=p, units='meters',
                  collection=COLLECTION, bodies=bodies, loops=loops, steering=steering, drive=drive,
                  couplings=couplings, suspension=suspension,
                  joins=sorted({tuple(j) for j in vg.JOINS}), rest_joins=sorted({tuple(j) for j in vg.REST_JOINS}),
                  vehicle_objects=len(objects), bounds_min=lo, bounds_max=hi, dimensions=[b-a for a, b in zip(lo, hi)],
                  historical_accuracy='approximate study of a 1909 Ford Model T touring; wheelbase, track, wheel '
                                      'size, transverse springs, torque tube, planetary two-speed and brakes are '
                                      'sourced (Wikipedia); part sizes, masses, spring rates, pivots and the '
                                      'steering-gear layout are assumptions')
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    if args.render:
        bpy.ops.render.render(write_still=True)
    print('BUILD_OK', args.stage, args.output, len(objects), 'objects')


if __name__ == '__main__':
    main()
