"""Mercedes 35 HP (1901). Run inside Blender. X forward, Y left, Z up; meters.

The first car in the series with suspension: rigid front and rear axles on
semi-elliptic springs. The rear axle is located by radius rods about the
countershaft, so the chains keep their centre distance as it moves. Raked
steering column into a steering box, pressed-steel ladder frame, honeycomb
radiator, front four-cylinder, four speeds, chain drive, hand brake on rear
drums, water-cooled foot brake on the countershaft.
--stage chassis: frame, sprung axles, radius rods, steering, wheels (the gate).
--stage full: engine, radiator, gearbox, chains, both brakes, body, mudguards.
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

COLLECTION = 'VEHICLE | 1901 Mercedes 35 HP'


def artillery_wheel(name, center, radius, spokes, tire, m, V, axle, hub_half):
    """Twelve-spoke wooden wheel with a steel-covered felloe and a pneumatic tyre."""
    x, y, z = center
    part(torus(f'{name}.tire', center, radius-tire, tire, m['rubber'], V), name, collide=False)
    felloe = radius-2*tire-.02
    part(torus(f'{name}.felloe', center, felloe, .025, m['wood'], V), name, collide=False)
    hub = part(rod(f'{name}.hub', (x, y-hub_half, z), (x, y+hub_half, z), .065, m['iron'], V), name, joins=[axle])
    hub['mounted_on'] = axle
    for i in range(spokes):
        a = math.tau*i/spokes+math.pi/spokes
        part(rod(f'{name}.spoke.{i:02}', (x+.06*math.cos(a), y, z+.06*math.sin(a)),
                 (x+felloe*math.cos(a), y, z+felloe*math.sin(a)), .018, m['wood'], V), name, collide=False)


def ring_on_axis(name, center, radius, minor, direction, mat, V, body, joins=()):
    """Torus whose axis is 'direction', with a polyline centreline for checks."""
    obj = torus(name, center, radius, minor, mat, V, wheel=False)
    obj.rotation_mode = 'QUATERNION'
    obj.rotation_quaternion = Vector(direction).to_track_quat('Z', 'Y')
    obj['centerline'] = [[radius*math.cos(math.tau*i/32), radius*math.sin(math.tau*i/32), 0] for i in range(33)]
    obj['section_radius'] = minor
    obj['capsule'] = True
    return part(obj, body, joins)


def leaf_spring(name, x0, x1, y, z_end, z_mid, m, V, joins, rest_joins, mates):
    """Semi-elliptic spring drawn at ride height; its flex is the axle joints'. The drawn
    spring is rigid, so the axle it rides on is a declared mate across the travel."""
    pts = [(x0+(x1-x0)*t, y, z_mid+(z_end-z_mid)*(2*t-1)**2) for t in [i/8 for i in range(9)]]
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


def build_full(V, p, m, susp):
    wb, rr, fr, rt, ft = p['wheelbase'], p['rear_radius'], p['front_radius'], p['rear_track'], p['front_track']
    fw, h = p['frame_width'], p['rail_height']
    xc, zc = p['countershaft']
    rail_top, rail_in = h+.075, fw/2-.025
    chain_r = .006

    def rotating(obj, mates=(), sweep='solid'):
        obj['sweep'] = sweep
        obj['mates'] = list(obj.get('mates', []))+list(mates)
        return obj
    rotating(bpy.data.objects['Countershaft'])   # driven from here on
    # Engine: four cylinders cast in pairs on a crankcase between the rails; flywheel behind.
    part(box('Crankcase', (2.025, 0, .61), (.55, .40, .18), m['iron'], V))   # clears the drag link at full bump
    for i, x in enumerate((1.92, 2.13)):
        part(box(f'Cylinder pair {i+1}', (x, 0, .85), (.20, .20, .30), m['iron'], V, .01), joins=['Crankcase'])
    for x in (1.90, 2.10):   # clear of the raked column
        for s in (-1, 1):
            part(rod(f'Engine bearer {x}.{s}', (x, s*.20, .66), (x, s*rail_in, .70), .02, m['frame'], V),
                 joins=['Crankcase', f'Frame rail {s}'])
    part(box('Honeycomb radiator', (2.42, 0, 1.00), (.10, fw, .50), m['brass'], V, .01),
         joins=['Frame rail -1', 'Frame rail 1'])
    rotating(part(rod('Flywheel', (1.67, 0, .60), (1.73, 0, .60), .22, m['iron'], V), joins=['Clutch shaft']))
    rotating(part(rod('Clutch shaft', (1.76, 0, .60), (1.29, 0, .60), .03, m['steel'], V),
                  joins=['Crankcase', 'Gearbox']), ['Crankcase', 'Gearbox'])
    part(box('Gearbox', (1.075, 0, .58), (.45, .30, .20), m['iron'], V, .01))
    for s in (-1, 1):
        part(rod(f'Gearbox bearer {s}', (1.075, s*.15, .63), (1.075, s*rail_in, .66), .018, m['frame'], V),
             joins=['Gearbox', f'Frame rail {s}'])
    rotating(part(rod('Gearbox output', (.86, 0, .58), (.70, 0, .58), .03, m['steel'], V),
                  joins=['Gearbox', 'Differential housing']), ['Gearbox', 'Differential housing'])
    part(rod('Differential housing', (xc, -.08, zc), (xc, .08, zc), .12, m['iron'], V), joins=['Countershaft'])
    # Water-cooled foot brake: drum on the countershaft, band anchored to the right hanger.
    rotating(part(rod('Foot brake drum', (xc, -.20, zc), (xc, -.14, zc), .13, m['steel'], V),
                  joins=['Countershaft']), ['Foot brake band'])
    band = torus('Foot brake band', (xc, -.17, zc), .136, .006, m['iron'], V)
    band['centerline'] = [[.136*math.cos(math.tau*i/32), .136*math.sin(math.tau*i/32), 0] for i in range(33)]
    band['section_radius'], band['capsule'] = .006, True
    part(band, joins=['Foot brake anchor'], mates=['Foot brake drum'])
    part(rod('Foot brake anchor', (xc, -.17, zc+.144), (xc, -fw/2, zc+.144), .008, m['iron'], V),
         joins=['Frame rail -1'])
    chain_y = rt/2-.10
    for s in (-1, 1):
        side = 'left' if s == 1 else 'right'
        wheel, wy = f'rear_{side}', s*rt/2
        rotating(part(rod(f'Countershaft sprocket {s}', (xc, s*chain_y-.008, zc), (xc, s*chain_y+.008, zc), .10,
                          m['steel'], V), joins=['Countershaft']), [f'Drive chain {s}'])
        bpy.data.objects[f'Countershaft sprocket {s}']['pitch_radius'] = .10+chain_r
        part(rod(f'{wheel}.carrier', (0, s*.44, rr), (0, wy-s*.055, rr), .06, m['iron'], V), wheel,
             joins=[f'{wheel}.hub'], mates=['Rear axle'])
        sprocket = part(rod(f'{wheel}.drive_sprocket', (0, s*chain_y-.008, rr), (0, s*chain_y+.008, rr), .22,
                            m['steel'], V), wheel, joins=[f'{wheel}.carrier'], mates=['Rear axle'])
        sprocket['pitch_radius'] = .22+chain_r
        # Chains ride with the rear axle, which swings about the countershaft.
        chain = tube(f'Drive chain {s}', mw.loop_points((xc, zc), .10+chain_r, (0, rr), .22+chain_r, s*chain_y),
                     chain_r, m['iron'], V)
        chain['wraps'] = [f'Countershaft sprocket {s}', f'{wheel}.drive_sprocket']
        rotating(part(chain, 'rear_axle', joins=chain['wraps']), chain['wraps'], sweep='loop')
        # Hand brake: 30 cm drum on each rear wheel, band anchored to the axle.
        part(rod(f'{wheel}.brake_drum', (0, s*.44, rr), (0, s*.50, rr), .15, m['steel'], V), wheel,
             joins=[f'{wheel}.carrier'], mates=['Rear axle', f'Rear brake band {s}'])
        band = torus(f'Rear brake band {s}', (0, s*.47, rr), .156, .006, m['iron'], V)
        band['centerline'] = [[.156*math.cos(math.tau*i/32), .156*math.sin(math.tau*i/32), 0] for i in range(33)]
        band['section_radius'], band['capsule'] = .006, True
        part(band, 'rear_axle', joins=[f'Rear brake anchor {s}'], mates=[f'{wheel}.brake_drum'])
        part(tube(f'Rear brake anchor {s}', [(0, s*.47, rr-.165), (0, s*.47, rr-.19), (0, s*.40, rr-.19),
                                            (0, s*.40, rr-.02)], .008, m['iron'], V), 'rear_axle',
             joins=[f'Radius rod {s}', 'Rear axle'])   # clamped at the axle/radius-rod junction
    # Driver's controls: hand-brake and gate-change levers at the right, on a rail bracket.
    part(box('Lever bracket', (1.26, -(fw/2+.10), .70), (.16, .15, .04), m['frame'], V, .004),
         joins=['Frame rail -1'])
    for name, x in (('Hand brake lever', 1.22), ('Gear lever', 1.30)):   # outboard of the seat
        part(rod(name, (x, -(fw/2+.17), .70), (x-.08, -(fw/2+.25), 1.15), .014, m['steel'], V),
             joins=['Lever bracket'])

    # Body: dash, floor, front bench, bonnet, four mudguards on stays.
    part(box('Dash', (1.62, 0, .94), (.02, fw+.05, .33), m['wood'], V, .006), joins=['Frame rail -1', 'Frame rail 1'])
    part(box('Floor', (1.01, 0, rail_top+.0125), (.82, fw+.05, .025), m['wood'], V, .003),   # clear of column and stays
         joins=['Frame rail -1', 'Frame rail 1'])
    for x in (.78, 1.12):
        for s in (-1, 1):
            part(rod(f'Seat support {x}.{s}', (x, s*.30, rail_top+.025), (x, s*.30, .93), .018, m['frame'], V),
                 joins=['Floor', 'Seat base'])
    part(box('Seat base', (.95, 0, .945), (.50, 1.00, .03), m['wood'], V))
    part(box('Seat cushion', (.955, 0, 1.00), (.44, .96, .08), m['leather'], V, .03), joins=['Seat base'])
    part(box('Seat back', (.705, 0, 1.15), (.06, .96, .38), m['leather'], V, .025), joins=['Seat base', 'Seat cushion'])
    for s in (-1, 1):
        part(box(f'Bonnet side {s}', (2.0, s*(fw/2-.01), .9375), (.74, .012, .325), m['frame'], V, .003),
             joins=[f'Frame rail {s}', 'Dash', 'Honeycomb radiator', 'Bonnet lid'])
    part(box('Bonnet lid', (2.0, 0, 1.11), (.74, fw, .02), m['frame'], V, .004), joins=['Dash', 'Honeycomb radiator'])
    guard_r = .62            # clears the wheel at full bump plus the steering offset
    for s in (-1, 1):
        for axle_x, zc_w, wy, label in ((wb, fr, s*ft/2, 'Front'), (0.0, rr, s*rt/2, 'Rear')):
            base = f'{label} mudguard {s}'
            a0, a1 = 25, 155   # stays land on the rail's height band at both ends
            for a, end in ((a1, 'rear'), (a0, 'front')):
                ax = axle_x+guard_r*math.cos(math.radians(a))
                az = zc_w+guard_r*math.sin(math.radians(a))
                part(rod(f'{base} stay {end}', (ax, s*(fw/2+.025), az), (ax, wy-s*.09, az), .012, m['frame'], V),
                     joins=[f'Frame rail {s}'])
            mudguard(base, (axle_x, zc_w), guard_r, wy, .10, a0, a1, m, V,
                     joins_first=[f'{base} stay front'], joins_last=[f'{base} stay rear'])
    return dict(chain_ratio=(.22+chain_r)/(.10+chain_r), sweeps=sorted(o.name for o in V.objects if 'sweep' in o))


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
    xc, zc = p['countershaft']
    kp = ft/2-p['kingpin_inset']
    rail_bottom, rail_in = h-.075, fw/2-.025

    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    scene = bpy.context.scene
    V = bpy.data.collections.new(COLLECTION)
    studio = bpy.data.collections.new('STUDIO | excluded from export')
    scene.collection.children.link(V)
    scene.collection.children.link(studio)
    m = dict(frame=material('Pressed steel | Mercedes white', (.72, .72, .70), .3, .35),
             black=material('Mudguard | black', (.02, .02, .02), .2, .4),
             wood=material('Varnished ash', (.33, .16, .06), 0, .35),
             steel=material('Machined steel', (.38, .43, .45), .8, .25),
             brass=material('Honeycomb brass', (.55, .38, .12), .8, .3),
             iron=material('Cast iron', (.07, .075, .08), .7, .42),
             rubber=material('Pneumatic tyre', (.017, .021, .023), 0, .7),
             leather=material('Buttoned leather', (.12, .05, .03), 0, .45))

    # --- Pressed-steel ladder frame (U-section channel drawn as a box).
    for s in (-1, 1):
        # Extends to the dumb irons carrying the spring and mudguard stays.
        part(box(f'Frame rail {s}', (1.175, s*fw/2, h), (3.55, .05, .15), m['frame'], V, .006))
    for x in (-.55, 2.90):
        part(box(f'Frame crossmember {x}', (x, 0, h), (.05, fw-.05, .10), m['frame'], V, .006),
             joins=['Frame rail -1', 'Frame rail 1'])
    # Steering box on the right rail (it is the 'Column bearing' the shared linkage expects).
    part(box('Column bearing', (cx, cy, .62), (.12, 2*(cy+rail_in) if cy > 0 else 2*(rail_in+cy), .12),
             m['iron'], V, .01), joins=['Frame rail -1'])

    # --- Front axle on semi-elliptic springs (the shared double-pivot front, re-parented).
    front = vg.double_pivot_front(V, p, dict(frame=m['iron'], steel=m['steel'], brass=m['brass'], wood=m['wood']),
                                  kp, cx, top=lambda V, x, y, z: None, column_top=.59, cy=cy)
    for name in ('Front beam', 'Axle eye -1', 'Axle eye 1'):
        bpy.data.objects[name]['body'] = 'front_axle'
    for s in (-1, 1):
        part(box(f'Front spring pad {s}', (wb, s*fw/2, fr+.032), (.10, .06, .02), m['iron'], V), 'front_axle',
             joins=['Front beam'])
        leaf_spring(f'Front spring {s}', wb-.45, wb+.45, s*fw/2, rail_bottom+.005, fr+.062, m, V,
                    joins=[f'Frame rail {s}'], rest_joins=[f'Front spring pad {s}'], mates=['Front beam'])
    # Raked steering column and wheel into the box; the box turns the pitman (vertical shaft).
    top = Vector((cx, cy, .68))
    hub = Vector((1.25, cy, 1.25))   # raked back from the box beside the engine; rim clears the seat
    d = (hub-top).normalized()
    part(rod('Raked column', tuple(top), tuple(hub), .02, m['brass'], V), 'steering_wheel', joins=['Column bearing'],
         mates=['Dash'])   # passes through a hole in the dash
    ring_on_axis('Steering wheel rim', tuple(hub), .20, .014, d, m['wood'], V, 'steering_wheel')
    e1 = d.orthogonal().normalized()
    e2 = d.cross(e1)
    part(rod('Steering wheel boss', tuple(hub-d*.02), tuple(hub+d*.02), .03, m['brass'], V), 'steering_wheel',
         joins=['Raked column'])
    for i in range(4):
        a = math.tau*i/4
        v = e1*math.cos(a)+e2*math.sin(a)
        part(rod(f'Steering wheel spoke {i}', tuple(hub+v*.025), tuple(hub+v*.20), .01, m['brass'], V),
             'steering_wheel', joins=['Steering wheel boss', 'Steering wheel rim'])

    # --- Rear: dead axle on springs, located by radius rods pivoting on the countershaft.
    part(rod('Rear axle', (0, -(rt/2+.07), rr), (0, rt/2+.07, rr), .04, m['steel'], V), 'rear_axle', role='axle')
    part(rod('Countershaft', (xc, -(rt/2-.02), zc), (xc, rt/2-.02, zc), .03, m['steel'], V))
    for s in (-1, 1):
        part(rod(f'Countershaft hanger {s}', (xc, s*fw/2, rail_bottom), (xc, s*fw/2, zc), .02, m['frame'], V),
             joins=[f'Frame rail {s}', 'Countershaft'], mates=['Countershaft'])
        # Inboard of the wheel carrier and brake drum, outboard of the spring.
        part(rod(f'Radius rod {s}', (0, s*.40, rr), (xc, s*.40, zc), .02, m['steel'], V), 'rear_axle',
             joins=['Rear axle', 'Countershaft'], mates=['Countershaft'])
        part(box(f'Rear spring pad {s}', (0, s*fw/2, rr+.05), (.10, .06, .02), m['iron'], V), 'rear_axle',
             joins=['Rear axle'])
        leaf_spring(f'Rear spring {s}', -.50, .50, s*fw/2, rail_bottom+.005, rr+.08, m, V,
                    joins=[f'Frame rail {s}'], rest_joins=[f'Rear spring pad {s}'], mates=['Rear axle'])

    # --- Wheels: 910 mm front, 1020 mm rear, twelve wooden spokes, pneumatic tyres.
    wheels = []
    for s, side in ((1, 'left'), (-1, 'right')):
        artillery_wheel(f'rear_{side}', (0, s*rt/2, rr), rr, p['rear_spokes'], tire, m, V, 'Rear axle', .06)
        artillery_wheel(f'front_{side}', (wb, s*ft/2, fr), fr, p['front_spokes'], tire, m, V, f'Stub axle {s}', .06)
        wheels += [dict(name=f'rear_{side}', center=[0, s*rt/2, rr], radius=rr, parent='rear_axle', driven=True),
                   dict(name=f'front_{side}', center=[wb, s*ft/2, fr], radius=fr, parent=f'knuckle_{side}',
                        driven=False)]
    drive = build_full(V, p, m, None) if args.stage == 'full' else None

    # --- Assembly for the physics adapter.
    bodies = {'chassis': dict(parent=None, origin=[0, 0, 0], joint='free')}
    steer_bodies, loops, steering = vg.steering_bodies(p, kp, cx, front, cy)
    bodies.update(steer_bodies)
    for side in ('left', 'right'):
        bodies[f'knuckle_{side}']['parent'] = 'front_axle'
    bodies['drag_link']['joint'] = dict(type='ball')   # it must tilt as the axle moves
    t, rt_travel, roll = p['front_travel_m'], p['rear_travel_m'], p['roll_travel_deg']
    lr = math.hypot(xc, zc-rr)
    bodies['front_axle'] = dict(parent='chassis', origin=[wb, 0, fr], joint=[
        dict(name='heave', type='slide', axis=[0, 0, 1], pos=[wb, 0, fr], range=[-t, t]),
        dict(name='roll', type='hinge', axis=[1, 0, 0], pos=[wb, 0, fr], range_deg=[-roll, roll])])
    bodies['rear_axle'] = dict(parent='chassis', origin=[0, 0, rr], joint=[
        dict(name='swing', type='hinge', axis=[0, 1, 0], pos=[xc, 0, zc], range=[-rt_travel/lr, rt_travel/lr]),
        dict(name='roll', type='hinge', axis=[1, 0, 0], pos=[0, 0, rr], range_deg=[-roll, roll])])
    bodies['steering_wheel'] = dict(parent='chassis', origin=list(top), joint=dict(type='hinge', axis=list(d)))
    for w in wheels:
        bodies[w['name']] = dict(parent=w['parent'], origin=w['center'], joint=dict(type='hinge', axis=[0, 1, 0]),
                                 wheel=dict(radius=w['radius'], half_width=max(tire, .035),
                                            driven=w['driven'], braked=w['driven']))
    couplings = [dict(joint1='steering_wheel_joint', joint2='column_joint', ratio=p['steering_box_ratio'],
                      body1='steering_wheel', body2='column')]
    rad = math.radians(roll)
    suspension = dict(front_axle_origin=[wb, 0, fr], front_heave_joint='front_axle_heave',
                      front_roll_joint='front_axle_roll', rear_swing_joint='rear_axle_swing',
                      rear_roll_joint='rear_axle_roll', radius_rod_m=lr, springs_on=True,
                      check_poses=[dict(front_heave=t, rear_swing=rt_travel/lr),
                                   dict(front_heave=-t, rear_swing=-rt_travel/lr),
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
    bpy.ops.wm.save_as_mainfile(filepath=str(args.output/'mercedes_35hp.blend'))
    report = dict(blender_version=bpy.app.version_string, stage=args.stage, parameters=p, units='meters',
                  collection=COLLECTION, bodies=bodies, loops=loops, steering=steering, drive=drive,
                  couplings=couplings, suspension=suspension,
                  joins=sorted({tuple(j) for j in vg.JOINS}), rest_joins=sorted({tuple(j) for j in vg.REST_JOINS}),
                  vehicle_objects=len(objects), bounds_min=lo, bounds_max=hi, dimensions=[b-a for a, b in zip(lo, hi)],
                  historical_accuracy='approximate study of the 1901 Mercedes 35 HP; dimensions, wheels, weight, '
                                      'engine rating, springs, brakes and chain drive are sourced (Wikipedia citing '
                                      'Eckermann); part sizes, masses, spring rates, ratios and the steering-box '
                                      'layout are assumptions')
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    if args.render:
        bpy.ops.render.render(write_still=True)
    print('BUILD_OK', args.stage, args.output, len(objects), 'objects')


if __name__ == '__main__':
    main()
