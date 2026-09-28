"""Renault Type A Voiturette (1898). Run inside Blender. X forward, Y left, Z up; meters.

Front De Dion-Bouton single, cone clutch, three-speed gearbox with direct-drive
third, propeller shaft with universal joints to a live rear axle with a bevel
differential; handlebar steering; one pedal that declutches and brakes.
--stage chassis builds frame, live-axle housing, steering and wheels for the
gate; --stage full adds the drivetrain, brakes and body. Tagged as in 006.
"""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import sys

import bpy
from mathutils import Vector

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('velo_generate', HERE.parent/'006_benz_velo/generate.py')
vg = importlib.util.module_from_spec(spec)
spec.loader.exec_module(vg)
mw, part, ring = vg.mw, vg.part, vg.ring
material, box, rod, tube, torus = mw.material, mw.box, mw.rod, mw.tube, mw.torus

COLLECTION = 'VEHICLE | 1898 Renault Voiturette'


def handlebar(V, cx, cy, top, m):
    """'Direction par guidon': a crossbar handlebar on the column."""
    part(rod('Handlebar', (cx, cy-.20, top), (cx, cy+.20, top), .012, m['steel'], V), 'column',
         joins=['Steering column'])
    for s in (-1, 1):
        part(rod(f'Handlebar grip {s}', (cx, cy+s*.20, top), (cx, cy+s*.30, top), .018, m['rubber'], V), 'column',
             joins=['Handlebar'])


def build_full(V, p, m):
    rr, rt, fw, h = p['rear_radius'], p['rear_track'], p['frame_width'], p['rail_height']
    za = .34            # crank, clutch, gearbox and propeller shaft height

    def rotating(obj, mates=(), sweep='solid'):
        obj['sweep'] = sweep
        obj['mates'] = list(obj.get('mates', []))+list(mates)
        return obj

    # Front engine: air-cooled vertical single on a crankcase between the rails (flywheels enclosed).
    part(box('Crankcase', (1.08, 0, za), (.20, .18, .18), m['iron'], V))
    part(rod('Cylinder', (1.08, 0, za+.09), (1.08, 0, .74), .055, m['iron'], V), joins=['Crankcase'])
    for i in range(6):
        z = za+.16+i*.045
        part(rod(f'Cooling fin {i}', (1.08, 0, z), (1.08, 0, z+.006), .085, m['iron'], V), joins=['Cylinder'])
    for s in (-1, 1):
        part(rod(f'Engine bearer {s}', (1.08, s*fw/2, h-.02), (1.08, s*.09, za+.05), .014, m['frame'], V),
             joins=[f'Frame rail {s}', 'Crankcase'])
    # Cone clutch, clutch shaft, gearbox (third is direct drive), universal joints, propeller shaft.
    rotating(part(rod('Clutch shaft', (.99, 0, za), (.73, 0, za), .018, m['steel'], V),
                  joins=['Crankcase', 'Gearbox']), ['Crankcase', 'Gearbox'])
    rotating(part(rod('Clutch cone', (.88, 0, za), (.94, 0, za), .075, m['steel'], V), joins=['Clutch shaft']))
    part(box('Gearbox', (.63, 0, za), (.22, .18, .14), m['iron'], V))
    for s in (-1, 1):
        part(rod(f'Gearbox cradle {s}', (.63, s*fw/2, h), (.63, s*.09, za+.03), .014, m['frame'], V),
             joins=[f'Frame rail {s}', 'Gearbox'])
    rotating(part(rod('Gearbox output', (.52, 0, za), (.482, 0, za), .018, m['steel'], V), joins=['Gearbox']),
             ['Gearbox'])
    rotating(part(rod('Front universal joint', (.482, -.028, za), (.482, .028, za), .028, m['steel'], V),
                  joins=['Gearbox output', 'Propeller shaft']))
    # The shafts meet inside each universal joint.
    rotating(part(rod('Propeller shaft', (.482, 0, za), (.14, 0, rr+.005), .016, m['steel'], V),
                  joins=['Gearbox output', 'Pinion shaft']))
    rotating(part(rod('Rear universal joint', (.14, -.028, rr+.005), (.14, .028, rr+.005), .028, m['steel'], V),
                  joins=['Propeller shaft', 'Pinion shaft']))
    rotating(part(rod('Pinion shaft', (.14, 0, rr), (.07, 0, rr), .018, m['steel'], V),
                  joins=['Differential housing']), ['Differential housing'])
    # Rear drum on each wheel, pulled by the pedal (equaliser cables not modelled), band anchored to the housing.
    for s in (-1, 1):
        wheel = 'rear_left' if s == 1 else 'rear_right'
        wy = s*rt/2
        part(rod(f'{wheel}.brake_drum', (0, wy-s*.12, rr), (0, wy-s*.075, rr), .085, m['steel'], V), wheel,
             joins=[f'{wheel}.drum_carrier'], mates=['Rear axle', f'Rear brake band {s}'])
        part(rod(f'{wheel}.drum_carrier', (0, wy-s*.08, rr), (0, wy-s*.035, rr), .035, m['brass'], V), wheel,
             joins=[f'{wheel}.hub'], mates=['Rear axle'])
        band = torus(f'Rear brake band {s}', (0, wy-s*.0975, rr), .091, .006, m['iron'], V)
        band['centerline'] = [[.091*math.cos(math.tau*i/32), .091*math.sin(math.tau*i/32), 0] for i in range(33)]
        band['section_radius'] = .006
        band['capsule'] = True
        part(band, joins=[f'Rear brake anchor {s}'], mates=[f'{wheel}.brake_drum'])
        part(tube(f'Rear brake anchor {s}', [(0, wy-s*.0975, rr-.099), (0, wy-s*.0975, rr-.115),
                                            (0, s*(rt/2-.17), rr-.115), (0, s*(rt/2-.17), rr-.035)], .008,
                  m['iron'], V), joins=['Axle housing'])
    # The single pedal (declutch, then brake) on a cross shaft between the rails.
    part(rod('Pedal shaft', (.76, -fw/2, h-.03), (.76, fw/2, h-.03), .012, m['iron'], V),
         joins=['Frame rail -1', 'Frame rail 1'])
    part(rod('Pedal', (.76, .13, h-.03), (.70, .13, .55), .014, m['iron'], V), joins=['Pedal shaft'])
    part(box('Pedal pad', (.70, .13, .56), (.03, .08, .02), m['rubber'], V, .004), joins=['Pedal'])

    # Body: two-seat over the rear axle, footboard with a column gap, dash, bonnet over the engine.
    for x in (-.05, .30):
        for s in (-1, 1):
            part(rod(f'Seat support {x}.{s}', (x, s*fw/2, h), (x, s*fw/2, .70), .016, m['frame'], V),
                 joins=[f'Frame rail {s}', 'Seat base'])
    part(box('Seat base', (.13, 0, .715), (.50, .80, .03), m['wood'], V))
    part(box('Seat cushion', (.145, 0, .77), (.44, .76, .08), m['leather'], V, .03), joins=['Seat base'])
    part(box('Seat back', (-.105, 0, .92), (.06, .76, .38), m['leather'], V, .025),
         joins=['Seat base', 'Seat cushion'])
    # Stops short of the pedal; notched clear of the off-centre column.
    part(box('Footboard', (.555, .08, h+.0305), (.29, .40, .025), m['wood'], V, .003), joins=['Frame rail 1'])
    part(box('Dash', (.955, 0, .615), (.02, .52, .37), m['wood'], V, .006),
         joins=['Frame rail -1', 'Frame rail 1', 'Bonnet side -1', 'Bonnet side 1', 'Bonnet lid'])
    # Bonnet sides stand on top of the rails, inboard of the steered front wheels.
    for s in (-1, 1):
        part(box(f'Bonnet side {s}', (1.13, s*(fw/2-.008), .615), (.33, .012, .37), m['wood'], V, .003),
             joins=[f'Frame rail {s}'])
    part(box('Bonnet lid', (1.13, 0, .81), (.35, .50, .02), m['wood'], V, .004),
         joins=['Bonnet side -1', 'Bonnet side 1'])
    part(box('Bonnet front', (1.30, 0, .615), (.012, .50, .37), m['wood'], V, .003),
         joins=['Bonnet side -1', 'Bonnet side 1', 'Bonnet lid', 'Frame rail -1', 'Frame rail 1'])
    return dict(final_drive='bevel pinion and crown wheel in the live axle', direct_top_gear=True,
                sweeps=sorted(o.name for o in V.objects if 'sweep' in o))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--stage', choices=('chassis', 'full'), required=True)
    parser.add_argument('--render', action='store_true')
    args = parser.parse_args(sys.argv[sys.argv.index('--')+1:])
    p = json.loads(args.config.read_text())
    wb, ft, rt = p['wheelbase'], p['front_track'], p['rear_track']
    fr, rr, tire, tr = p['front_radius'], p['rear_radius'], p['tire_radius'], p['tube_radius']
    fw, h, cx, cy = p['frame_width'], p['rail_height'], p['column_x'], p['column_y']
    kp = ft/2-p['kingpin_inset']
    if not (rr+.05 < h and fw/2+tr < kp-.05 and 0 < cx < wb-.3):
        raise ValueError('Rails must sit above the axle housing and inside the steered front wheels')

    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    scene = bpy.context.scene
    V = bpy.data.collections.new(COLLECTION)
    studio = bpy.data.collections.new('STUDIO | excluded from export')
    scene.collection.children.link(V)
    scene.collection.children.link(studio)
    m = dict(frame=material('Frame | green enamel', (.03, .10, .07), .5, .35),
             wood=material('Varnished mahogany', (.25, .08, .04), 0, .35),
             steel=material('Machined steel', (.38, .43, .45), .8, .25),
             brass=material('Warm brass', (.55, .30, .075), .75, .26),
             iron=material('Cast iron', (.07, .075, .08), .7, .42),
             rubber=material('Pneumatic tyre', (.017, .021, .023), 0, .7),
             leather=material('Buttoned leather', (.10, .04, .02), 0, .45),
             spoke=material('Wheel | red', (.35, .03, .02), .1, .4))

    # --- Tubular frame above the live-axle housing.
    for s in (-1, 1):
        part(tube(f'Frame rail {s}', [(-.35, s*fw/2, h), (wb+.10, s*fw/2, h)], tr, m['frame'], V))
    for x in (-.30, wb+.08):
        part(rod(f'Frame crossmember {x}', (x, -fw/2, h), (x, fw/2, h), tr, m['frame'], V),
             joins=['Frame rail -1', 'Frame rail 1'])
    part(rod('Column bearing', (cx, cy, h-.03), (cx, cy, h+.03), .03, m['iron'], V),
         joins=['Column crossmember -1', 'Column crossmember 1'])
    part(rod('Column crossmember -1', (cx, -fw/2, h), (cx, cy-.03, h), tr, m['frame'], V), joins=['Frame rail -1'])
    part(rod('Column crossmember 1', (cx, cy+.03, h), (cx, fw/2, h), tr, m['frame'], V), joins=['Frame rail 1'])
    # Live axle: a fixed housing with the bevel differential at its centre; the rotating
    # half-shafts are the 'Rear axle' the hubs turn on (no suspension: housing is chassis).
    part(rod('Axle housing', (0, -(rt/2-.16), rr), (0, rt/2-.16, rr), .035, m['iron'], V),
         joins=['Differential housing', 'Rear axle'])
    part(rod('Differential housing', (0, -.07, rr), (0, .07, rr), .085, m['iron'], V))
    part(rod('Rear axle', (0, -(rt/2+.06), rr), (0, rt/2+.06, rr), .02, m['steel'], V), role='axle',
         joins=['Differential housing'])
    for s in (-1, 1):
        # Stands on the housing's top: the half-shafts spin inside it.
        part(rod(f'Axle mount {s}', (0, s*fw/2, rr+.035), (0, s*fw/2, h+.02), .025, m['frame'], V),
             joins=[f'Frame rail {s}', 'Axle housing'])
        part(rod(f'Front hanger {s}', (wb, s*fw/2, h), (wb, s*fw/2, fr), .02, m['frame'], V),
             joins=[f'Frame rail {s}', 'Front beam'])
    front = vg.double_pivot_front(V, p, dict(frame=m['frame'], steel=m['steel'], brass=m['brass'], wood=m['wood']),
                                  kp, cx, top=lambda V, x, y, z: handlebar(V, x, y, z, m), column_top=.95, cy=cy)

    # --- Wire wheels on pneumatic tyres (sizes assumed).
    wheels = []
    for s, side in ((1, 'left'), (-1, 'right')):
        vg.wheel(f'rear_{side}', (0, s*rt/2, rr), rr, p['rear_spokes'], tire, (m['rubber'], m['spoke'], m['brass']),
                 V, 'Rear axle')
        vg.wheel(f'front_{side}', (wb, s*ft/2, fr), fr, p['front_spokes'], tire, (m['rubber'], m['spoke'],
                 m['brass']), V, f'Stub axle {s}')
        wheels += [dict(name=f'rear_{side}', center=[0, s*rt/2, rr], radius=rr, parent='chassis', driven=True),
                   dict(name=f'front_{side}', center=[wb, s*ft/2, fr], radius=fr, parent=f'knuckle_{side}',
                        driven=False)]
    drive = build_full(V, p, m) if args.stage == 'full' else None

    bodies = {'chassis': dict(parent=None, origin=[0, 0, 0], joint='free')}
    steer_bodies, loops, steering = vg.steering_bodies(p, kp, cx, front, cy)
    bodies.update(steer_bodies)
    for w in wheels:
        bodies[w['name']] = dict(parent=w['parent'], origin=w['center'], joint=dict(type='hinge', axis=[0, 1, 0]),
                                 wheel=dict(radius=w['radius'], half_width=max(tire, .035),
                                            driven=w['driven'], braked=w['driven']))
    bpy.context.view_layer.update()
    objects = [o for o in V.objects if o.type in {'MESH', 'CURVE'}]
    for obj in objects:
        assert 'body' in obj, obj.name
    print('SEATING_OK', mw.check_drive_seating(objects))
    lo = [min((o.matrix_world @ Vector(c))[i] for o in objects for c in o.bound_box) for i in range(3)]
    hi = [max((o.matrix_world @ Vector(c))[i] for o in objects for c in o.bound_box) for i in range(3)]
    for w in wheels:
        t = bpy.data.objects[f'{w["name"]}.tire']
        assert abs(min((t.matrix_world @ Vector(c)).z for c in t.bound_box)) < 1e-5, w['name']

    box('Ground', (wb/2, 0, -.055), (200, 200, .10), material('Studio limestone', (.19, .215, .22), 0, .8),
        studio, .001)
    bpy.ops.object.camera_add(location=(3.4, -3.9, 2.2))
    camera = bpy.context.object
    for c in list(camera.users_collection):
        c.objects.unlink(camera)
    studio.objects.link(camera)
    mw.aim(camera, (wb*.45, 0, .45))
    camera.data.type = 'ORTHO'
    camera.data.ortho_scale = max(hi[0]-lo[0], hi[1]-lo[1], hi[2]-lo[2])*1.5
    scene.camera = camera
    for name, loc, power in [('Key', (1, -3, 5), 1000), ('Fill', (0, 4, 3), 700), ('Rim', (-3, -1, 4), 1000)]:
        light = bpy.data.lights.new(name, 'AREA')
        light.energy, light.shape, light.size = power, 'DISK', 3
        obj = bpy.data.objects.new(name, light)
        studio.objects.link(obj)
        obj.location = loc
        mw.aim(obj, (.6, 0, .5))
    scene.world.color = (.22, .22, .22)
    scene.render.engine = 'CYCLES'
    scene.cycles.samples = 32
    scene.cycles.use_denoising = True
    scene.render.resolution_x, scene.render.resolution_y = 1400, 1050
    args.output.mkdir(parents=True, exist_ok=True)
    scene.render.filepath = str(args.output/'preview.png')
    bpy.ops.wm.save_as_mainfile(filepath=str(args.output/'renault_1898.blend'))
    report = dict(blender_version=bpy.app.version_string, stage=args.stage, parameters=p, units='meters',
                  collection=COLLECTION, bodies=bodies, loops=loops, steering=steering, drive=drive,
                  joins=sorted({tuple(j) for j in vg.JOINS}), vehicle_objects=len(objects),
                  bounds_min=lo, bounds_max=hi, dimensions=[b-a for a, b in zip(lo, hi)],
                  historical_accuracy='approximate study of the 1898 Renault Type A; front engine, three speeds with '
                                      'direct third, cardan shaft to a live axle with differential, handlebar and a '
                                      'combined clutch/brake pedal are sourced; dimensions, masses and brake location '
                                      'are assumptions')
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    if args.render:
        bpy.ops.render.render(write_still=True)
    print('BUILD_OK', args.stage, args.output, len(objects), 'objects')


if __name__ == '__main__':
    main()
