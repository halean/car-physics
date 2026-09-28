"""Panhard et Levassor (1891) study, 'Systeme Panhard'. Run inside Blender.

X forward, Y left, Z up; meters. Rear axle at x = 0. --stage chassis builds the
wooden frame, axles, the shared double-pivot steering with a tiller, and four
artillery wheels for the downhill gate; --stage full adds the front V-twin,
cone clutch, three-speed gearbox, bevel countershaft with chains, rim-block
brakes and bodywork. Parts are tagged exactly as in experiment 006.
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

COLLECTION = 'VEHICLE | 1891 Panhard et Levassor'


def artillery_wheel(name, center, radius, spokes, tire, m, V, axle, hub_half):
    """Wooden artillery wheel: hub, straight wooden spokes, felloe, solid tyre."""
    x, y, z = center
    part(torus(f'{name}.tire', center, radius-tire, tire, m['rubber'], V), name, collide=False)
    felloe = radius-2*tire-.018
    part(torus(f'{name}.felloe', center, felloe, .022, m['wood'], V), name, collide=False)
    hub = part(rod(f'{name}.hub', (x, y-hub_half, z), (x, y+hub_half, z), .055, m['iron'], V),
               name, joins=[axle])
    hub['mounted_on'] = axle
    for i in range(spokes):
        a = math.tau*i/spokes+math.pi/spokes
        part(rod(f'{name}.spoke.{i:02}', (x+.05*math.cos(a), y, z+.05*math.sin(a)),
                 (x+felloe*math.cos(a), y, z+felloe*math.sin(a)), .015, m['wood'], V), name, collide=False)


def tiller(V, cx, cy, top, m):
    """1891: tiller steering (the steering wheel arrived on a Panhard in 1894)."""
    part(rod('Tiller', (cx, cy, top), (cx-.30, cy, top), .016, m['steel'], V), 'column', joins=['Steering column'])
    part(rod('Tiller grip', (cx-.30, cy-.08, top), (cx-.30, cy+.08, top), .022, m['wood'], V), 'column',
         joins=['Tiller'])


def build_full(V, p, m):
    rr, rt, fw, h = p['rear_radius'], p['rear_track'], p['frame_width'], p['rail_height']
    rail_bottom = h-.05
    chain_r = .006

    def rotating(obj, mates=(), sweep='solid', pitch=None):
        obj['sweep'] = sweep
        obj['mates'] = list(obj.get('mates', []))+list(mates)
        if pitch:
            obj['pitch_radius'] = pitch
        return obj

    # Front engine: Daimler-type V-twin on a crankcase between the rails (flywheels enclosed).
    part(box('Crankcase', (1.33, 0, .52), (.26, .22, .20), m['iron'], V))
    for s in (-1, 1):
        part(rod(f'Engine bearer {s}', (1.33, s*(fw/2-.03), rail_bottom), (1.33, s*.11, .58), .016, m['frame'], V),
             joins=[f'Frame rail {s}', 'Crankcase'])
    for i, (x, lean) in enumerate(((1.27, 1), (1.40, -1))):
        base = (x, 0, .62)
        topp = (x, lean*.28*math.sin(math.radians(10)), .62+.28*math.cos(math.radians(10)))
        part(rod(f'Cylinder {i+1}', base, topp, .06, m['iron'], V), joins=['Crankcase'])
    # Cone clutch and shaft to the gearbox; gearbox; bevel to a transverse countershaft.
    zs = .50
    rotating(part(rod('Clutch shaft', (1.21, 0, zs), (.94, 0, zs), .025, m['steel'], V),
                  joins=['Crankcase', 'Gearbox']), ['Crankcase', 'Gearbox'])
    rotating(part(rod('Clutch cone', (1.06, 0, zs), (1.13, 0, zs), .09, m['steel'], V), joins=['Clutch shaft']))
    part(box('Gearbox', (.775, 0, .50), (.35, .26, .20), m['iron'], V))
    for s in (-1, 1):
        part(rod(f'Gearbox cradle {s}', (.78, s*(fw/2-.03), rail_bottom), (.78, s*.13, .55), .016, m['frame'], V),
             joins=[f'Frame rail {s}', 'Gearbox'])
    xc, zc = .53, .45
    part(rod('Bevel housing', (xc, -.08, zc), (xc, .08, zc), .07, m['iron'], V), joins=['Gearbox'])
    cs_end = rt/2-.05
    rotating(part(rod('Countershaft', (xc, -cs_end, zc), (xc, cs_end, zc), .025, m['steel'], V),
                  joins=['Bevel housing']), ['Bevel housing'])
    chain_y = rt/2-.08
    for s in (-1, 1):
        side = 'left' if s == 1 else 'right'
        part(rod(f'Countershaft hanger {s}', (xc, s*fw/2, rail_bottom), (xc, s*fw/2, zc), .018, m['frame'], V),
             joins=[f'Frame rail {s}', 'Countershaft'], mates=['Countershaft'])
        rotating(part(rod(f'Countershaft sprocket {s}', (xc, s*chain_y-.006, zc), (xc, s*chain_y+.006, zc),
                          .05, m['steel'], V), joins=['Countershaft']), [f'Drive chain {s}'], pitch=.05+chain_r)
        wheel = f'rear_{side}'
        part(rod(f'{wheel}.sprocket_carrier', (0, s*(chain_y-.01), rr), (0, s*(rt/2-.055), rr), .05,
                 m['iron'], V), wheel, joins=[f'{wheel}.hub'], mates=['Rear axle'])
        sprocket = part(rod(f'{wheel}.drive_sprocket', (0, s*chain_y-.006, rr), (0, s*chain_y+.006, rr), .20,
                            m['steel'], V), wheel, joins=[f'{wheel}.sprocket_carrier'], mates=['Rear axle'])
        sprocket['pitch_radius'] = .20+chain_r
        chain = tube(f'Drive chain {s}', mw.loop_points((xc, zc), .05+chain_r, (0, rr), .20+chain_r, s*chain_y),
                     chain_r, m['iron'], V)
        chain['wraps'] = [f'Countershaft sprocket {s}', f'{wheel}.drive_sprocket']
        rotating(part(chain, joins=chain['wraps']), chain['wraps'], sweep='loop')
        # Leather-faced wooden block on the front of each rear tyre, on an arm from a cross shaft.
        part(box(f'Brake block {s}', (rr+.03, s*rt/2, rr), (.04, .05, .10), m['wood'], V, .004),
             joins=[f'Brake arm {s}'])
        part(rod(f'Brake arm {s}', (rr+.05, s*(rt/2-.015), rr), (.62, s*(fw/2+.06), .70), .012, m['iron'], V),
             joins=['Brake cross shaft'])
        part(rod(f'Brake shaft bracket {s}', (.62, s*(fw/2+.03), h+.05), (.62, s*(fw/2+.03), .70), .012,
                 m['iron'], V), joins=[f'Frame rail {s}', 'Brake cross shaft'])
    # The hand lever is clamped outboard of the right arm on the same shaft.
    part(rod('Brake cross shaft', (.62, -(fw/2+.11), .70), (.62, fw/2+.06, .70), .012, m['iron'], V))
    part(rod('Brake lever', (.62, -(fw/2+.10), .70), (.55, -(fw/2+.14), 1.12), .014, m['iron'], V),
         joins=['Brake cross shaft'])
    part(rod('Brake lever grip', (.55, -(fw/2+.14), 1.12), (.54, -(fw/2+.14), 1.20), .02, m['wood'], V),
         joins=['Brake lever'])

    # Body: bench seat behind the gearbox, footboard, dash and a wooden engine box.
    for x in (.08, .44):
        for s in (-1, 1):
            part(rod(f'Seat support {x}.{s}', (x, s*(fw/2-.03), h+.05), (x, s*(fw/2-.03), .92), .016,
                     m['frame'], V), joins=[f'Frame rail {s}', 'Seat base'])
    part(box('Seat base', (.26, 0, .935), (.52, .82, .03), m['wood'], V))
    part(box('Seat cushion', (.275, 0, .99), (.47, .78, .08), m['leather'], V, .03), joins=['Seat base'])
    part(box('Seat back', (.01, 0, 1.14), (.06, .78, .38), m['leather'], V, .025),
         joins=['Seat base', 'Seat cushion'])
    # Behind the brake cross shaft, clear of the off-centre column; rests on the left rail.
    part(box('Footboard', (.83, .065, h+.0625), (.34, .47, .025), m['wood'], V, .003), joins=['Frame rail 1'])
    part(box('Dash', (1.14, 0, .845), (.02, .66, .35), m['wood'], V, .006),
         joins=['Frame rail -1', 'Frame rail 1', 'Engine box side -1', 'Engine box side 1', 'Engine box lid'])
    for s in (-1, 1):
        part(box(f'Engine box side {s}', (1.46, s*(fw/2+.036), .845), (.64, .012, .35), m['wood'], V, .003),
             joins=[f'Frame rail {s}'])
    part(box('Engine box lid', (1.46, 0, 1.03), (.66, .70, .02), m['wood'], V, .004),
         joins=['Engine box side -1', 'Engine box side 1'])
    part(box('Engine box front', (1.785, 0, .845), (.012, .66, .35), m['wood'], V, .003),
         joins=['Engine box side -1', 'Engine box side 1', 'Engine box lid', 'Frame rail -1', 'Frame rail 1'])
    chain_ratio = (.20+chain_r)/(.05+chain_r)
    return dict(chain_ratio=chain_ratio, sweeps=sorted(o.name for o in V.objects if 'sweep' in o))


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
    if not (rr+.05 < h-.05 and fw/2+.03 < kp-.05 and 0 < cx < wb-.3):
        raise ValueError('Rails must sit above the rear axle and inside the steered front wheels')

    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    scene = bpy.context.scene
    V = bpy.data.collections.new(COLLECTION)
    studio = bpy.data.collections.new('STUDIO | excluded from export')
    scene.collection.children.link(V)
    scene.collection.children.link(studio)
    m = dict(frame=material('Frame | ash, black paint', (.03, .03, .03), .1, .45),
             wood=material('Varnished oak', (.30, .15, .06), 0, .35),
             steel=material('Machined steel', (.38, .43, .45), .8, .25),
             brass=material('Warm brass', (.55, .30, .075), .75, .26),
             iron=material('Cast iron', (.07, .075, .08), .7, .42),
             rubber=material('Solid rubber', (.017, .021, .023), 0, .7),
             leather=material('Buttoned leather', (.08, .03, .02), 0, .45))

    # --- Wooden frame: two box-section rails above the rear axle.
    rail_bottom = h-.05
    for s in (-1, 1):
        part(box(f'Frame rail {s}', ((wb+.25-.40)/2, s*fw/2, h), (wb+.25+.40, .06, .10), m['frame'], V, .004))
    for x in (-.35, wb+.20):
        part(box(f'Frame crossmember {x}', (x, 0, h), (.06, fw, .08), m['frame'], V, .004),
             joins=['Frame rail -1', 'Frame rail 1'])
    part(rod('Column bearing', (cx, cy, h-.03), (cx, cy, h+.03), .03, m['iron'], V),
         joins=['Column crossmember -1', 'Column crossmember 1'])
    part(rod('Column crossmember -1', (cx, -(fw/2-.03), h), (cx, cy-.03, h), .018, m['iron'], V),
         joins=['Frame rail -1'])
    part(rod('Column crossmember 1', (cx, cy+.03, h), (cx, fw/2-.03, h), .018, m['iron'], V),
         joins=['Frame rail 1'])
    for s in (-1, 1):
        part(rod(f'Axle mount {s}', (0, s*fw/2, rr), (0, s*fw/2, rail_bottom), .03, m['iron'], V),
             joins=[f'Frame rail {s}', 'Rear axle'])
        part(rod(f'Front hanger {s}', (wb, s*fw/2, rail_bottom), (wb, s*fw/2, fr), .02, m['iron'], V),
             joins=[f'Frame rail {s}', 'Front beam'])
    part(rod('Rear axle', (0, -(rt/2+.08), rr), (0, rt/2+.08, rr), .03, m['steel'], V), role='axle')
    front = vg.double_pivot_front(V, p, dict(frame=m['iron'], steel=m['steel'], brass=m['brass'], wood=m['wood']),
                                  kp, cx, top=lambda V, x, y, z: tiller(V, x, y, z, m), column_top=1.10, cy=cy)

    # --- Wheels: wooden artillery wheels on solid tyres (sizes assumed).
    wheels = []
    for s, side in ((1, 'left'), (-1, 'right')):
        artillery_wheel(f'rear_{side}', (0, s*rt/2, rr), rr, p['rear_spokes'], tire, m, V, 'Rear axle', .06)
        artillery_wheel(f'front_{side}', (wb, s*ft/2, fr), fr, p['front_spokes'], tire, m, V,
                        f'Stub axle {s}', .045)
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
    if args.stage == 'full':
        print('DRIVE_SEATING_OK', mw.check_drive_seating(objects))
    lo = [min((o.matrix_world @ Vector(c))[i] for o in objects for c in o.bound_box) for i in range(3)]
    hi = [max((o.matrix_world @ Vector(c))[i] for o in objects for c in o.bound_box) for i in range(3)]
    for w in wheels:
        t = bpy.data.objects[f'{w["name"]}.tire']
        assert abs(min((t.matrix_world @ Vector(c)).z for c in t.bound_box)) < 1e-5, w['name']

    box('Ground', (wb/2, 0, -.055), (200, 200, .10), material('Studio limestone', (.19, .215, .22), 0, .8),
        studio, .001)
    bpy.ops.object.camera_add(location=(4.2, -4.8, 2.7))
    camera = bpy.context.object
    for c in list(camera.users_collection):
        c.objects.unlink(camera)
    studio.objects.link(camera)
    mw.aim(camera, (wb*.45, 0, .55))
    camera.data.type = 'ORTHO'
    camera.data.ortho_scale = max(hi[0]-lo[0], hi[1]-lo[1], hi[2]-lo[2])*1.5
    scene.camera = camera
    for name, loc, power in [('Key', (1, -3, 5), 1000), ('Fill', (0, 4, 3), 700), ('Rim', (-3, -1, 4), 1000)]:
        light = bpy.data.lights.new(name, 'AREA')
        light.energy, light.shape, light.size = power, 'DISK', 3
        obj = bpy.data.objects.new(name, light)
        studio.objects.link(obj)
        obj.location = loc
        mw.aim(obj, (.8, 0, .6))
    scene.world.color = (.22, .22, .22)
    scene.render.engine = 'CYCLES'
    scene.cycles.samples = 32
    scene.cycles.use_denoising = True
    scene.render.resolution_x, scene.render.resolution_y = 1400, 1050
    args.output.mkdir(parents=True, exist_ok=True)
    scene.render.filepath = str(args.output/'preview.png')
    bpy.ops.wm.save_as_mainfile(filepath=str(args.output/'panhard_1891.blend'))
    report = dict(blender_version=bpy.app.version_string, stage=args.stage, parameters=p, units='meters',
                  collection=COLLECTION, bodies=bodies, loops=loops, steering=steering, drive=drive,
                  joins=sorted({tuple(j) for j in vg.JOINS}), vehicle_objects=len(objects),
                  bounds_min=lo, bounds_max=hi, dimensions=[b-a for a, b in zip(lo, hi)],
                  historical_accuracy='approximate study of the 1891 Panhard et Levassor; layout, clutch, '
                                      'three-speed sliding gearbox, chains, rear rim-block brakes and wooden '
                                      'chassis are sourced, all dimensions and masses are assumptions')
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    if args.render:
        bpy.ops.render.render(write_still=True)
    print('BUILD_OK', args.stage, args.output, len(objects), 'objects')


if __name__ == '__main__':
    main()
