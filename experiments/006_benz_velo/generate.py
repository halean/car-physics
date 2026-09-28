"""Benz Velo (1894) study. Run inside Blender. X forward, Y left, Z up; meters.

--stage chassis builds only what the downhill gate needs: frame, axles, the
full double-pivot steering linkage and four wheels. --stage full adds the
drivetrain, brake and bodywork. Every part records its physics body and the
parts it is intentionally joined to; the MuJoCo checks verify each declared
join touches and that every other pair stays apart.
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
spec = importlib.util.spec_from_file_location(
    'motorwagen', HERE.parent/'001_patent_motorwagen/generate.py')
mw = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mw)
material, box, rod, tube, torus = mw.material, mw.box, mw.rod, mw.tube, mw.torus

COLLECTION = 'VEHICLE | 1894 Velo'
JOINS = []


def part(obj, body='chassis', joins=(), role=None, collide=True, mates=()):
    """Tag a part: physics body, collider role, declared joins and moving mates."""
    obj['body'] = body
    obj['collide'] = collide
    if role:
        obj['role'] = role
    for other in joins:
        JOINS.append(sorted((obj.name, other)))
    if mates:
        obj['mates'] = list(mates)
    return obj


def ring(name, center, radius, minor, mat, collection, body, joins=()):
    """Horizontal torus with a polyline centerline so it can be checked and collided."""
    obj = torus(name, center, radius, minor, mat, collection, wheel=False)
    obj['centerline'] = [[radius*math.cos(math.tau*i/32), radius*math.sin(math.tau*i/32), 0]
                         for i in range(33)]
    obj['section_radius'] = minor
    obj['capsule'] = True
    return part(obj, body, joins)


def wheel(name, center, radius, spokes, tire, mats, collection, axle):
    x, y, z = center
    rubber, steel, brass = mats
    part(torus(f'{name}.tire', center, radius-tire, tire, rubber, collection), name, collide=False)
    rim = radius-2.3*tire
    part(torus(f'{name}.rim', center, rim, tire*.45, steel, collection), name, collide=False)
    hub = part(rod(f'{name}.hub', (x, y-.04, z), (x, y+.04, z), .035, brass, collection),
               name, joins=[axle])
    hub['mounted_on'] = axle
    for i in range(spokes):
        a = math.tau*i/spokes
        part(rod(f'{name}.spoke.{i:02}', (x, y+(-.026 if i % 2 else .026), z),
                 (x+rim*math.cos(a), y, z+rim*math.sin(a)), .0028, steel, collection), name, collide=False)
    return hub


def handwheel(V, cx, column_top, brass, wood):
    """Velo column top: a small three-spoke handwheel."""
    ring('Handwheel rim', (cx, 0, column_top), .13, .011, wood, V, 'column')
    part(rod('Handwheel boss', (cx, 0, column_top-.02), (cx, 0, column_top+.015), .025, brass, V), 'column',
         joins=['Steering column'])
    for i in range(3):
        a = math.tau*i/3
        part(rod(f'Handwheel spoke {i}', (cx+.02*math.cos(a), .02*math.sin(a), column_top),
                 (cx+.13*math.cos(a), .13*math.sin(a), column_top), .008, brass, V), 'column',
             joins=['Handwheel boss', 'Handwheel rim'])


def double_pivot_front(V, p, m, kp, cx, top, column_top=1.05, cy=0.0):
    """Beam axle with kingpin knuckles, Jeantaud trapezoid and drag link from a
    central column (Benz double-pivot steering). The column must pass through a
    'Column bearing' supplied by the frame; top(V, x, y, z) adds the driver's control.
    cy moves the column sideways (e.g. clear of a central drive shaft)."""
    wb, ft, fr = p['wheelbase'], p['front_track'], p['front_radius']
    frame_mat, steel, brass = m['frame'], m['steel'], m['brass']
    part(rod('Front beam', (wb, -(kp-.03), fr), (wb, kp-.03, fr), .022, frame_mat, V),
         joins=['Axle eye -1', 'Axle eye 1'])

    # --- Double-pivot steering: kingpin knuckles, trapezoid tie rod aimed at
    # the rear-axle centre (Ackermann/Jeantaud), drag link from the column.
    # Heights: tie rod, drag link (on ball studs below its arms), drag arm, pitman arm.
    zt, zl, zd, zp = .13, .172, .20, .215
    arm = p['steering_arm']
    alpha = math.atan2(kp, wb)
    balls = {}
    for s, side in ((1, 'left'), (-1, 'right')):
        body = f'knuckle_{side}'
        part(rod(f'Axle eye {s}', (wb, s*kp, fr-.025), (wb, s*kp, fr+.025), .03, frame_mat, V))
        web_y = s*(kp+.048)   # 6 mm outside the eye, 10 mm inside the hub
        part(rod(f'Kingpin {s}', (wb, s*kp, zt-.01), (wb, s*kp, fr+.05), .012, steel, V), body,
             joins=[f'Axle eye {s}'])
        for z, name in ((fr+.041, 'upper'), (fr-.041, 'lower')):
            part(rod(f'Knuckle {name} ear {s}', (wb, s*kp, z), (wb, web_y, z), .012, steel, V), body,
                 joins=[f'Kingpin {s}', f'Knuckle web {s}'], mates=[f'Axle eye {s}'])
        part(rod(f'Knuckle web {s}', (wb, web_y, fr-.041), (wb, web_y, fr+.041), .012, steel, V), body)
        part(rod(f'Stub axle {s}', (wb, web_y, fr), (wb, s*(ft/2+.06), fr), .015, steel, V), body,
             role='axle', joins=[f'Knuckle web {s}'])
        ball = (wb-arm*math.cos(alpha), s*(kp-arm*math.sin(alpha)), zt)
        part(rod(f'Steering arm {s}', (wb, s*kp, zt), ball, .011, steel, V), body,
             joins=[f'Kingpin {s}'])
        balls[side] = ball
    drag_ball = (wb, kp-.10, zl)
    part(rod('Drag arm', (wb, kp, zd), (wb, kp-.10, zd), .011, steel, V), 'knuckle_left', joins=['Kingpin 1'])
    part(rod('Drag arm stud', (wb, kp-.10, zd), drag_ball, .008, steel, V), 'knuckle_left',
         joins=['Drag arm'])
    part(rod('Tie rod', balls['left'], balls['right'], .011, steel, V), 'tie_rod',
         joins=['Steering arm 1', 'Steering arm -1'])
    pitman_ball = (cx, cy-.10, zl)  # on the right, so turning the control left steers left
    part(rod('Steering column', (cx, cy, zp), (cx, cy, column_top), .016, brass, V), 'column',
         joins=['Column bearing'])
    part(rod('Pitman arm', (cx, cy, zp), (cx, cy-.10, zp), .011, steel, V), 'column', joins=['Steering column'])
    part(rod('Pitman stud', (cx, cy-.10, zp), pitman_ball, .008, steel, V), 'column', joins=['Pitman arm'])
    part(rod('Drag link', pitman_ball, drag_ball, .011, steel, V), 'drag_link',
         joins=['Pitman stud', 'Drag arm stud'])
    top(V, cx, cy, column_top)
    return dict(balls=balls, drag_ball=drag_ball, pitman_ball=pitman_ball, zd=zd)


def steering_bodies(p, kp, cx, front, cy=0.0):
    """Physics bodies, loop closures and plan geometry of the double-pivot front."""
    wb, fr = p['wheelbase'], p['front_radius']
    balls, drag_ball, pitman_ball = front['balls'], front['drag_ball'], front['pitman_ball']
    limit = p['column_limit_deg']
    bodies = {}
    for side, s in (('left', 1), ('right', -1)):
        bodies[f'knuckle_{side}'] = dict(parent='chassis', origin=[wb, s*kp, fr],
                                         joint=dict(type='hinge', axis=[0, 0, 1]))
    bodies['tie_rod'] = dict(parent='knuckle_left', origin=list(balls['left']),
                             joint=dict(type='hinge', axis=[0, 0, 1]))
    bodies['column'] = dict(parent='chassis', origin=[cx, cy, front['zd']],
                            joint=dict(type='hinge', axis=[0, 0, 1], range_deg=[-limit, limit]))
    bodies['drag_link'] = dict(parent='column', origin=list(pitman_ball),  # at the stud's ball
                               joint=dict(type='hinge', axis=[0, 0, 1]))
    loops = [dict(body1='tie_rod', body2='knuckle_right', point=list(balls['right'])),
             dict(body1='drag_link', body2='knuckle_left', point=list(drag_ball))]
    steering = dict(column=[cx, cy], pitman_ball=list(pitman_ball[:2]), drag_ball=list(drag_ball[:2]),
                    kingpins={'left': [wb, kp], 'right': [wb, -kp]},
                    arm_balls={k: list(v[:2]) for k, v in balls.items()}, column_limit_deg=limit,
                    kingpin_track=2*kp, wheelbase=wb)
    return bodies, loops, steering


def build_full(V, p, m):
    """Drivetrain (two belt speeds, countershaft differential, chains), band brake, bodywork."""
    wb, rt, rr, fw, h = p['wheelbase'], p['rear_track'], p['rear_radius'], p['frame_width'], p['rail_height']
    tr = p['tube_radius']
    belt_r, chain_r = .008, .006
    xe, xc, zs = -.60, -.18, .30          # engine pulley shaft, countershaft, their height
    fly = (xe, 0, .64)

    def rotating(obj, mates=(), sweep='solid', pitch=None):
        obj['sweep'] = sweep
        obj['mates'] = list(obj.get('mates', []))+list(mates)
        if pitch:
            obj['pitch_radius'] = pitch
        return obj

    # Engine: horizontal cylinder ahead of the crank; horizontal flywheel above the frame.
    part(rod('Engine cylinder', (-.55, 0, .45), (-.33, 0, .45), .10, m['iron'], V), joins=['Crankcase'])
    part(box('Crankcase', (-.60, 0, .45), (.12, .18, .14), m['iron'], V))
    for s in (-1, 1):
        part(rod(f'Engine mount {s}', (-.44, s*fw/2, h), (-.44, s*.09, .45), .014, m['frame'], V),
             joins=[f'Frame rail {s}', 'Engine cylinder'])
        part(rod(f'Crankcase hanger {s}', (-.55, s*fw/2, h), (-.60, s*.085, .50), .014, m['frame'], V),
             joins=[f'Frame rail {s}', 'Crankcase'])
    rotating(part(rod('Flywheel shaft', (xe, 0, zs), (xe, 0, fly[2]+.03), .028, m['brass'], V),
                  joins=['Crankcase', 'Bevel gear housing']), mates=['Crankcase'])
    fw_rim = part(torus('Flywheel', fly, .18, .022, m['iron'], V, wheel=False))
    fw_rim['centerline'] = [[.18*math.cos(math.tau*i/32), .18*math.sin(math.tau*i/32), 0] for i in range(33)]
    fw_rim['section_radius'] = .022
    fw_rim['capsule'] = True
    rotating(fw_rim, ['Flywheel sweep'])
    rotating(part(rod('Flywheel boss', (fly[0], 0, fly[2]-.025), (fly[0], 0, fly[2]+.025), .045, m['iron'], V),
                  joins=['Flywheel shaft']), ['Flywheel sweep'])
    for i in range(6):
        a = math.tau*i/6
        rotating(part(rod(f'Flywheel arm {i}', (fly[0]+.04*math.cos(a), .04*math.sin(a), fly[2]),
                          (fly[0]+.18*math.cos(a), .18*math.sin(a), fly[2]), .012, m['steel'], V),
                      joins=['Flywheel boss', 'Flywheel']), ['Flywheel sweep'])
    # Invisible swept solid of the spinning flywheel and arms.
    env = rod('Flywheel sweep', (fly[0], 0, fly[2]-.022), (fly[0], 0, fly[2]+.022), .202, m['iron'], V)
    env.hide_render = True
    env.display_type = 'WIRE'
    env['envelope'] = True
    rotating(part(env, joins=['Flywheel shaft']), ['Flywheel']+[f'Flywheel arm {i}' for i in range(6)]
             + ['Flywheel shaft', 'Flywheel boss'])

    # Bevel gears to a transverse pulley shaft; two belt speeds.
    part(rod('Bevel gear housing', (xe, -.05, zs), (xe, .05, zs), .05, m['iron'], V))
    rotating(part(rod('Belt drive shaft', (xe, .035, zs), (xe, .225, zs), .02, m['steel'], V),
                  joins=['Bevel gear housing']), ['Bevel gear housing'])
    speeds = dict(high=(.14, .10, .115), low=(.20, .06, .137))   # belt plane y, engine r, countershaft r
    for name, (y, re, rc) in speeds.items():
        rotating(part(rod(f'Engine pulley {name}', (xe, y-.025, zs), (xe, y+.025, zs), re, m['steel'], V),
                      joins=['Belt drive shaft']), [f'Drive belt {name}'], pitch=re+belt_r)
        rotating(part(rod(f'Countershaft pulley {name}', (xc, y-.025, zs), (xc, y+.025, zs), rc, m['steel'], V),
                      joins=['Countershaft']), [f'Drive belt {name}'], pitch=rc+belt_r)
        belt = tube(f'Drive belt {name}', mw.loop_points((xe, zs), re+belt_r, (xc, zs), rc+belt_r, y),
                    belt_r, m['rubber'], V)
        belt['envelope_radius'] = .025          # flat belt face width
        belt['wraps'] = [f'Engine pulley {name}', f'Countershaft pulley {name}']
        rotating(part(belt, joins=belt['wraps']), belt['wraps'], sweep='loop')   # held by its pulleys
    cs_end = rt/2-.05
    rotating(part(rod('Countershaft', (xc, -cs_end, zs), (xc, cs_end, zs), .022, m['steel'], V)))
    rotating(part(rod('Differential housing', (xc, -.10, zs), (xc, .10, zs), .08, m['iron'], V),
                  joins=['Countershaft']))
    # Band brake: drum on the countershaft; stationary band anchored to a hanger.
    rotating(part(rod('Brake drum', (xc, -.165, zs), (xc, -.115, zs), .10, m['steel'], V),
                  joins=['Countershaft']), ['Brake band'])
    band = torus('Brake band', (xc, -.14, zs), .106, .006, m['iron'], V)
    band['centerline'] = [[.106*math.cos(math.tau*i/32), .106*math.sin(math.tau*i/32), 0] for i in range(33)]
    band['section_radius'] = .006
    band['capsule'] = True
    part(band, joins=['Brake band anchor'], mates=['Brake drum'])
    part(rod('Brake band anchor', (xc, -.14, zs+.114), (xc, -fw/2, zs+.114), .008, m['iron'], V),
         joins=['Countershaft hanger -1'])
    chain_y = rt/2-.06
    for s in (-1, 1):
        side = 'left' if s == 1 else 'right'
        part(rod(f'Countershaft hanger {s}', (xc, s*fw/2, h), (xc, s*fw/2, zs), tr, m['frame'], V),
             joins=[f'Frame rail {s}', 'Countershaft'], mates=['Countershaft'])
        rotating(part(rod(f'Countershaft sprocket {s}', (xc, s*chain_y-.006, zs), (xc, s*chain_y+.006, zs),
                          .045, m['steel'], V), joins=['Countershaft']), [f'Drive chain {s}'],
                 pitch=.045+chain_r)
        wheel = f'rear_{side}'
        carrier = part(rod(f'{wheel}.sprocket_carrier', (0, s*.40, rr), (0, s*(rt/2-.035), rr),
                           .045, m['brass'], V), wheel, joins=[f'{wheel}.hub'], mates=['Rear axle'])
        # Hand brake: drum cast with the sprocket, a contracting band anchored to the axle mount.
        part(rod(f'{wheel}.brake_drum', (0, s*.40, rr), (0, s*.448, rr), .12, m['steel'], V), wheel,
             joins=[f'{wheel}.sprocket_carrier'], mates=['Rear axle', f'Rear brake band {s}'])
        band = torus(f'Rear brake band {s}', (0, s*.424, rr), .126, .006, m['iron'], V)
        band['centerline'] = [[.126*math.cos(math.tau*i/32), .126*math.sin(math.tau*i/32), 0] for i in range(33)]
        band['section_radius'] = .006
        band['capsule'] = True
        part(band, joins=[f'Rear brake anchor {s}'], mates=[f'{wheel}.brake_drum'])
        # Up from the band, across above the drum, down onto the axle mount.
        part(tube(f'Rear brake anchor {s}', [(0, s*.424, rr+.134), (0, s*.424, rr+.155),
                                             (0, s*fw/2, rr+.155), (0, s*fw/2, rr+.10)], .008, m['iron'], V),
             joins=[f'Axle mount {s}', f'Frame rail {s}'])   # bolted at the rail/mount junction
        sprocket = part(rod(f'{wheel}.drive_sprocket', (0, s*chain_y-.006, rr), (0, s*chain_y+.006, rr),
                            .15, m['steel'], V), wheel, joins=[f'{wheel}.sprocket_carrier'], mates=['Rear axle'])
        sprocket['pitch_radius'] = .15+chain_r
        chain = tube(f'Drive chain {s}', mw.loop_points((xc, zs), .045+chain_r, (0, rr), .15+chain_r, s*chain_y),
                     chain_r, m['iron'], V)
        chain['wraps'] = [f'Countershaft sprocket {s}', f'{wheel}.drive_sprocket']
        rotating(part(chain, joins=chain['wraps']), chain['wraps'], sweep='loop')

    # Bodywork: bench seat, split footboard around the column, dash, panelled engine bonnet.
    for x in (.06, .40):   # clear of the rear axle mounts at x = 0
        for s in (-1, 1):
            part(rod(f'Seat support {x}.{s}', (x, s*fw/2, h), (x, s*fw/2, .78), tr, m['frame'], V),
                 joins=[f'Frame rail {s}', 'Seat base'])
    part(box('Seat base', (.20, 0, .795), (.56, .92, .03), m['wood'], V))
    part(box('Seat cushion', (.22, 0, .85), (.48, .88, .08), m['leather'], V, .03), joins=['Seat base'])
    part(box('Seat back', (-.05, 0, 1.00), (.06, .88, .38), m['leather'], V, .025),
         joins=['Seat base', 'Seat cushion'])
    cx = p['column_x']
    for s in (-1, 1):
        part(box(f'Footboard {s}', (.85, s*.1675, h+tr+.0125), (.70, .265, .025), m['wood'], V, .003),
             joins=[f'Frame rail {s}', f'Column crossmember {s}'])
    part(box('Dash', (1.21, 0, .66), (.02, .60, .26), m['wood'], V, .006), joins=['Footboard -1', 'Footboard 1'])
    part(box('Bonnet lid', (-.575, 0, .73), (.55, .56, .02), m['wood'], V, .004),
         joins=['Bonnet side -1', 'Bonnet side 1', 'Bonnet back'])
    for s in (-1, 1):
        # Side panels overlap the rail's outer face by 1 mm: fastened to it.
        part(box(f'Bonnet side {s}', (-.575, s*(fw/2+tr+.005), .61), (.55, .012, .22), m['wood'], V, .003),
             joins=[f'Frame rail {s}'])
    part(box('Bonnet back', (-.855, 0, .61), (.012, .56, .22), m['wood'], V, .003),
         joins=['Bonnet side -1', 'Bonnet side 1'])

    ratio = lambda y_name: (bpy.data.objects[f'Countershaft pulley {y_name}']['pitch_radius'] /
                            bpy.data.objects[f'Engine pulley {y_name}']['pitch_radius'])
    chain_ratio = (.15+chain_r)/(.045+chain_r)
    return dict(chain_ratio=chain_ratio, gears={g: dict(belt_ratio=ratio(g), overall_ratio=ratio(g)*chain_ratio)
                                                for g in speeds},
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
    fw, h, cx = p['frame_width'], p['rail_height'], p['column_x']
    kp = ft/2-p['kingpin_inset']          # kingpin lateral position
    if not (rr+.03 < h and fr+.1 < h and fw/2+tr < kp-.03 and 0 < cx < wb-.2):
        raise ValueError('Rails must clear the rear axle and front wheels; column inside wheelbase')

    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    scene = bpy.context.scene
    vehicle = bpy.data.collections.new(COLLECTION)
    studio = bpy.data.collections.new('STUDIO | excluded from export')
    scene.collection.children.link(vehicle)
    scene.collection.children.link(studio)
    frame_mat = material('Frame | carriage black', (.02, .02, .022), .5, .35)
    steel = material('Machined steel', (.38, .43, .45), .8, .25)
    brass = material('Warm brass', (.55, .30, .075), .75, .26)
    rubber = material('Solid rubber', (.017, .021, .023), 0, .7)
    wood = material('Varnished ash', (.33, .16, .06), 0, .35)
    red = material('Wheel | carriage red', (.35, .03, .02), .1, .4)
    V = vehicle

    # --- Frame: two rails above the rear axle, front hangers to the beam axle.
    for s in (-1, 1):
        part(tube(f'Frame rail {s}', [(-.55, s*fw/2, h), (wb+.08, s*fw/2, h)], tr, frame_mat, V),
             joins=[])
    for x in (-.28, .20, wb+.06):   # rear crossmember clears the engine cylinder
        part(rod(f'Frame crossmember {x}', (x, -fw/2, h), (x, fw/2, h), tr, frame_mat, V),
             joins=['Frame rail -1', 'Frame rail 1'])
    # Column bearing splits the crossmember at the column.
    part(rod('Column bearing', (cx, 0, h-.03), (cx, 0, h+.03), .03, frame_mat, V),
         joins=['Column crossmember -1', 'Column crossmember 1'])
    for s in (-1, 1):
        part(rod(f'Column crossmember {s}', (cx, s*fw/2, h), (cx, s*.03, h), tr, frame_mat, V),
             joins=[f'Frame rail {s}'])
        part(rod(f'Axle mount {s}', (0, s*fw/2, rr), (0, s*fw/2, h+.025), .03, frame_mat, V),
             joins=[f'Frame rail {s}', 'Rear axle'])
        part(rod(f'Front hanger {s}', (wb, s*fw/2, h), (wb, s*fw/2, fr), .02, frame_mat, V),
             joins=[f'Frame rail {s}', 'Front beam'])
    part(rod('Rear axle', (0, -(rt/2+.06), rr), (0, rt/2+.06, rr), .027, steel, V), role='axle')
    front = double_pivot_front(V, p, dict(frame=frame_mat, steel=steel, brass=brass, wood=wood), kp, cx,
                               top=lambda V, x, y, z: handwheel(V, x, z, brass, wood))
    balls, drag_ball, pitman_ball, zd = front['balls'], front['drag_ball'], front['pitman_ball'], front['zd']

    # --- Wheels: 850 mm rear, 550 mm front wire wheels on solid tyres.
    wheels = []
    for s, side in ((1, 'left'), (-1, 'right')):
        wheel(f'rear_{side}', (0, s*rt/2, rr), rr, p['rear_spokes'], tire, (rubber, red, brass), V, 'Rear axle')
        wheel(f'front_{side}', (wb, s*ft/2, fr), fr, p['front_spokes'], tire, (rubber, red, brass), V,
              f'Stub axle {s}')
        wheels += [dict(name=f'rear_{side}', center=[0, s*rt/2, rr], radius=rr, parent='chassis',
                        driven=True, braked=True),
                   dict(name=f'front_{side}', center=[wb, s*ft/2, fr], radius=fr, parent=f'knuckle_{side}',
                        driven=False, braked=False)]

    drive = None
    if args.stage == 'full':
        drive = build_full(V, p, dict(frame=frame_mat, steel=steel, brass=brass, rubber=rubber, wood=wood,
                                      iron=material('Engine iron', (.055, .063, .065), .7, .42),
                                      leather=material('Black leather', (.03, .025, .022), 0, .45)))

    # --- Assembly description for the physics adapter (rest pose = straight ahead).
    bodies = {'chassis': dict(parent=None, origin=[0, 0, 0], joint='free')}
    steer_bodies, loops, steering = steering_bodies(p, kp, cx, front)
    bodies.update(steer_bodies)
    for w in wheels:
        bodies[w['name']] = dict(parent=w['parent'], origin=w['center'],
                                 joint=dict(type='hinge', axis=[0, 1, 0]),
                                 wheel=dict(radius=w['radius'], half_width=max(tire, .03),
                                            driven=w['driven'], braked=w['braked']))

    bpy.context.view_layer.update()
    objects = [o for o in V.objects if o.type in {'MESH', 'CURVE'}]
    for obj in objects:
        assert 'body' in obj, obj.name
    lo = [min((o.matrix_world @ Vector(c))[i] for o in objects for c in o.bound_box) for i in range(3)]
    hi = [max((o.matrix_world @ Vector(c))[i] for o in objects for c in o.bound_box) for i in range(3)]
    for w in wheels:
        tire_obj = bpy.data.objects[f'{w["name"]}.tire']
        bottom = min((tire_obj.matrix_world @ Vector(c)).z for c in tire_obj.bound_box)
        assert abs(bottom) < 1e-5, (w['name'], bottom)

    ground = material('Studio limestone', (.19, .215, .22), 0, .8)
    box('Ground', (wb/2, 0, -.055), (200, 200, .10), ground, studio, .001)
    bpy.ops.object.camera_add(location=(3.6, -4.4, 2.4))
    camera = bpy.context.object
    for c in list(camera.users_collection):
        c.objects.unlink(camera)
    studio.objects.link(camera)
    mw.aim(camera, (wb*.45, 0, .45))
    camera.data.type = 'ORTHO'
    camera.data.ortho_scale = max(hi[0]-lo[0], hi[1]-lo[1], hi[2]-lo[2])*1.55
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
    bpy.ops.wm.save_as_mainfile(filepath=str(args.output/'benz_velo.blend'))
    if args.stage == 'full':
        seats = mw.check_drive_seating(objects)   # belts, chains, sprockets and hubs
        print('DRIVE_SEATING_OK', seats)
    report = dict(blender_version=bpy.app.version_string, stage=args.stage, parameters=p, units='meters', drive=drive,
                  collection=COLLECTION, bodies=bodies, loops=loops, steering=steering,
                  joins=sorted({tuple(j) for j in JOINS}), vehicle_objects=len(objects),
                  bounds_min=lo, bounds_max=hi, dimensions=[b-a for a, b in zip(lo, hi)],
                  historical_accuracy='approximate study of the 1894 Benz Velo; dimensions from secondary sources, '
                                      'linkage details, frame and part sizes are assumptions')
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    if args.render:
        bpy.ops.render.render(write_still=True)
    print('BUILD_OK', args.stage, args.output, len(objects), 'objects')


if __name__ == '__main__':
    main()
