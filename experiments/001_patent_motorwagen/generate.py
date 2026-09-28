"""Run inside Blender. Coordinates: X forward, Y left, Z up; meters."""
import argparse
import json
import math
from pathlib import Path
import sys

import bpy
from mathutils import Vector
from mathutils.geometry import intersect_line_line


def material(name, color, metallic=0, roughness=0.4):
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = (*color, 1)
    mat.use_nodes = True
    shader = mat.node_tree.nodes.get('Principled BSDF')
    shader.inputs['Base Color'].default_value = (*color, 1)
    shader.inputs['Metallic'].default_value = metallic
    shader.inputs['Roughness'].default_value = roughness
    return mat


def finish(obj, name, mat, collection):
    obj.name = name
    for parent in list(obj.users_collection):
        parent.objects.unlink(obj)
    collection.objects.link(obj)
    obj.data.materials.append(mat)
    return obj


def box(name, center, dimensions, mat, collection, bevel=0.012):
    bpy.ops.mesh.primitive_cube_add(size=1, location=center)
    obj = finish(bpy.context.object, name, mat, collection)
    obj.dimensions = dimensions
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    mod = obj.modifiers.new('Soft manufactured edges', 'BEVEL')
    mod.width = bevel
    mod.segments = 3
    obj.modifiers.new('Weighted normals', 'WEIGHTED_NORMAL')
    return obj


def rod(name, a, b, radius, mat, collection):
    direction = Vector(b) - Vector(a)
    bpy.ops.mesh.primitive_cylinder_add(vertices=16, radius=radius,
        depth=direction.length, location=(Vector(a) + Vector(b)) / 2)
    obj = finish(bpy.context.object, name, mat, collection)
    obj['centerline'] = [[0, 0, -direction.length/2], [0, 0, direction.length/2]]
    obj['section_radius'] = radius
    obj.rotation_mode = 'QUATERNION'
    obj.rotation_quaternion = direction.to_track_quat('Z', 'Y')
    for face in obj.data.polygons:
        face.use_smooth = True
    return obj


def tube(name, points, radius, mat, collection):
    curve = bpy.data.curves.new(name, 'CURVE')
    curve.dimensions = '3D'
    curve.bevel_depth = radius
    curve.bevel_resolution = 3
    curve.use_fill_caps = True
    spline = curve.splines.new('POLY')
    spline.points.add(len(points) - 1)
    for point, value in zip(spline.points, points):
        point.co = (*value, 1)
    obj = bpy.data.objects.new(name, curve)
    collection.objects.link(obj)
    obj.data.materials.append(mat)
    obj['centerline'] = [list(point) for point in points]
    obj['section_radius'] = radius
    return obj


def torus(name, center, major, minor, mat, collection, wheel=True):
    bpy.ops.mesh.primitive_torus_add(major_segments=96, minor_segments=12,
        location=center, major_radius=major, minor_radius=minor,
        rotation=(math.pi / 2, 0, 0) if wheel else (0, 0, 0))
    obj = finish(bpy.context.object, name, mat, collection)
    for face in obj.data.polygons:
        face.use_smooth = True
    return obj


def aim(obj, target):
    obj.rotation_euler = (Vector(target) - obj.location).to_track_quat('-Z', 'Y').to_euler()


def check_frame_connectivity(objects):
    """Check swept member contact using finite centerline segments and radii.

    The frame stays an editable assembly, not a Boolean-unioned mesh.
    This detects separated members; it does not certify mechanical joints.
    """
    prefixes = ('Frame ', 'Front fork', 'Front axle', 'Rear axle', 'Steering column',
                'Bench support', 'Bench rear support', 'Leaf spring', 'Spring mount',
                'Axle mount', 'Engine mount', 'Steering bearing', 'Engine bed',
                'Flywheel shaft', 'Flywheel arm', 'Engine output shaft', 'Bevel gear housing',
                'Belt drive shaft', 'Engine pulley', 'Countershaft', 'Differential housing',
                'Brass reservoir')
    members = [obj for obj in objects if obj.name.startswith(prefixes)]

    def point_distance(p, a, b):
        ab = b-a
        t = max(0.0, min(1.0, (p-a).dot(ab)/ab.length_squared))
        return (p-(a+t*ab)).length

    def segment_distance(a, b, c, d):
        distance = min(point_distance(a,c,d), point_distance(b,c,d),
                       point_distance(c,a,b), point_distance(d,a,b))
        closest = intersect_line_line(a,b,c,d)
        if closest:
            p, q = closest
            if (point_distance(p,a,b) < 1e-7 and point_distance(q,c,d) < 1e-7):
                distance = min(distance, (p-q).length)
        return distance

    def box_distance(box, a, b):
        # Sampled segment to oriented box; returns a lower bound on the gap.
        inverse = box.matrix_world.inverted()
        lo = [min(c[i] for c in box.bound_box) for i in range(3)]
        hi = [max(c[i] for c in box.bound_box) for i in range(3)]
        steps = max(1, math.ceil((b-a).length/.002))
        best = math.inf
        for i in range(steps+1):
            local = inverse @ (a+(b-a)*(i/steps))
            nearest = Vector([max(lo[j], min(hi[j], local[j])) for j in range(3)])
            best = min(best, (local-nearest).length)
        return best - (b-a).length/steps/2

    segments = {}
    boxes = [obj for obj in members if 'centerline' not in obj]
    for obj in members:
        if obj in boxes:
            continue
        points = [obj.matrix_world @ Vector(p) for p in obj['centerline']]
        segments[obj.name] = list(zip(points, points[1:]))
    graph = {obj.name: set() for obj in members}
    for i, a in enumerate(members):
        for b in members[i+1:]:
            if a in boxes and b in boxes:
                continue
            if a in boxes or b in boxes:
                box, rod_ = (a, b) if a in boxes else (b, a)
                touching = any(box_distance(box, *seg) <= rod_['section_radius'] + 1e-6
                               for seg in segments[rod_.name])
            else:
                limit = a['section_radius'] + b['section_radius'] + 1e-6
                touching = any(segment_distance(*sa,*sb) <= limit
                               for sa in segments[a.name] for sb in segments[b.name])
            if touching:
                graph[a.name].add(b.name)
                graph[b.name].add(a.name)
    reached = set()
    pending = [members[0].name]
    while pending:
        name = pending.pop()
        if name not in reached:
            reached.add(name)
            pending.extend(graph[name] - reached)
    disconnected = set(graph) - reached
    if disconnected:
        raise ValueError(f'Disconnected frame members: {sorted(disconnected)}')
    return len(members)


def check_wheel_clearance(objects, wheels, tire_radius):
    """Conservative rotating-disk envelope, including tire and spoke thickness.

    Axles intentionally enter the wheel hubs and are excluded. Swept structural
    members are sampled at <=2 mm; subtracting half that step gives a lower
    bound on their surface clearance between samples. This is a static check
    at zero steering angle, not a full steering/suspension travel simulation.
    """
    prefixes = ('Frame ', 'Front fork', 'Steering column', 'Bench support',
                'Bench rear support', 'Leaf spring', 'Spring mount',
                'Axle mount', 'Engine mount', 'Steering bearing', 'Countershaft',
                'Drive chain', 'Drive belt')
    minimum = math.inf
    half_width = max(tire_radius, .030)  # spokes alternate across hub flanges
    for name, x, y, radius in wheels:
        def distance(point):
            radial = max(0.0, math.hypot(point.x-x, point.z-radius)-radius)
            lateral = max(0.0, abs(point.y-y)-half_width)
            return math.hypot(radial, lateral)

        for obj in objects:
            if obj.name.startswith(prefixes):
                points = [obj.matrix_world @ Vector(p) for p in obj['centerline']]
                r = obj['section_radius']
                gap = math.inf
                for a,b in zip(points, points[1:]):
                    steps = max(1, math.ceil((b-a).length/.002))
                    if obj.type == 'CURVE':
                        # Swept round section: a capsule, exact apart from sampling.
                        bound = (b-a).length/steps/2
                        for i in range(steps+1):
                            gap = min(gap, distance(a+(b-a)*(i/steps)) - r - bound)
                        continue
                    # Mesh rods are flat-ended cylinders; a capsule would pad a
                    # short disk (pulley, sprocket) axially by its full radius.
                    # Sample axial rings at <=2 mm (center, half radius and rim).
                    u = (b-a).normalized()
                    e1 = u.orthogonal().normalized()
                    e2 = u.cross(e1)
                    count = max(16, math.ceil(math.tau*r/.002))
                    rings = [(e1*math.cos(math.tau*k/count)+e2*math.sin(math.tau*k/count))
                             for k in range(count)]
                    bound = max((b-a).length/steps, math.tau*r/count)/2
                    for i in range(steps+1):
                        c = a+(b-a)*(i/steps)
                        gap = min(gap, distance(c), *(distance(c+v*f) for v in rings for f in (r, r/2)))
                    gap -= bound
            elif obj.name.startswith('Floor plank'):
                corners = [obj.matrix_world @ Vector(p) for p in obj.bound_box]
                lo = [min(p[i] for p in corners) for i in range(3)]
                hi = [max(p[i] for p in corners) for i in range(3)]
                # Closest point of the conservative plank box to wheel axis.
                closest = Vector([max(lo[i], min(hi[i], v))
                                  for i,v in enumerate((x,y,radius))])
                gap = distance(closest)
            else:
                continue
            if gap < .005:
                raise ValueError(f'Wheel clearance failure: {obj.name} / {name}: {gap:.4f} m')
            minimum = min(minimum, gap)
    return minimum


def loop_points(a, ra, b, rb, y, step=math.radians(8)):
    """Open (uncrossed) belt/chain centerline around two circles in an XZ plane."""
    a, b = Vector((a[0], a[1])), Vector((b[0], b[1]))
    d = (b-a).length
    u = (b-a)/d
    n = Vector((-u.y, u.x))
    s = (ra-rb)/d
    c = math.sqrt(1-s*s)
    upper, lower = u*s+n*c, u*s-n*c

    def arc(center, r, start, end, through):
        t0, t1, tm = (math.atan2(v.y, v.x) for v in (start, end, through))
        sweep = (t1-t0) % math.tau
        if (tm-t0) % math.tau > sweep:  # 'through' lies on the other side
            sweep -= math.tau
        count = max(2, math.ceil(abs(sweep)/step))
        return [center+Vector((math.cos(t0+sweep*i/count), math.sin(t0+sweep*i/count)))*r
                for i in range(count+1)]

    points = arc(a, ra, upper, lower, -u) + arc(b, rb, lower, upper, u)
    points.append(points[0])
    return [(p.x, y, p.y) for p in points]


def check_drive_seating(objects):
    """Belts/chains must lie on their pulleys' pitch circles and faces;
    wheel sprockets must be coaxial with and overlap their hub sleeves."""
    by_name = {obj.name: obj for obj in objects}

    def axis(obj):
        a, b = (obj.matrix_world @ Vector(p) for p in obj['centerline'])
        return a, b

    def polyline_gap(points, line):
        # Largest distance from any point to the other polyline.
        worst = 0.0
        for p in points:
            best = math.inf
            for a, b in zip(line, line[1:]):
                ab = b-a
                t = max(0.0, min(1.0, (p-a).dot(ab)/max(ab.length_squared, 1e-12)))
                best = min(best, (p-(a+ab*t)).length)
            worst = max(worst, best)
        return worst

    checked = 0
    for obj in objects:
        if 'wraps' in obj:
            points = [obj.matrix_world @ Vector(p) for p in obj['centerline']]
            wheels = [by_name[name] for name in obj['wraps']]
            axes = [axis(wheel) for wheel in wheels]
            for (a, b), wheel in zip(axes, wheels):
                if abs((b-a).normalized().y) < 1-1e-6:
                    raise ValueError(f'Drive seating check supports Y-axis pulleys only: {wheel.name}')
                lo, hi = sorted((a.y, b.y))
                if not all(lo-1e-4 <= p.y <= hi+1e-4 for p in points):
                    raise ValueError(f'Drive seating failure: {obj.name} off the face of {wheel.name}')
            # Rebuild the ideal loop on the pulleys' actual axes and pitch circles.
            (a, _), (c, _) = axes
            y = (axes[0][0].y+axes[0][1].y)/2
            ideal = [Vector(p) for p in loop_points((a.x, a.z), wheels[0]['pitch_radius'],
                                                   (c.x, c.z), wheels[1]['pitch_radius'], y, math.radians(2))]
            gap = max(polyline_gap(points, ideal), polyline_gap(ideal, points))
            if gap > .002:
                raise ValueError(f'Drive seating failure: {obj.name} is {gap:.4f} m off its pulley pitch path')
            checked += len(wheels)
        if 'mounted_on' in obj:
            a, b = axis(obj)
            c, d = axis(by_name[obj['mounted_on']])
            u = (d-c).normalized()
            off = max(((p-c)-u*(p-c).dot(u)).length for p in (a, b))
            span = sorted(((a-c).dot(u), (b-c).dot(u)))
            if off > 1e-4 or span[1] < 0 or span[0] > (d-c).length:
                raise ValueError(f'Drive seating failure: {obj.name} not mounted on {obj["mounted_on"]}')
            checked += 1
    return checked


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--render', action='store_true')
    args = parser.parse_args(sys.argv[sys.argv.index('--') + 1:])
    p = json.loads(args.config.read_text())
    expected = {'wheelbase', 'rear_track', 'rear_radius', 'front_radius', 'tire_radius',
                'tube_radius', 'seat_width', 'spokes', 'steering_limit_deg'}
    if set(p) != expected:
        raise ValueError(f'Expected exactly these parameters: {sorted(expected)}')
    for key, value in p.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'{key} must be finite and positive')
    if p['steering_limit_deg'] > 35:
        raise ValueError('Steering limit must not exceed 35 degrees')
    if not isinstance(p['spokes'], int) or not 12 <= p['spokes'] <= 96:
        raise ValueError('spokes must be an integer between 12 and 96')
    wb, track, rr, fr, tire, tr, sw = [p[k] for k in
        ('wheelbase', 'rear_track', 'rear_radius', 'front_radius', 'tire_radius', 'tube_radius', 'seat_width')]
    if not (tire < min(rr, fr) / 8 and tr < 0.05 and sw < track - 4 * tire
            and wb > rr + fr and min(rr, fr) > 0.15):
        raise ValueError('Invalid wheel clearance, tire/tube thickness, radius or seat width')

    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    scene = bpy.context.scene
    scene.unit_settings.system = 'METRIC'
    scene.unit_settings.scale_length = 1
    vehicle = bpy.data.collections.new('VEHICLE | 1886 study')
    studio = bpy.data.collections.new('STUDIO | excluded from export')
    scene.collection.children.link(vehicle)
    scene.collection.children.link(studio)
    green = material('Frame | deep enamel green', (0.025, 0.095, 0.075), 0.65, 0.27)
    steel = material('Machined steel', (0.38, 0.43, 0.45), 0.8, 0.25)
    brass = material('Warm brass', (0.55, 0.30, 0.075), 0.75, 0.26)
    rubber = material('Solid rubber', (0.017, 0.021, 0.023), 0, 0.7)
    wood = material('Oiled timber', (0.29, 0.115, 0.034), 0, 0.4)
    leather = material('Oxblood leather', (0.085, 0.017, 0.014), 0, 0.43)
    dark = material('Engine iron', (0.055, 0.063, 0.065), 0.7, 0.42)

    wheels = [('rear_left', 0, track/2, rr), ('rear_right', 0, -track/2, rr), ('front', wb, 0, fr)]
    for name, x, y, radius in wheels:
        center = (x, y, radius)
        torus(f'{name}.tire', center, radius-tire, tire, rubber, vehicle)
        rim = radius - 2.3*tire
        torus(f'{name}.rim', center, rim, tire*0.45, steel, vehicle)
        rod(f'{name}.hub', (x,y-0.065,radius), (x,y+0.065,radius), 0.041, brass, vehicle)
        for i in range(p['spokes']):
            angle = math.tau*i/p['spokes']
            rod(f'{name}.spoke.{i:02}', (x,y+(-0.026 if i%2 else 0.026),radius),
                (x+rim*math.cos(angle),y,radius+rim*math.sin(angle)), 0.0032, steel, vehicle)

    h = rr + 0.035
    # Crown and steering bearing must clear the TOP of the rotating tire,
    # not merely its axle center. Fork legs stay outside both wheel faces.
    crown_z = max(h + .18, 2*fr + .08 + tr)
    steering_pivot = (wb, 0, crown_z+.045)
    steering_head = (wb-.05, 0, steering_pivot[2])
    fork_offset = max(.065, tire + tr*.8 + .025)

    rail_bend_x = wb*(.39 + .67/2) + .025

    def rail_point(x, side):
        start = Vector((rail_bend_x, side*sw*.38, h))
        if x <= start.x:
            return (x, side*sw*.38, h)
        end = Vector(steering_head)
        return tuple(start + (end-start)*((x-start.x)/(end.x-start.x)))

    rod('Rear axle', (0,-track/2,rr), (0,track/2,rr), 0.027, steel, vehicle)
    for side in (-1,1):
        y = side*sw*0.38
        tube(f'Frame rail {side}', [(-0.46,y,h),(rail_bend_x,y,h),steering_head], tr, green, vehicle)
        rod(f'Axle mount {side}', (0,y,rr), (0,y,h+.025), .032, green, vehicle)
        for x in (-.30,.32):
            rod(f'Spring mount {side}.{x}', (x,y,h), (x,y,h+.13), .015, steel, vehicle)
        rod(f'Bench support {side}', (0.05,y,h), (0.05,y,h+0.33), tr, green, vehicle)
        rod(f'Bench rear support {side}', (-0.28,y,h), (-0.28,y,h+0.33), tr, green, vehicle)
        tube(f'Seat handrail {side}', [(.27,side*(sw/2-.015),h+.32),(.27,side*(sw/2+.025),h+.32),(.27,side*(sw/2+.025),h+.57),
            (-.32,side*(sw/2+.025),h+.57),(-.32,side*(sw/2+.025),h+.32),
            (-.32,side*(sw/2-.015),h+.32)], 0.012, brass, vehicle)
        # Layered leaf-spring silhouette, not a working suspension.
        for layer in range(3):
            tube(f'Leaf spring {side}.{layer}', [(-.30,y,h+.10+layer*.012),
                (0,y,h+.055+layer*.012),(.32,y,h+.10+layer*.012)], .009, dark, vehicle)
    for x in (-.40,.20,wb*.65):
        rod(f'Frame crossmember {x}', rail_point(x,-1),rail_point(x,1),tr,green,vehicle)
    for x in (.20,wb*.65):
        for side in (-1,1):
            a = rail_point(x, side)
            rod(f'Frame floor standoff {x}.{side}', a, (x,a[1],h+.11), tr,green,vehicle)
        rod(f'Frame floor bearer {x}', (x,-sw*.46,h+.11),(x,sw*.46,h+.11),tr,green,vehicle)
    for i in range(9):
        box(f'Floor plank {i}', (wb*.39, (i-4)*sw/10,h+.14), (wb*.67,sw/10-.009,.035),wood,vehicle,.004)
    box('Seat timber base',(-.035,0,h+.32),(.65,sw,.065),wood,vehicle)
    box('Bench cushion',(-.025,0,h+.40),(.60,sw,.12),leather,vehicle,.042)
    box('Bench back',(-.335,0,h+.60),(.095,sw,.37),leather,vehicle,.035)
    for i in range(7):
        # Small upholstery buttons on the forward face of the backrest.
        rod(f'Backrest button {i}',(-.281,(i-3)*sw/8,h+.63),(-.275,(i-3)*sw/8,h+.63),.012,brass,vehicle)
    for side in (-1,1):
        tube(f'Front fork {side}',[(wb,side*fork_offset,fr),(wb,side*fork_offset,crown_z-.04),
            (wb,0,crown_z-.04)],tr*.8,green,vehicle)
    rod('Front axle',(wb,-fork_offset-.02,fr),(wb,fork_offset+.02,fr),.018,steel,vehicle)
    rod('Steering bearing', (wb,0,steering_pivot[2]-.04),
        (wb,0,steering_pivot[2]+.04), .042, green, vehicle)
    tiller_z=max(h+.67,steering_pivot[2]+.30)
    tube('Steering column',[(wb,0,crown_z-.04),(wb,0,tiller_z)],.016,brass,vehicle)
    tube('Tiller',[(wb,0,tiller_z),(wb-.48,0,tiller_z)],.018,green,vehicle)
    rod('Tiller grip',(wb-.48,-.10,tiller_z),(wb-.48,.10,tiller_z),.024,wood,vehicle)

    for side in (-1,1):
        rod(f'Engine mount {side}',(-.40,side*sw*.38,h),(-.40,side*sw*.25,h-.06),tr,green,vehicle)
    box('Engine bed',(-.36,0,h-.07),(.66,sw*.63,.10),green,vehicle)
    rod('Horizontal engine cylinder',(-.48,0,h+.035),(-.11,0,h+.035),.115,dark,vehicle)
    for i in range(8):
        rod(f'Cylinder cooling collar {i}',(-.43+i*.033,0,h+.035),(-.419+i*.033,0,h+.035),.126,steel,vehicle)
    fly = (-.50,0,h+.20)
    torus('Horizontal flywheel',fly,.255,.025,dark,vehicle,wheel=False)
    for i in range(6):
        a=math.tau*i/6
        rod(f'Flywheel arm {i}',fly,(fly[0]+.255*math.cos(a),.255*math.sin(a),fly[2]),.013,steel,vehicle)
    rod('Flywheel shaft',(-.50,0,h-.10),(-.50,0,h+.23),.032,brass,vehicle)
    rod('Brass reservoir',(-.11,.23,h-.02),(-.11,.23,h+.265),.075,brass,vehicle)
    tube('Exhaust pipe',[(-.21,-.11,h+.03),(-.21,-sw*.34,h-.10),(-.62,-sw*.34,h-.10)],.021,dark,vehicle)

    # Drive, after the 1886 layout: flywheel shaft -> bevel gears -> pulley ->
    # uncrossed flat belt -> countershaft pulley beside the differential ->
    # a roller chain each side to sprockets on the rear wheels.
    # Custom properties: 'sweep' marks rotating parts (checked as swept solids),
    # 'mates' lists intended contacts, 'pitch_radius' sets the drive ratio.
    zg, xc, zc, belt_y = h-.24, -.15, h-.30, .225
    chain_y = track/2-.07
    belt_r, chain_r = .008, .006
    def drive(obj, mates, sweep='solid', pitch=None):
        obj['sweep'] = sweep
        obj['mates'] = mates
        if pitch:
            obj['pitch_radius'] = pitch
        return obj
    drive(rod('Engine output shaft',(-.50,0,h-.09),(-.50,0,zg),.025,brass,vehicle),
          ['Flywheel shaft','Engine bed','Bevel gear housing'])
    rod('Bevel gear housing',(-.50,-.06,zg),(-.50,.06,zg),.055,dark,vehicle)
    # Bevel gears join the shafts inside the housing; the shafts themselves stay apart.
    drive(rod('Belt drive shaft',(-.50,.035,zg),(-.50,.25,zg),.02,steel,vehicle),
          ['Bevel gear housing','Engine pulley'])
    drive(rod('Engine pulley',(-.50,belt_y-.025,zg),(-.50,belt_y+.025,zg),.052,steel,vehicle),
          ['Belt drive shaft','Drive belt'],pitch=.052+belt_r)
    drive(rod('Countershaft',(xc,-(track/2-.06),zc),(xc,track/2-.06,zc),.022,steel,vehicle),
          ['Countershaft hanger -1','Countershaft hanger 1','Countershaft pulley',
           'Differential housing','Countershaft sprocket -1','Countershaft sprocket 1'])
    drive(rod('Countershaft pulley',(xc,belt_y-.025,zc),(xc,belt_y+.025,zc),.13,steel,vehicle),
          ['Countershaft','Differential housing','Drive belt'],pitch=.13+belt_r)
    drive(rod('Differential housing',(xc,.08,zc),(xc,belt_y-.025,zc),.08,dark,vehicle),
          ['Countershaft','Countershaft pulley'])
    belt = tube('Drive belt',loop_points((-.50,zg),.052+belt_r,(xc,zc),.13+belt_r,belt_y),
                belt_r,rubber,vehicle)
    # A flat belt is as wide as the pulley face; clearance uses that width.
    drive(belt,['Engine pulley','Countershaft pulley'],sweep='loop')
    belt['envelope_radius'] = .025
    belt['wraps'] = ['Engine pulley','Countershaft pulley']
    for side in (-1,1):
        y = side*chain_y
        rod(f'Countershaft hanger {side}',(xc,side*sw*.38,h),(xc,side*sw*.38,zc),tr,green,vehicle)
        drive(rod(f'Countershaft sprocket {side}',(xc,y-.006,zc),(xc,y+.006,zc),.045,steel,vehicle),
              ['Countershaft',f'Drive chain {side}'],pitch=.045+chain_r)
        wheel = 'rear_left' if side == 1 else 'rear_right'
        sprocket = rod(f'{wheel}.drive_sprocket',(0,y-.006,rr),(0,y+.006,rr),.10,steel,vehicle)
        sprocket['pitch_radius'] = .10+chain_r
        sprocket['mounted_on'] = f'{wheel}.brake_sleeve'
        chain = tube(f'Drive chain {side}',loop_points((xc,zc),.045+chain_r,(0,rr),.10+chain_r,y),
                     chain_r,dark,vehicle)
        drive(chain,[f'Countershaft sprocket {side}',f'{wheel}.drive_sprocket'],sweep='loop')
        chain['wraps'] = [f'Countershaft sprocket {side}',f'{wheel}.drive_sprocket']
    fw = bpy.data.objects['Horizontal flywheel']
    # Spinning flywheel and arms sweep a solid disk about the shaft.
    fw['sweep'] = 'disk'
    fw['sweep_axis'] = [[fly[0],0,fly[2]-.025],[fly[0],0,fly[2]+.025]]
    fw['sweep_radius'] = .255+.025
    fw['mates'] = ['Flywheel shaft']+[f'Flywheel arm {i}' for i in range(6)]
    drive(bpy.data.objects['Flywheel shaft'],['Engine bed','Horizontal flywheel','Engine output shaft',
          'Horizontal engine cylinder']+[f'Flywheel arm {i}' for i in range(6)])

    # Experimental rear drum brakes: rotating drums belong to their wheels;
    # stationary backing plates and linkage attach to the chassis.
    brake_red = material('Brake control | red enamel', (.38,.025,.018), .35, .32)
    brake_radius = min(.13, rr*.26)
    for side, name in ((1,'rear_left'),(-1,'rear_right')):
        wy = side*track/2
        plate_y = wy-side*.1725
        rod(f'{name}.brake_drum', (0,wy-side*.15,rr),
            (0,wy-side*.085,rr), brake_radius, brass, vehicle)
        rod(f'{name}.brake_sleeve', (0,wy-side*.095,rr),
            (0,wy-side*.045,rr), .035, brass, vehicle)
        rod(f'Brake backing plate {side}', (0,wy-side*.18,rr),
            (0,wy-side*.165,rr), brake_radius+.01, dark, vehicle)
        rod(f'Brake anchor {side}',(.12,side*sw*.38,h),
            (.12,plate_y,rr), .012,green,vehicle)
        rod(f'Brake crank {side}',(.08,plate_y,rr+.06),
            (.25,plate_y,h-.06), .012,steel,vehicle)
    lever_y = -sw/2-.045
    rod('Brake cross shaft',(.25,-track/2+.1725,h-.06),
        (.25,track/2-.1725,h-.06), .012,steel,vehicle)
    rod('Brake lever bracket',(.40,-sw*.38,h),(.40,lever_y,h+.04),.016,green,vehicle)
    rod('Brake hand lever',(.40,lever_y,h+.04),(.25,lever_y,h+.56),.016,brake_red,vehicle)
    rod('Brake hand grip',(.25,lever_y,h+.56),(.225,lever_y,h+.65),.024,rubber,vehicle)
    tube('Brake pull rod',[(.40,lever_y,h+.08),(.30,lever_y,h-.06),
         (.25,-track/2+.1725,h-.06)],.006,brass,vehicle)

    bpy.context.view_layer.update()
    rig=bpy.data.objects.new('Steering pivot',None)
    vehicle.objects.link(rig)
    rig.location=steering_pivot
    rig.empty_display_type='ARROWS'
    rig.empty_display_size=.15
    rig['angle_deg']=0.0
    rig['limit_deg']=p['steering_limit_deg']
    rig.id_properties_ui('angle_deg').update(min=-p['steering_limit_deg'],max=p['steering_limit_deg'])
    driver=rig.driver_add('rotation_euler',2).driver
    var=driver.variables.new()
    var.name='angle'
    var.targets[0].id=rig
    var.targets[0].data_path='["angle_deg"]'
    driver.expression=f'max(-{p["steering_limit_deg"]},min({p["steering_limit_deg"]},angle))*pi/180'
    bpy.context.view_layer.update()
    for obj in list(vehicle.objects):
        if obj.name.startswith(('front.','Front fork')) or obj.name in ('Front axle','Steering column','Tiller','Tiller grip'):
            world=obj.matrix_world.copy()
            obj.parent=rig
            obj.matrix_world=world
            obj['assembly']='front' if obj.name.startswith('front.') else 'steering'
    bpy.context.view_layer.update()
    frame_members = check_frame_connectivity(vehicle.objects)
    wheel_gap = check_wheel_clearance(vehicle.objects, wheels, tire)
    drive_seats = check_drive_seating(vehicle.objects)
    bounds = [obj.matrix_world @ Vector(corner) for obj in vehicle.objects if obj.type in {'MESH','CURVE'} for corner in obj.bound_box]
    low = [min(v[i] for v in bounds) for i in range(3)]
    high = [max(v[i] for v in bounds) for i in range(3)]
    assert all(math.isfinite(v) for v in low+high)
    assert len([o for o in vehicle.objects if o.name.endswith('.tire')]) == 3
    for name, x, y, radius in wheels:
        obj = bpy.data.objects[f'{name}.tire']
        bottom = min((obj.matrix_world @ Vector(c)).z for c in obj.bound_box)
        assert abs(bottom) < 1e-5, (name, bottom)
    assert abs(bpy.data.objects['rear_left.tire'].location.y + bpy.data.objects['rear_right.tire'].location.y) < 1e-6

    ground = material('Studio limestone',(0.19,.215,.22),0,.8)
    box('Ground',(wb/3,0,-.055),(200,200,.10),ground,studio,.001)
    bpy.ops.object.camera_add(location=(4.1,-5.3,3.1))
    camera=bpy.context.object
    for collection in list(camera.users_collection):
        collection.objects.unlink(camera)
    studio.objects.link(camera)
    aim(camera,(wb*.31,0,.65))
    camera.data.type='ORTHO'
    camera.data.ortho_scale=max(high[0]-low[0],high[1]-low[1],high[2]-low[2])*1.65
    scene.camera=camera
    for name,location,power,size in [('Key',(1,-3,5),1000,4),('Fill',(0,4,3),800,3),('Rim',(-3,-1,4),1100,3)]:
        data=bpy.data.lights.new(name,'AREA')
        data.energy=power
        data.shape='DISK'
        data.size=size
        obj=bpy.data.objects.new(name,data)
        studio.objects.link(obj)
        obj.location=location
        aim(obj,(.4,0,.6))
    scene.world.color=(.22,.22,.22)
    scene.render.engine='CYCLES'
    scene.cycles.samples=32
    scene.cycles.use_denoising=True
    scene.render.resolution_x=1400
    scene.render.resolution_y=1050
    scene.render.resolution_percentage=100
    scene.render.image_settings.file_format='PNG'
    # Open into a useful camera/material view.
    for screen in bpy.data.screens:
        for area in screen.areas:
            if area.type=='VIEW_3D':
                area.spaces.active.region_3d.view_perspective='CAMERA'
                area.spaces.active.shading.color_type='MATERIAL'
    args.output.mkdir(parents=True,exist_ok=True)
    scene.render.filepath=str(args.output/'preview.png')
    bpy.ops.object.select_all(action='DESELECT')
    for obj in vehicle.objects:
        obj.select_set(True)
    # Export evaluated mesh copies so swept curves are included in GLB.
    originals=list(vehicle.objects)
    bpy.context.view_layer.objects.active=originals[0]
    bpy.ops.object.duplicate()
    bpy.ops.object.convert(target='MESH')
    bpy.ops.export_scene.gltf(filepath=str(args.output/'patent_motorwagen.glb'),export_format='GLB',use_selection=True)
    bpy.ops.object.delete(use_global=False)
    for obj in originals:
        obj.select_set(True)
    bpy.context.view_layer.objects.active=originals[0]
    bpy.ops.wm.save_as_mainfile(filepath=str(args.output/'patent_motorwagen.blend'))
    report={'blender_version':bpy.app.version_string,'parameters':p,'units':'meters',
        'steering_pivot':list(steering_pivot),'steering_limit_deg':p['steering_limit_deg'],
        'vehicle_objects':len(originals),'bounds_min':low,'bounds_max':high,
        'dimensions':[b-a for a,b in zip(low,high)],
        'checks':{'three_wheels':True,'ground_contact':True,'rear_symmetry':True,'finite_bounds':True,
                  'frame_connected':True,'frame_members_checked':frame_members,
                  'frame_wheel_clearance':True,'minimum_wheel_clearance_m':wheel_gap,
                  'drive_seating':True,'drive_seats_checked':drive_seats},
        'historical_accuracy':'approximate visual study; experimental rear brakes are not a historical reconstruction; '
            'belt/countershaft/differential/chain drive follows the 1886 layout with assumed dimensions'}
    (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    if args.render:
        bpy.ops.render.render(write_still=True)
    print('BUILD_OK',args.output)


if __name__=='__main__':
    main()
