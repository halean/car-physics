"""Run in Blender with the generated blend open; tests actual object transforms."""
from pathlib import Path
import importlib.util
import bpy

spec = importlib.util.spec_from_file_location('car_generator', Path(__file__).with_name('generate.py'))
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)
vehicle = bpy.data.collections['VEHICLE | 1886 study']
count = generator.check_frame_connectivity(vehicle.objects)
# Regression: a moved frame member must fail even when its stored centerline
# and all wheel checks are unchanged. Restore it without saving the mutation.
member = bpy.data.objects['Front fork -1']
original = member.location.copy()
try:
    member.location.z += 3
    bpy.context.view_layer.update()
    try:
        generator.check_frame_connectivity(vehicle.objects)
    except ValueError as error:
        assert 'Front fork -1' in str(error)
    else:
        raise AssertionError('Disconnected fork was not detected')
finally:
    member.location = original
    bpy.context.view_layer.update()
print(f'FRAME_CHECK_OK: {count} connected members; detached-fork regression passed')

import json
p = json.loads(Path(bpy.data.filepath).with_name('report.json').read_text())['parameters']
wheels = [('rear_left',0,p['rear_track']/2,p['rear_radius']),
          ('rear_right',0,-p['rear_track']/2,p['rear_radius']),
          ('front',p['wheelbase'],0,p['front_radius'])]
gap = generator.check_wheel_clearance(vehicle.objects,wheels,p['tire_radius'])
# Reproduce the old fork crown inside the wheel's rotating disk.
fork = bpy.data.objects['Front fork -1']
points = [list(point) for point in fork['centerline']]
try:
    bad = [point[:] for point in points]
    bad[-1][2] = p['front_radius']*1.8
    fork['centerline'] = bad
    try:
        generator.check_wheel_clearance(vehicle.objects,wheels,p['tire_radius'])
    except ValueError as error:
        assert 'Front fork -1 / front' in str(error)
    else:
        raise AssertionError('Fork crown inside wheel envelope was not detected')
finally:
    fork['centerline'] = points
print(f'WHEEL_CLEARANCE_OK: minimum conservative gap {gap:.4f} m; low-crown regression passed')

# Rails must pass below/beyond the floor, not emerge through its planks.
import math
from mathutils import Vector
planks = [obj for obj in vehicle.objects if obj.name.startswith('Floor plank')]
for rail in (obj for obj in vehicle.objects if obj.name.startswith('Frame rail')):
    points = [rail.matrix_world @ Vector(p) for p in rail['centerline']]
    for plank in planks:
        corners = [plank.matrix_world @ Vector(p) for p in plank.bound_box]
        lo = [min(p[i] for p in corners) for i in range(3)]
        hi = [max(p[i] for p in corners) for i in range(3)]
        for a,b in zip(points,points[1:]):
            steps = max(1,math.ceil((b-a).length/.002))
            for i in range(steps+1):
                point = a+(b-a)*(i/steps)
                nearest = Vector([max(lo[j],min(hi[j],point[j])) for j in range(3)])
                gap = (point-nearest).length-rail['section_radius']-(b-a).length/steps/2
                assert gap > .005, f'{rail.name} intersects {plank.name}'
print('FLOOR_CLEARANCE_OK: both rails clear all floor planks')

# Drivetrain: seating and support regressions, restored without saving.
seats = generator.check_drive_seating(vehicle.objects)
for name, move, expect, check in [
        ('Drive belt', (0, .03, 0), 'Drive belt', generator.check_drive_seating),
        ('Drive chain 1', (0, 0, .01), 'Drive chain 1', generator.check_drive_seating),
        ('Countershaft hanger 1', (0, 0, 3), 'Countershaft hanger 1', generator.check_frame_connectivity),
        ('Brass reservoir', (0, 0, .10), 'Brass reservoir', generator.check_frame_connectivity)]:
    obj = bpy.data.objects[name]
    original = obj.location.copy()
    try:
        obj.location += Vector(move)
        bpy.context.view_layer.update()
        try:
            check(vehicle.objects)
        except ValueError as error:
            assert expect in str(error), error
        else:
            raise AssertionError(f'Displaced {name} was not detected')
    finally:
        obj.location = original
        bpy.context.view_layer.update()
generator.check_drive_seating(vehicle.objects)
generator.check_frame_connectivity(vehicle.objects)
print(f'DRIVE_CHECK_OK: {seats} belt/chain/sprocket seatings; displaced belt, chain, hanger and reservoir detected')
