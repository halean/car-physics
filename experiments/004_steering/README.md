# 004 — Tiller steering: travel clearance and steered descent

Experiment 001 now builds the front fork, front wheel, steering column and tiller
as one assembly parented to a `Steering pivot` empty on the steering bearing
axis. Its `angle_deg` property is limited to ±`steering_limit_deg` (25° by
default in `parameters.json`). The saved `.blend` is always at 0°. This experiment
checks that the full steering travel clears the frame. It also checks that the
unpowered car turns both ways on the slope and can still brake and hold mid-turn.

## Run

After the straight-ahead gate (`002`) and brake test (`003`):

```bash
.venv/bin/python experiments/004_steering/run.py
# Watch a run live:
.venv/bin/python experiments/004_steering/run.py --viewer
# Left/right comparison video (ffmpeg and EGL required):
.venv/bin/python experiments/004_steering/record.py
# Orthographic/perspective inspection views at -25°, 0° and +25° (does not save the .blend):
blender --background experiments/001_patent_motorwagen/output/patent_motorwagen.blend \
  --python-exit-code 1 --python experiments/004_steering/render_views.py -- experiments/004_steering/output
```

`run.py` re-exports the saved Blender model with experiment 002's exporter and
reuses its MuJoCo model. It writes `report.json`, `left.csv`, `right.csv`,
`downhill.xml` and meshes to `output/`. `record.py` writes
`steering_comparison.mp4`, and `render_views.py` writes `output/views/`, where
the steering assembly is orange.

## Model

The steering body is a child of the chassis with a limited yaw hinge
(`damping=1`, `armature=.02`, 2 kg). The chassis is reduced to 118 kg so the
total stays at 135 kg. The front wheel is a hinged child of the steering body.
A MuJoCo position actuator stands in for the rider's hand: kp 180, kv 12 and a
±20 N m force limit, with its control range clipped to the geometric limit.
Rear brakes are experiment 003's equivalent dry-friction joints.

Collision masks allow only two mating contacts: front axle to front hub, and
steering column to steering bearing. The fork, tiller and grip collide with
every chassis collider. That includes the bench, floor, engine and brake
linkage. The front wheel's rolling envelope collides with the chassis in every
steering pose.

## Checks

**Clearance sweep.** The kinematic sweep runs from −25° to +25° in 0.5° steps
(101 poses). At each pose it measures `mj_geomDistance` between every
steering/front collider and every chassis/rear-wheel collider. Between samples,
motion is bounded by `radius × half-step`, using the largest collider reach
from the pivot. A pair fails when the gap minus that bound falls below 5 mm.

**Sweep negative control.** A 40 mm chassis cube placed inside the fork's swept
path must be reported as a failure.

**Steered descents.** Each run lasts 6 s on the 5° slope from rest, left
(+15°) and right (−15°). The tiller ramps in from 0.75 s to 1.5 s. The rear
brakes are applied from 3.0 s with a 0.25 s ramp. Each run must meet these
criteria:
- Turns the commanded way: lateral offset > 0.25 m and yaw > 5°.
- Ends within 2° of the tiller command, inside the steering limit.
- Moves more than 1 m downhill.
- Rolls rather than skids: each wheel's rolled distance matches its
  slope-plane path within 10%.
- Final yaw matches the rear-axle tricycle kinematics ∫ v·tan δ / L within
  max(1°, 5%). This is the lateral-skid check.
- Has no forbidden contacts, keeps every wheel on the ground, stays under 10°
  of tilt and produces no solver warnings.
- Stops and holds: speed < 0.02 m/s and drift < 0.01 m over the last second.

**Half timestep.** The left run is repeated at 0.5 ms. It must still pass, and
yaw, lateral offset and stopping distance must change by less than 1°, 0.03 m
and 0.03 m respectively.

## Recorded result (MuJoCo 3.14.0)

The conservative sweep gap is **12.2 mm**. The closest pair is the steering
column and frame rail −1 at −25°. The sweep blocker was detected.

Each run moves 5.00 m downhill and 1.756 m sideways, and yaws 43.73°. The
kinematic prediction is 43.96°. The runs reach 2.2 m/s and stop 1.72 m after
braking begins. Final speed is 0.0002 m/s, the largest rolling error is 0.42%,
and there are no contacts or warnings. The left and right runs mirror each other.
At half timestep, yaw changes by 0.017°, lateral offset by 1.3 mm and stopping
distance by 1.5 mm.

The tiller servo is force-limited, so while the car is moving it sits 2–3.5°
short of the 15° command (about 12.6–13.2° during the coast). The final-angle
check passes once the car is stationary. The kinematic check uses the actual
angle, so the lag does not hide skidding. It is a property of the assumed hand
torque, not of the geometry.

## Not modeled

- Rake, trail, caster return, kingpin inclination, tire slip angles and
  self-aligning torque. The yaw axis is vertical in the chassis frame.
- Suspension travel, frame flexibility and structural strength.
- Rider reach, tiller effort or historical accuracy of the steering.
- Turns at higher speeds or on side slopes, where rollover could occur. A
  three-wheeler's lateral stability is not tested here beyond the 10° tilt
  limit at these speeds.
