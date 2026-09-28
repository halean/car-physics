# 002 — Does the car roll downhill?

A MuJoCo integration test of the saved Blender model. The exporter reads all 197
vehicle objects from experiment 001; it does not rebuild a second hand-written
frame. Curves become capsules and rods become cylinders, boxes become conservative oriented boxes,
and the horizontal flywheel rim becomes a ring of capsule segments. Evaluated
Blender meshes supply the visuals. Each wheel uses a conservative cylindrical
collision envelope containing the tire and swept spokes.

## Run

From the repository root, after building experiment 001:

```bash
.venv/bin/python -m pip install -r experiments/002_downhill/requirements.txt
.venv/bin/python experiments/002_downhill/run.py
# Watch a new test in a live desktop window:
.venv/bin/python experiments/002_downhill/run.py --viewer
# Optional recording (ffmpeg and EGL required):
.venv/bin/python experiments/002_downhill/record.py
```

Every test exports the current saved `.blend` again. Save Blender edits first.
Override `--blend`, `--output`, `--blender`, `--slope-deg`, or `--seconds` as needed.
The default is a 5-degree slope, four seconds, 1 ms time steps, zero initial
velocity and no motors. Outputs are `downhill.xml`, visual OBJ files,
`geometry.json`, `trajectory.csv` and `report.json`. `record.py` records the default
output model for four seconds as `downhill.mp4`.

## Acceptance criteria

- More than 1 m of downhill progress and more than 1 m of wheel rolling distance.
- Each wheel's angle × radius agrees with displacement within 10%.
- All wheels contact the ground during the test.
- No chassis ground drag or forbidden wheel/chassis contacts.
- Less than 10 degrees of chassis tilt relative to the slope; no solver warnings
  or non-finite state values.
- A negative control with a bar through the front wheel must produce a forbidden
  contact. Its failure is expected and does not overwrite the passing model.

Parent/child collision filtering is explicitly disabled. Collision bitmasks
allow all ordinary chassis/wheel pairs; only named axle primitives can overlap
wheel hubs. Visual meshes have collision disabled so MuJoCo does not silently
replace the entire hollow chassis with one convex collision hull.

## Assumptions and limits

The chassis is one rigid body, mass 118 kg (plus a 2 kg steering assembly), center of mass `(0.3, 0, 0.65)` m,
diagonal inertia `(15, 45, 50)` kg m². Each wheel is 5 kg with ring-approximation
inertia. These are plausible test inputs, not historical measurements. Sliding
friction is 0.8 and axle damping is 0.01. The fork/tiller assembly is on a yaw hinge
held straight ahead by the tiller position servo (see
[experiment 004](../004_steering/README.md) for steered runs); wheel rotation is free. No drivetrain, aerodynamic load or tire deformation is modeled.

A disconnected decorative or structural part assigned to the chassis cannot fall
off: rigid attachment is an input to this simulation. Parts on the same rigid
body do not collide with each other. Keep experiment 001's connectivity and
floor-clearance tests. A passing descent is evidence of rolling and clearance
for these collision shapes and conditions, not proof of construction feasibility.

The exporter uses construction centerlines for rod/curve collision proxies;
arbitrary edits to their mesh vertices are not reflected in those proxies.
The evaluated visual meshes do reflect such edits. This approximation must remain
explicit when interpreting a pass.

## Recorded result

MuJoCo 3.14.0: 6.149 m in four seconds; all three wheels' rolling distances agree
within 0.006%; no forbidden contacts or solver warnings. The blocked-wheel
negative control was detected.

References: [MuJoCo collision filtering and convex mesh handling](https://mujoco.readthedocs.io/en/latest/computation/),
[Python bindings](https://mujoco.readthedocs.io/en/latest/python.html).
