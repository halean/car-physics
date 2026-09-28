# 006 — Benz Velo (1894): a second car, built gate-first

A procedural study of the early **Benz Velo**, Benz's first series-production
car. It is the Motorwagen's four-wheeled successor, with Benz's double-pivot
steering. The Velo is built and validated with the lessons from experiments
001–005.

**Sourced figures:**
- 1340 mm wheelbase, 1000 mm front and 1040 mm rear track, 2.25 m long [S8].
- 550/850 mm front/rear wire wheels on solid tyres [S8].
- 1045 cc rear engine, 1.5 hp (1.1 kW) at 450 rpm [S8].
- Belt to countershaft, chain final drive, two forward speeds [S8].
- Double-pivot steering from a central column [S8].
- 280 kg [S9].

The following are **assumptions**, not reconstruction: frame layout, linkage
dimensions, pulley and sprocket sizes, both brakes, the horizontal flywheel,
masses, and bodywork. No source gives the Velo's brake locations. A
contemporary Benz had a hand and a foot brake [S10].

## Build stages and commands

```bash
# 1. Chassis only: frame, axles, full steering linkage, wheels -> gate
blender --background --factory-startup --python-exit-code 1 --python experiments/006_benz_velo/generate.py -- \
  --config experiments/006_benz_velo/parameters.json --output experiments/006_benz_velo/output/chassis --stage chassis
.venv/bin/python experiments/006_benz_velo/run.py --stage chassis
# 2. Full car: + engine, two-speed belts, countershaft differential, chains, brakes, bodywork
blender --background --factory-startup --python-exit-code 1 --python experiments/006_benz_velo/generate.py -- \
  --config experiments/006_benz_velo/parameters.json --output experiments/006_benz_velo/output/full --stage full --render
.venv/bin/python experiments/006_benz_velo/run.py --stage full        # --viewer to watch the gate
.venv/bin/python experiments/006_benz_velo/record.py                  # velo_drive.mp4, force arrows
blender --background experiments/006_benz_velo/output/full/benz_velo.blend --python-exit-code 1 \
  --python experiments/006_benz_velo/render_views.py -- \
  experiments/006_benz_velo/output/full/physics/geometry.json experiments/006_benz_velo/output/full
```

| File | Role |
| --- | --- |
| `generate.py` | Blender generator. It reuses the Motorwagen's primitives, belt loops and seating check. Each part records its physics body, declared joins and moving mates. |
| `export_blender.py` | Writes per-body colliders and meshes. Rods become cylinders; curves and rings become capsules; other meshes become boxes. |
| `vehicle.py`, `linkage.py` | MuJoCo model, linkage kinematics and build checks. |
| `run.py`, `dynamics.py` | Build checks, the downhill gate and all scenarios, with thresholds in `LIMITS`. |
| `control.py`, `engine.json`, `vehicle.json` | Driver, engine, brakes and physical assumptions. |

## What carried over from 001–005

- **Gate first.** The bare chassis passed the unpowered downhill gate before any
  drivetrain or bodywork was added. That evidence is kept in `output/chassis/`.
- **Declared joins.** Every intended attachment is declared. It must touch at
  every steering angle, and the declared joins must connect every part.
- **Everything else stays apart, same body included.** Undeclared pairs must
  stay apart: 1 mm for parts on the same rigid body, and 5 mm for parts that
  move relative to each other. Rotating drive parts count as their swept
  solids. This closes the gap that a rigid-body simulation leaves open.
- **Negative controls for the checks.** Three planted defects must be caught:
  a lifted front hanger (loose join), a drag arm moved into the knuckle ear
  (same-body intersection), and a blocker at the tie-rod ball's full-lock
  position (sweep intrusion).
- **A real linkage in physics.** Knuckles, tie rod and drag link are hinged
  bodies, with the two loops closed by connect constraints. The Ackermann
  behaviour comes from the geometry; `linkage.py` solves the same geometry
  analytically for posing and the sweep.
- **Other carry-overs.**
  - Historical mass, 280 kg.
  - Differential as a tendon.
  - Engine that is torque-limited and governed.
  - One-dimensional reference model.
  - Locked-differential and too-steep-hill negative controls.
  - Half-timestep repeats.
  - Force-arrow video.

## Model notes and changes made during validation

**Steering.** The drag-link side is a parallelogram, so the column turns the left
knuckle 1:1. The trapezoid is aimed at the rear-axle centre (Jeantaud), which
gives under-Ackermann: the outer wheel turns more than ideal, by 0.11° at 5° of
column and 1.87° at full lock. At −30° column the inner wheel reaches 37.8°.
Right turns are tighter than left ones: 3.43 m against 3.91 m radius at 20°
of column.

**Driver grip.** The column is driven by a position servo with a 40 N m hand
limit. The first full run used kp 150, which lagged 5° under the ~13 N m
holding torque. It was raised to kp 1000 with no change to thresholds. The
steering needs about 6.6 N m at 20° under way.

**Linkage inertia.** Small bodies originally had their mass lumped at the joint.
That pushed the steering under acceleration. Centre of mass and inertia now
come from each body's colliders.

**Tyres.** A flat 60 mm cylinder with the inherited 0.005 s contact chattered in
cornering: 16% of steps had no contact at all, and loads reached 2.6 × weight.
The video's force arrows showed it; no test did. Tyres are now round-crowned:
an ellipsoid with 18 mm half-width and a 0.02 s contact time constant, roughly
0.3–0.4 MN/m per wheel, which is assumed. A separate full-width envelope, which
never touches the road, keeps spoke clearance checks conservative.

**Brakes.**
- *Foot brake:* a band on a countershaft drum. It acts through the differential,
  as tendon friction, with 350 N m at the wheels.
- *Hand brake:* a band on a drum cast with each rear sprocket, 100 N m per
  wheel, as joint friction.
- *Why two:* braking through an open differential gives the unloaded inner rear
  wheel equal torque in a turn. On a 20° descent it skids at 11% and spins
  backwards at up to 1.3 rad/s. The steered descents therefore use the rear
  drums. The band-in-a-turn run is kept in the report as an informational
  finding.

**Hold window.** "Holds before release" now measures from the car's own stop
(+0.25 s) until release. The old 4–5 s window assumed the Motorwagen brake's
stopping time. The 0.02 m/s threshold is unchanged.

**Reference model.** Engine torque, 23.3 N m, follows from 1.1 kW at 450 rpm.
Wikipedia's 4.4 N m is inconsistent with its own rating and is not used.

## Result (MuJoCo 3.14.0, full build `d4eacc91…`)

**Build checks**
- 97,472 collider pairs at 121 steering angles.
- All 138 joins touch. Nothing is disconnected.
- All three planted defects were caught.
- The closest moving pairs are 5–6 mm apart: knuckle web to axle eye, and the
  footboard's edge to the column bearing.

**Downhill gate** (5°, 4 s, brakes released, belts off)
- 6.21 m travelled, 0.14 mm sideways.
- Rolling error below 0.006%.
- Loop closure within 1 µm.
- No forbidden contacts. The blocked-wheel control was caught.

| Scenario | Result |
| --- | --- |
| Steered descents, ±20° column, rear drums at 3 s | Yaw 72.0° left and −77.2° right, against 72.6° and −77.9° kinematic. Stops and holds; slip under 0.1%. |
| Band brake, 5° | Stops in 0.91 m / 1.0 s from 1.55 m/s; 2.7 mm hold drift; half dt 0.91 m. Unbraked control fails. |
| Rear drums, 5° | Stops in 2.00 m / 2.3 s; 1.4 mm drift; half dt 2.005 m. |
| Low-gear launch, shift to high at 6 s, 30 s, heading-holding driver | 3.206 m/s at 4 s against 3.206 reference; 148.68 m against 148.92 m. Low governs at 3.07 m/s; final 6.00 m/s is high gear at no-load. 1100 W peak. Ends 0.001 mm off line. |
| 5° hill start, low gear | 44.99 m in 20 s against 45.06 reference; no rollback. Predicted limit 7.5°. |
| 5° hill start, high gear (negative control) | Rolls back 1.59 m against 1.58 reference. Predicted limit 4.0°. |
| Powered turns, low, ±20° | Rear wheel speed ratio 1.313 against 1.307, and 1.367 against 1.357. Kinematic yaw within 2%. |
| Locked differential (negative control) | Ratio 1.000 against 1.306 expected, so caught. Inner rear wheel slips 15%. |
| Engine and band brake together | Stops and holds while the engine drives at full torque (152 N m at the wheels against 350 N m of brake). |
| Half timestep, launch | Changes of 0.003% in speed and 0.04% in distance. |

**Launch straightness, resolved with a heading-holding driver.**
- *First result:* with hands fixed at straight ahead, the 30 s launch ended
  0.27 m right after 149 m, a 0.12° heading change. That failed the 0.05 m limit
  carried over from 005.
- *Cause:* the linkage is asymmetric. The drag arm and diagonal drag link are on
  the left, so under acceleration it produces about 0.1 N m of steering torque,
  and the driver's grip yields about 0.006°.
- *Evidence from isolation runs:*
  - A near-rigid grip cuts the drift tenfold.
  - Changing the loop-constraint stiffness changes nothing.
  - Rear wheel speeds match within 1%.
- *Fix:* the gated launch now uses a driver who holds the heading. The column
  command is −1.0 × heading error − 0.3 rad/m × sideways offset. The launch ends
  0.001 mm off line, and the 0.05 m threshold is unchanged.
- *Kept as information:* the fixed-hands run, `finding_open_loop_launch`. It
  shows the sensitivity a real driver would correct for.

## Not modeled

- Suspension, frame flexibility and caster/trail.
- Tyre slip-angle mechanics beyond MuJoCo friction.
- Engine pulsation and flywheel gyroscopics.
- Drivetrain inertia and friction; belt creep and loose pulleys.
- Rolling resistance and drag, so top speed is set by the governor.
- Brake linkage, heat and fade.
- Occupants.
- Structural strength.
- Historical dimensional accuracy beyond the sourced figures.
