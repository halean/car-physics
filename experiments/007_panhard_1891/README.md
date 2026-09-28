# 007 — Panhard et Levassor (1891): the "Système Panhard"

This is the first front-engined layout in the series: the engine at the front,
then a friction clutch, a sliding three-speed gearbox, a bevel to a countershaft
with the differential, and chains to the rear wheels. It is built gate-first on
experiment 006's validated tooling.

**Sourced features:**
- Daimler-licence vertical V-twin in front of a friction clutch and a sliding
  three-speed gearbox, with chain drive to the rear wheels [S11].
- Hand-lever, leather-faced wooden block brakes on the rims of the rear wheels
  only, plus a sprag [S11].
- Wooden chassis on unequal front and rear wheels [S11].
- Front engine driving the rear axle through clutch, gearbox and differential
  [S12].
- The steering wheel only appeared on a Panhard in 1894, so this 1891 car has a
  tiller [S12].
- 1.1 L, 4 bhp and 19 km/h come only from a search-result snippet; the page
  itself was blocked [S13].

**Assumptions:**
- All dimensions: 1.60 m wheelbase, 1.20/1.25 m track, 0.70/1.00 m artillery
  wheels.
- 600 kg mass.
- Governed at 750 rpm, which gives 38 N m.
- Gearbox ratios 5.16/3.13/2.02; third gear gives about 19 km/h.
- Rim-brake capacity of 180 N m per wheel.
- The double-pivot front axle, carried over from the Velo (its linkage
  details are not sourced).
- Bodywork.

## Commands

```bash
blender --background --factory-startup --python-exit-code 1 --python experiments/007_panhard_1891/generate.py -- \
  --config experiments/007_panhard_1891/parameters.json --output experiments/007_panhard_1891/output/chassis --stage chassis
.venv/bin/python experiments/007_panhard_1891/run.py --stage chassis
blender --background --factory-startup --python-exit-code 1 --python experiments/007_panhard_1891/generate.py -- \
  --config experiments/007_panhard_1891/parameters.json --output experiments/007_panhard_1891/output/full --stage full --render
.venv/bin/python experiments/007_panhard_1891/run.py --stage full
.venv/bin/python experiments/007_panhard_1891/record.py        # panhard_drive.mp4, 50 ms averaged force arrows
blender --background experiments/007_panhard_1891/output/full/panhard_1891.blend --python-exit-code 1 \
  --python experiments/006_benz_velo/render_views.py -- experiments/007_panhard_1891/output/full/physics/geometry.json \
  experiments/007_panhard_1891/output/full experiments/007_panhard_1891/views.json
```

## Reuse from 006

These come unchanged from 006: `export_blender.py`, `vehicle.py`, `linkage.py`,
the build checks, the simulation loop and `render_views.py`, which now accepts
extra close-up views. The generator imports the Velo's part tagging and the
shared `double_pivot_front`, which gained an off-centre column so it clears the
central drive shaft. The refactor was proven exact: re-exporting the full Velo
gave identical colliders, joins, bodies and loops, and its whole suite passes.

This car's own files:
- `generate.py`: wooden box rails, artillery wheels, tiller, drivetrain,
  brakes, body.
- `control.py`: clutch, three speeds, rim brakes.
- `dynamics.py`: the scenarios, with `LIMITS`.
- `run.py`: the runner and its own planted defects.
- `record.py`, `views.json`: the video and the close-up views.

## Found and fixed while building

- **Build checks:**
  - The seat back missed the cushion by 1 cm.
  - The hand lever and the right brake arm met at the same point of the cross
    shaft; the lever now sits outboard.
  - The engine-box front rested on the rails without a declared join.
  - Before the first build, a manual check found that the footboard would cross
    the brake cross shaft.
- **Brake runs lengthened:** the rim-block hand brakes stop a 600 kg car slowly,
  at 0.32 m/s² net on 5°. The 6 s windows carried over from lighter cars ended
  before the stop. Runs are now 12 s, with release at 10 s; engine against
  brakes gets 25 s. Thresholds are unchanged. Weak hand brakes fit the history:
  the car also carried a sprag for hills.

## Result (MuJoCo 3.14.0, full build `c49ca0e8…`)

**Chassis stage** (`40d8cbb6…`)
- Build checks: 1,104 pairs, all 47 joins touching.
- Gate: 6.24 m.
- All three planted defects caught.

**Full build checks**
- 14,911 collider pairs at 121 tiller angles, same-body pairs included.
- All 112 joins touching; nothing disconnected.
- All three planted defects caught.
- The closest pairs are 5–6 mm apart: the brake lever to the right arm, the
  drag arm to the drag link, and the knuckle web to the axle eye.

**Downhill gate** (5°, 4 s, belts off, brakes released)
- 6.24 m travelled, 0.13 mm sideways.
- Rolling error below 0.006%.
- Loops closed within 2 µm.
- Blocked-wheel control caught.

| Scenario | Result |
| --- | --- |
| Steered descents, ±20° tiller, rim brakes at 3 s | Yaw 72.9° and −76.5°, against 74.1° and −77.9° kinematic. Stops and holds; slip 0.2%. Tiller torque up to 15.8 N m. |
| Rim brakes, 5° | Stop 4.55 m / 5.3 s from 1.56 m/s; 5.4 mm hold drift; half dt 4.551 m. Unbraked control fails. |
| Launch 1st → 2nd at 4 s → 3rd at 8 s, 30 s, heading-holding driver | 2.152 m/s at 3 s against 2.152 reference; 136.47 m against 136.60 m. Governed 2.07, 3.41 and 5.28 m/s; final 5.50 m/s at no-load. 2980 W peak. |
| 10° hill start, first gear | 34.96 m in 20 s against 35.01 reference; no rollback. Predicted limit 14.2°. |
| 8° hill start, third gear (negative control) | Rolls back 4.11 m against 4.10 reference. Predicted limit 5.5°. |
| Powered turns, second gear, ±20° | Rear wheel speed ratio 1.307 against 1.311, and 1.359 against 1.364. Kinematic yaw within 1.5%. |
| Locked differential (negative control) | 1.000 against 1.311, so caught. |
| Engine and brakes, third gear | Stops and holds while driving: 282 N m at the wheels against 360 N m of brake. |
| Half timestep, launch | Changes of 0.004% in speed and 0.04% in distance. |

**Findings (informational)**
- *First gear against the brakes:* 720 N m of drive against 360 N m of brake,
  so the car keeps going at 2.1 m/s. The driver must declutch.
- *Fixed-hands launch:* 0.22 m of drift from linkage asymmetry, the same effect
  as on the Velo.
- *Ackermann:* up to 2.0° over ideal at full lock.

## Known limitation: tyre-force jitter in powered turns

In powered cornering the tyre loads oscillate at about 95 Hz. The motion is
micrometre-scale, but in a second-gear 20° turn the total load has a 25%
standard deviation and the inner front tyre's load up to 72%. Straight running
shows none of this.

Isolation runs ruled out the steering loop constraints, servo damping, timestep
and friction-cone type. It is the stiff tyre–road contact. The contact is still
nearly rigid at wheel scale: deflection is micrometres, where a real solid tyre
deflects millimetres. Lateral stick-slip under cornering excites a pitch mode.

The averages used by every check are correct:
- Loads sum to the car's weight.
- Yaw, speeds and differential ratios match the kinematics.

Instantaneous contact forces in turns are not trustworthy, so the video draws
**50 ms averaged** force arrows (`visual.ForceAverager`). Making MuJoCo's
contact compliance match a real tyre is separate work. It would require
re-validating the Velo and this car.

## Not modeled

- Suspension, caster and trail.
- Engine pulsation, clutch dynamics beyond slip at rated torque, and gear
  internals. The gearbox is a box, and its ratios are parameters.
- Reverse gear.
- The sprag.
- Rolling resistance and drag.
- Brake heat.
- Occupants.
- Structural strength.
