# 005 — Engine drive: belt, countershaft, differential and chains

The engine now drives the rear wheels, following the 1886 layout [S7]. Power
goes from the flywheel shaft through bevel gears to a pulley. An uncrossed flat
belt carries it to a countershaft pulley, which sits beside the differential. A
roller chain on each side drives a sprocket bolted to the outer face of that
wheel's brake drum. Shifting the belt onto the fixed pulley is the clutch.
Dimensions are assumptions, not a measured reconstruction.

| Stage | Pitch radii (m) | Ratio |
| --- | --- | --- |
| Bevel gears | — | 1 : 1 (assumed) |
| Belt: engine pulley → countershaft pulley | 0.060 → 0.138 | 2.30 |
| Chain: countershaft sprocket → wheel sprocket | 0.051 → 0.106 | 2.08 |
| Overall | | **4.78** |

The engine is modeled at 0.5 kW governed at 400 rpm [S7], which is 11.9 N m at
the flywheel shaft. With 0.5 m rear wheels, 400 rpm gives 4.38 m/s (15.8 km/h).
The commonly quoted top speed is about 16 km/h.

The build also fixes two earlier geometry faults:
- The brass reservoir floated 10 cm above the engine bed, inside the flywheel's
  swept disk. It now stands on the bed, clear of the flywheel.
- The flywheel was 4 mm above the cooling collars. It is now 14 mm above them.

## Run

After rebuilding and passing experiments 002–004:

```bash
.venv/bin/python scripts/build.py --render
blender --background experiments/001_patent_motorwagen/output/patent_motorwagen.blend \
  --python-exit-code 1 --python experiments/001_patent_motorwagen/check_frame.py
.venv/bin/python experiments/005_engine_drive/run.py            # --viewer watches the flat launch
.venv/bin/python experiments/005_engine_drive/record.py         # engine_drive.mp4 (ffmpeg + EGL)
blender --background experiments/001_patent_motorwagen/output/patent_motorwagen.blend \
  --python-exit-code 1 --python experiments/005_engine_drive/render_views.py -- experiments/005_engine_drive/output
```

The last command writes the inspection views to `output/views/`, with the
drivetrain in orange and the brakes in red. The engine assumptions are in
`engine.json`.

The video and `--viewer` draw per-wheel contact-force arrows (`visual.py`),
summed from `mj_contactForce` over each tire's contacts:
- **Blue: tire load.** 1 m per 1000 N.
- **Orange: road force on the tire** (drive, braking, cornering). 1 m per 200 N.

Each arrow has its own scale, because MuJoCo's built-in contact arrows share
one, which shrinks the ~57 N drive force to a tenth of the ~530 N load. MuJoCo
3.14 draws `mjGEOM_ARROW` at half the length set by `mjv_connector`
(measured). The helper doubles it, so the stated scales are true. The arrows
are display only; the tests do not use them.

## Model

**Differential.** The drive is a MuJoCo motor on a fixed tendon, ½·θ_left +
½·θ_right, with gear equal to the overall ratio. This is an ideal open
differential: each rear wheel gets half the torque, their speeds are free to
differ, and the actuator's velocity is the engine speed. The drivetrain has no
mass or friction.

**Engine and belt** (`control.py`):
- Below 400 rpm the belt slips on the flywheel-driven pulley and passes rated
  torque. The flywheel is assumed to hold engine speed.
- The governor cuts torque linearly to zero at 420 rpm. The droop is an
  assumption.
- There is no engine braking.
- The belt shift takes 0.5 s.

**Unpowered runs.** When experiments 002–004 build the model, the drive
actuator is absent. With the belt on the loose pulley there is no drive path,
so those tests stay unpowered with the brakes released.

**Collisions.** Each chain is exempt from contact only with its own wheel
sprocket, using its own collision bit. It still collides with the wheel's
rolling envelope and brake drum. Pulleys, shafts, the belt and the countershaft
are chassis colliders against the wheels and the steering assembly.

## Checks (thresholds fixed before the first powered run)

**Spatial checks (Blender, `check_frame.py`):**
- Connectivity now covers 55 members. It includes the engine bed (checked as a
  box), the reservoir and the full drive path to the frame rails.
- The belt and chains must follow the ideal open-belt path on their pulleys'
  actual axes and pitch circles, within 2 mm.
- Wheel sprockets must be coaxial with their hub sleeves and overlap them.
- Moving the belt, a chain, a countershaft hanger or the reservoir must fail.
- Wheel clearance now samples flat mesh cylinders exactly, instead of treating
  them as capsules.

**Swept drive clearance.** Every rotating part is checked as the solid it
sweeps: flywheel and arms as a disk, pulleys and shafts as their own
cylinders, and the belt and chains as fixed loop shapes. They must stay 5 mm
from every other collider, including parts on the same rigid body. Only listed
mating parts are exempt. A 20 mm cube planted in the belt's path must be caught.

**Powered runs.** Speeds and distances are compared with a one-dimensional
longitudinal model using the MuJoCo masses, wheel inertia, engine curve, belt
shift and brake schedule. Every run also must pass the downhill-gate
contact, rolling, stability and warning checks. Engine power must stay at or
below 500 W, and engine speed at or below 1.01 × no-load speed.

| Run | Pass condition |
| --- | --- |
| Flat launch (engage 0.5 s, disengage 20 s) | Within 3% of the reference model at 5 s and 20 s; ≥ 95% of governed speed; no drive after disengaging; straight; equal rear wheel speeds |
| 3° hill start (brake held, released at 1.5 s) | Rolls back < 0.05 m; climbs > 5 m; within 3% of the reference |
| 7° hill start | Rolls back > 0.5 m at full engine torque, as the reference predicts. This negative control catches a velocity-forced drive. |
| Powered turns, left and right, 15° tiller | Outer/inner rear wheel speed ratio within 3% of (R + t/2)/(R − t/2); kinematic yaw within max(1°, 5%) |
| Locked differential (negative control) | Must fail the ratio check |
| Brakes against the engaged engine | Stops and holds (< 0.02 m/s, < 0.01 m drift over the last second) while the engine still commands full torque |
| Half timestep, flat launch | Speed at 10 s and distance at 20 s change < 1% |

Two test-design errors were found and fixed after the first run. The
thresholds were not changed.
- **Yaw wrap-around.** The powered turns completed nearly a full circle, so the
  heading wrapped and the lateral offset returned to near zero. Heading is now
  unwrapped.
- **Brake run too short.** The brakes-against-engine run ended before the
  predicted 3.3 s stop. It now runs to 15 s.

The same run also exposed a real fault. The bevel output shaft passed through
the engine output shaft. It now starts 10 mm away, with gears inside the
housing.

## Recorded result (MuJoCo 3.14.0, build `52f5cdf3…`)

**Swept clearance:** 41,797 pairs checked. The smallest gap is 9 mm, between
drive chain −1 and the right brake drum. The planted blocker was detected.

**Flat launch:**
- 3.23 m/s at 5 s against a reference of 3.23 m/s.
- 74.58 m at 20 s against a reference of 74.62 m.
- 4.60 m/s at 20 s, with the engine at 420 rpm (governor no-load).
- Peak power 499.98 W.

**3° hill start:** 49.05 m in 20 s (reference 49.23 m), with no measurable
rollback. The predicted grade limit is **4.9°**.

**7° hill start:** rolls back 3.20 m (reference 3.20 m).

**Powered turns:**
- Steering 13.5°, turn radius 6.05 m.
- Wheel speed ratio 1.2145 against 1.2180 predicted.
- Yaw 346.6° against 349.7° kinematic, at 4.47 m/s with 0.002° chassis tilt.

**Locked differential:** ratio 1.000 against 1.246 expected, so the control
was caught. Front-wheel rolling error rose to 4.7%, showing tire scrub.

**Brakes against the engine:** stops 8.95 m after application from 4.6 m/s,
and holds with 0.02 mm drift.

**Half timestep:** changes are below 1e-6. Acceleration is constant before the
governor cuts in, so the result barely depends on timestep.

## Engine and brakes together

```bash
.venv/bin/python experiments/005_engine_drive/brake_with_engine.py
```

This compares three flat-road runs with the belt engaged throughout; in two of
them the rear brakes come on at 5 s. It writes `brake_with_engine.json`, one CSV
per case and `brake_with_engine.mp4` with force arrows. It exits nonzero if any
verdict fails. The light-brake prediction was written down before the first run.

| Case | Result |
| --- | --- |
| Engine only | 4.60 m/s at 14 s; governor near no-load (420 rpm), almost no drive torque left |
| Full brakes, 75 N m per wheel (150 total vs 57 N m of drive at the wheels) | Stops from 3.23 m/s and holds while the engine still commands its full 11.9 N m. The belt slips with the engine held at governed speed; about 1.3 kJ goes into brake heat. |
| Light brakes, 20 N m per wheel (40 total, below the drive) | Keeps going. The governor settles where engine torque × 4.78 = 40 N m: 405.9 rpm and 4.4454 m/s, against 406.0 rpm and 4.4467 m/s predicted. About 356 W of engine power goes into the dragging brakes (2.9 kJ in 9 s). |

While the full brakes are slowing the car, each rear tire takes about 87 N
backward. The front tire's load rises from 260 N to 387 N as weight shifts
forward. The video shows both. The simulation has no brake heating or fade.

## Not modeled

- Drivetrain inertia and friction; belt creep, belt tension and chain dynamics.
- Engine combustion, torque pulsation, starting and stalling.
- Engine braking and gyroscopic effects of the horizontal flywheel.
- Rolling resistance and aerodynamic drag, so top speed is set by the governor.
- Tire slip angles and suspension.

Mass stays at the earlier 135 kg assumption for comparability. The historical
car is reported at 270 kg without occupants [S7]. At that mass the 4.78
ratio would limit it to about 2.5°, so the 3° climb would fail.
