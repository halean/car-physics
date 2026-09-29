# 009 — Mercedes 35 HP (1901): the first sprung car

Wilhelm Maybach's car for Emil Jellinek is often called the first modern car.
It has a pressed-steel frame, a front four-cylinder engine behind a honeycomb
radiator, a gate-change four-speed gearbox, and chain drive from a countershaft.
It is the first car in this series with **suspension**: semi-elliptic leaf
springs on both axles. It is also the first with a raked **steering wheel**
through a steering box. It is built gate-first on the 006 tooling.

**Sourced** [S16], via Wikipedia, which cites Eckermann:
- 5.9 L four-cylinder, 35 PS (25.7 kW) at 950 rpm, running range 300–1000 rpm.
- Four speeds plus reverse, gate change, chain final drive.
- Hand brake on the rear drums (about 30 cm).
- Water-cooled foot brake on the countershaft.
- Pressed-steel ladder frame of U section; rigid axles on semi-elliptic springs.
- Wheelbase 2,345 mm.
- 1200 kg; 70–75 km/h.
- The source also gives a length of 2,766 mm. That conflicts with the
  wheelbase: wheelbase plus the two wheel radii alone is 3.31 m. It is not used;
  this model's frame is 3.55 m long.

**Assumptions:**
- Track 1.40 m, wheels 0.455/0.51 m radius on 60 mm tyres.
- Gearbox 3.6/2.2/1.45/1.0 with a 1.145 bevel and 2.13 chains, so fourth gear
  gives about 75 km/h at the governor.
- Hand brake 500 N m per rear drum; foot brake 1400 N m at the wheels.
- Spring rates (static deflection about 83 mm front and 42 mm rear) and
  damping.
- Steering geometry, the frame width and all the bodywork.
- Reverse gear is not simulated.

## Commands

As for 007, with `009_mercedes_35hp_1901` and `mercedes_35hp`:

```bash
blender --background --factory-startup --python-exit-code 1 --python experiments/009_mercedes_35hp_1901/generate.py -- \
  --config experiments/009_mercedes_35hp_1901/parameters.json --output experiments/009_mercedes_35hp_1901/output/chassis --stage chassis
.venv/bin/python experiments/009_mercedes_35hp_1901/run.py --stage chassis
# ... --stage full --render, then run.py --stage full, record.py (reads the spring preloads from
# report.json), and render_views.py with views.json
```

## New mechanisms and how they are checked

- **Sprung axles.**
  - *Front axle:* heaves on a slide joint and rolls on a hinge.
  - *Rear axle:* swings about the countershaft on radius rods, which keeps the
    chains at constant length. It also rolls.
  - *Springs:* joint stiffness. The runner calibrates each preload so that the
    loaded car settles at design ride height. The gate requires 20–150 mm static
    deflection (`plausible_springs`) and less than 90% of the travel used on the
    descent.
- **Suspension poses in the build checks.** Joins, rest joins (leaf springs to
  their pads, verified at rest) and clearances are checked at bump, rebound and
  roll. Each pose is combined with the full steering sweep: 145 poses in total.
- **Steering wheel and box.** A raked column turns a 6:1 box beside the engine.
  The box is a joint-equality coupling. The pitman arm and drag link run to the
  axle. The linkage is solved in 3D at every suspension pose, and the solver
  fails loudly if the linkage binds.
- **Bump and roll steer.** Steer at the front wheels is measured at the extreme
  poses with the column held still: 0.75° worst in bump, 0.17° in roll.
- **Road bump.** A 40 mm bar at 7 m, crossed in third gear. All four wheels
  must stay on the ground and the springs must stay within travel.
- **Pure-pursuit driver.** The launch reaches 22 m/s. The earlier heading-hold
  gains went unstable above about 11 m/s, so the driver now aims at a point on
  the starting line 0.8 s ahead.

## Found and fixed while building

- **Frame width.** The build checks sweep the tyres at every suspension pose.
  They showed the steered front tyre reaching the rails at full lock with bump
  or roll, so the frame was narrowed from 0.76 m to 0.66 m.
- **Roll steer.** The first layout copied the Velo's diagonal drag link, set
  0.10 m below the axle's roll axis. It gave up to 7.7° roll steer, and the car
  spun at 16 m/s with the column held straight. Locking the axle's roll made
  that spin disappear, which confirmed the cause. The redesign:
  - The steering box moved beside the engine, with arms pointing back.
  - A transverse drag link sits at roll-axis height.
  - Roll steer is now 0.17°.
- **Hand brake locking.** At 800 N m per drum, braking pitched the sprung body
  forward and locked the rear wheels (29% slide on the steered descents). At
  500 N m the wheels stay below lock. That is still twice the 5° holding torque
  and more than fourth-gear drive.
- **Bottoming on the bump.** The first damping let the axles hit their stops.
  Damping was doubled and the rear springs made 30% stiffer.
- **Tyres lost load under the no-slip pass.** MuJoCo's no-slip friction pass,
  used since 007, dropped the normal force to zero on a lightly loaded tyre. The
  shared adapter now uses an elliptic friction cone with impratio 50, which also
  stops braked tyres creeping. All four cars were revalidated with it.
- **Tyres floated 2 mm above the road.** The recorder's force arrows vanished
  for loaded wheels, which led to a check of the tyre contact. The shared tyre
  had a 2 mm contact margin with gap = margin. It was meant to find contacts
  early but only push at real penetration. In MuJoCo 3.14 the gap does not
  cancel the margin, so the tyre pushed from 2 mm away and the loaded tyres
  rested at +0.1 to +0.7 mm. The margin was removed; the elliptic cone had
  already fixed the dropouts it was added for. All four cars were revalidated
  again, and every video was re-recorded.
- **Linkage and axle mass in the wrong place.** The Model T (010) calibrated its
  springs to impossible preloads, and the cause was traced to the shared
  `part_inertia`. It was missing a division by mass, so every small body's
  centre-of-mass offset was multiplied by its mass in kilograms. This car's rear
  axle, whose mass sits along the radius rods, had its centre of mass placed
  about 14 m forward. The bug was present since 006. After the fix, the static
  rear deflection fell from 74 to 42 mm and loop closure improved from 23 to
  2 µm. All five cars were revalidated and re-recorded.
- **Loop metric.** `max_loop_violation_m` had mixed the steering box's
  coupling (in radians) into a distance. It now counts connect equalities only.

## Result (MuJoCo 3.14.0, full build `03137461…`)

**Chassis stage** (`bbe41197…`)
- 6,357 pairs at 145 poses, all 64 joins touching, nothing disconnected.
- Gate: 6.22 m.
- All planted defects caught.

**Full build checks**
- 77,275 pairs at 145 poses (101 steering angles, plus suspension extremes
  combined with the steering sweep).
- All 187 joins touching (worst gap 0.07 µm); nothing disconnected.
- All planted defects caught.
- Closest moving pair: steering-wheel spokes to the raked column at 5.0 mm, at
  the moving limit.

**Downhill gate**
- 6.22 m travelled, 0.14 mm sideways.
- Rolling error 0.3%.
- Loops closed within 2.2 µm.
- Front heave used 14% of its travel.
- Static spring deflection 83 mm front and 42 mm rear.
- Blocked-wheel control caught.

| Scenario | Result |
| --- | --- |
| Steered descents ±20°, foot brake at 3 s | Yaw 45.4° and −48.1°, against 47.0° and −50.0° kinematic. Stops and holds. |
| Hand brake (rear drums), 5° | Stop 2.11 m / 2.42 s from 1.56 m/s; 1.8 mm hold drift; half dt 2.112 m. Holds before release and rolls after. Unbraked control fails. |
| Foot brake (countershaft), 5° | Stop 1.24 m / 1.38 s; 2.2 mm hold drift; half dt 1.238 m. |
| Launch through four gears, 40 s, pure-pursuit driver | 4.222 m/s at 2 s against 4.214 reference; 715.1 m against 716.8 m. Governed 5.8, 9.4, 14.3 and 20.8 m/s; final 21.8 m/s (79 km/h, no-load). 25.70 kW peak. Lateral 0.02 mm. |
| 15° hill start, first gear | 91.9 m against 91.95 reference; no rollback. Predicted limit 22.2°. |
| 8° hill start, fourth gear (negative control) | Rolls back 3.11 m against 3.14 reference. Predicted limit 6.0°. |
| Powered turns, first gear, ±10° | Radius 13.4 and 12.6 m, about 0.27–0.29 g. Differential ratio 1.109 against 1.110, and 1.116 against 1.118. Tyre loads steady within 0.01%. |
| Locked differential (negative control) | 1.000 against 1.112, so caught. |
| Engine against the hand brake, fourth gear | 631 N m of drive is held by the 1000 N m brake; the car stops. |
| Road bump, 40 mm, third gear | All wheels on the ground; peak travel 98% front heave and 74% rear swing (limit 100%); 1.8° peak tilt. |
| Half timestep, launch | Changes of 0.0004% in speed and 0.03% in distance. |

**Findings (informational)**
- *Engine against the hand brake in first gear:* 2270 N m of drive beats the
  1000 N m brake. The driver must declutch, as with any car of the period.
- *Foot brake in a turn:* it works through the open differential (the Velo
  lesson). This time no wheel slid.
- *Fixed-hands launch:* drifts 10.8 m sideways over 716 m. The fixed column
  holds a small linkage offset, and at speed this adds up.
- *Ackermann:* up to 1.9° over ideal at full lock.
- *Bump margin:* the 40 mm bump uses 98% of the front travel. That passes, but
  a larger bump would hit the stops.

## Not modeled

- Leaf-spring wind-up, friction between leaves, and axle location by the
  springs. The joints locate the axles; the springs are joint stiffness.
- Engine internals, the honeycomb radiator's cooling, the water-cooled brake's
  heat, and the exact gate pattern.
- Reverse gear.
- Pneumatic tyre behaviour beyond the shared compliant contact.
- Rolling resistance and drag, which matter at 22 m/s. Top speed is set by the
  governor, not by drag.
- Occupants.
