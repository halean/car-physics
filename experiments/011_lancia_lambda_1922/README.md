# 011 — Lancia Lambda (1922): independent front suspension, unitary hull, four-wheel brakes

Vincenzo Lancia's Lambda is the first car in this series with three modern
features at once. Each front wheel rides up and down its own **sliding pillar**,
with a coil spring and a hydraulic damper, so a bump on one side no longer
tilts the whole axle. The body is a **load-bearing hull** rather than a frame
with a body on top. And the brakes work on **all four wheels**. It is built
gate-first on the 006 tooling, with the suspension checks from 009 and 010.

**Sourced** [S18, S19, S20]:
- Narrow-angle V4 (13°), 2121 cc, 49 PS (36 kW) at 3250 rpm.
- Load-bearing unitary body (without a stressed roof), a welded and riveted
  steel shell with a central tunnel for the drive shaft and a transverse tunnel
  for the rear axle.
- Independent front suspension: sliding pillars with coil springs, the spring
  and a hydraulic damper in one unit.
- Rear: live axle on semi-elliptic springs with friction dampers.
- Four-wheel mechanical drum brakes.
- Three-speed gearbox (first to fourth series).
- Wheelbase 3.10 m; width 1.66 m; 1200–1350 kg (S18), 1065 kg for a
  fourth-series car (S19).

**Assumptions:**
- 1200 kg, the low end of S18's range; no occupants.
- Track 1.40 m, wheels of 0.38 m radius on 55 mm tyres, wire wheels.
- Gearbox 3.4/1.9/1.0 with a 4.5:1 final drive: 104 km/h at rated speed in
  top; later series are quoted at 68–70 mph.
- Brakes: 260 N m at each front drum, 300 N m at each rear, all through the
  pedal; the hand lever works the rear drums only.
- Springs: 30 kN/m per pillar (80 mm static deflection) with 1.9 kN s/m of
  damping, about 0.35 of critical, standing in for the hydraulic damper. Rear:
  86 kN/m heave with 300 N of dry friction standing in for the friction
  dampers, and 60 N m in roll.
- Right-hand drive, with a steering box behind the front cross member and an
  8:1 reduction. The driving side is not established from the sources used.
- Hull dimensions, engine placement, and all the bodywork.
- Reverse gear is not simulated.

## Commands

As for 010, with `011_lancia_lambda_1922` and `lancia_lambda`:

```bash
blender --background --factory-startup --python-exit-code 1 --python experiments/011_lancia_lambda_1922/generate.py -- \
  --config experiments/011_lancia_lambda_1922/parameters.json --output experiments/011_lancia_lambda_1922/output/chassis --stage chassis
.venv/bin/python experiments/011_lancia_lambda_1922/run.py --stage chassis
# ... --stage full --render, then run.py --stage full, record.py, and render_views.py with views.json
```

## New mechanisms and how they are checked

- **Sliding pillars.** Each side has a fixed pillar (the kingpin) between the
  hull's upper and lower arms. A carrier slides on it (a `pillar_<side>` body
  with a vertical slide joint carrying the spring, the damping and the
  preload) and the knuckle turns on the carrier (a hinge, as before). The
  coil spring is drawn as a helix with a cylinder as its collider; its flex is
  the joint's, so its seat on the upper arm is a rest join. The fits that
  slide and turn are declared joins, which must touch at every pose.
- **Independent poses in the build checks.** The shared solver
  (`linkage.suspended_pose`) gained an `independent_front` branch: each pillar
  at its own height, the tie rod on a ball joint (it tilts when the wheels are
  at different heights), the drag link closed in 3D. The check poses are both
  wheels up, both down, and one up with the other down, each combined with
  the full steering sweep: 173 poses in all.
- **Bump steer, by design.** The pitman and drag arms are equal and parallel
  (a parallelogram, so the steering is 1:1 and symmetric), and the drag link
  runs across the car. With purely vertical wheel travel, the link's end moves
  inboard only by dz²/2L, so bump steer is second order: 0.59° at full travel,
  0.73° at full lock. A first layout with a lengthwise link and a non-parallel
  drag arm gave 1.6:1 steering and 2.5° of bump steer. With one wheel up and
  the other down, the tilted tie rod steers the right wheel 2.0°.
- **Propeller shaft that lengthens.** Two bodies: the shaft on a ball joint at
  the front universal joint, and a splined rear piece on a slide joint,
  closed to the axle's pinion by a connect constraint. The solver (`_shafts`)
  poses both from the axle's position, so the build checks see the shaft at
  every pose. All four pieces are swept solids.
- **Four-wheel brakes.** Each wheel joint has its own brake friction. Two new
  gated scenarios:
  - *Four-wheel braking in a turn* on a 5° descent: the car must stop and
    hold with every wheel still rolling (on the Velo, braking through the
    differential skidded the inner wheel).
  - *Weight transfer:* braking from third gear on level ground, the front
    tyres' share of the load must rise (measured from the contact forces) and
    the rear wheels must keep rolling.
- **Friction dampers as dry friction.** The rear axle's heave joint carries
  300 N of `frictionloss`, the first use of the adapter's joint friction for
  a damper.

## Found and fixed while building

- **Steering layout.** The first layout steered 1.6:1 and had 2.5° of bump
  steer (above). Redesigned as a parallelogram with a transverse link.
- **Collar enclosed the pillar.** The knuckle's collar was a solid cylinder
  around the fixed pillar: they collided in MuJoCo (the gate reported the
  contact) and the knuckle was not connected to anything. It is now a ring
  around the carrier, a declared turning fit.
- **Front wheels hit the hull at full lock.** The sills and floor ran past the
  wheels. The hull now ends 0.4 m behind the front wheels, with narrower
  horns carrying the pillars and the radiator, as a torpedo body does.
- **Rear axle through the sills.** The sourced transverse tunnel means the
  axle passes through the body sides: the sills are split at the axle, with
  short rear sills behind it.
- **Steering box.** The pitman arm sat inside the box, and at full lock it
  swung into a horn; the box moved up and the whole linkage down.
- **Propeller shaft on the floor.** The sourced tunnel is open-bottomed, so
  the floor is two panels beside it.
- **Engine through the bulkhead.** The engine moved forward onto the horns
  and the bulkhead became two side pieces, open for the gearbox.
- **Steering wheel through the body side.** The sides now stop at the
  driver's elbow.
- **Brake anchors.** The front anchor is a 2 mm disc between the collar
  (1.5 mm) and the wheel's swept envelope (5.5 mm): the tightest spot on the
  car. The rear anchors moved inboard of the hub carriers.
- **Pedals and levers** moved to the driver's side and ahead of the seat;
  several hull parts that touched (arms into the cross member, cap on the
  arm, rear floor on the cross member) were declared.

## Result (MuJoCo 3.14.0, full build `f7357532…`)

**Chassis stage** (`df56de7d…`)
- 13,643 pairs at 173 poses, all 80 joins touching, nothing disconnected.
- Gate: 6.39 m.
- All three planted defects caught.

**Full build checks**
- 38,014 pairs at 173 poses (121 steering angles, plus four suspension
  extremes each combined with the steering sweep).
- All 220 joins touching; nothing disconnected.
- All three planted defects caught.
- Closest pairs: front brake anchor to the knuckle collar 1.5 mm (static), and
  to the wheel envelope 5.5 mm (moving).

**Downhill gate**
- 6.39 m travelled, 0.01 mm sideways.
- Loops (tie rod, drag link, propeller shaft) closed within 9 µm.
- Static deflection 80 mm front and 56 mm rear; travel used 14%.
- Blocked-wheel control caught.

| Scenario | Result |
| --- | --- |
| Steered descents ±20°, pedal at 3 s | Yaw 45.7° and −47.6°, against 46.9° and −49.0° kinematic. Stops and holds. Steering torque 37.6 N m at the pitman (4.7 N m at the wheel). |
| Hand brake (rear drums), 5° | Stop 3.51 m / 4.00 s from 1.56 m/s; 1.2 mm hold drift; half dt 3.511 m. Holds before release and rolls after. Unbraked control fails. |
| Pedal (all four drums), 5° | Stop 1.15 m / 1.24 s; 1.3 mm hold drift; half dt 1.154 m. Three times shorter than the rear drums alone. |
| Four-wheel braking in a 20° turn, 5° | Yaw 30.0° against 30.9° kinematic; every wheel rolls (slip below 0.4%); stops and holds. |
| Weight transfer, pedal from third gear on the level | The front tyres' share of the load rises from 45.6% to 50.2% (5,374 N to 5,915 N); the rear wheels keep rolling. |
| Launch through three gears, 40 s, pure-pursuit driver | 4.157 m/s at 2 s against 4.144 reference; 900.6 m against 902.4 m. Governed 8.4, 15.1 and 28.7 m/s; final 30.0 m/s (108 km/h, no-load). 36.0 kW peak. |
| 15° hill start, first gear | 119.1 m against 119.0 reference; no rollback. Predicted limit 21.2°. |
| 8° hill start, third gear (negative control) | Rolls back 3.13 m against 3.17 reference. Predicted limit 6.1°. |
| Powered turns, first gear, ±7° | Radius 25.5 and 24.1 m, 0.30–0.32 g. Differential ratio 1.056 against 1.056, and 1.060 against 1.060. Tyre loads steady. |
| Locked differential (negative control) | 1.000 against 1.056, so caught. |
| Engine against the hand brake, third gear | 476 N m of drive held by 600 N m of brake; the car stops. |
| Road bump, 40 mm, first gear (8.8 m/s) | All wheels on the ground; peak travel 77–78% of each pillar's and 84% of the rear axle's; 1.0° peak tilt. |
| Half timestep, launch | Changes of 0.005% in speed and 0.03% in distance. |

**Findings (informational)**
- *Engine against the hand brake in first gear:* 1,618 N m of drive beats
  600 N m of brake. The driver must declutch.
- *One wheel up, the other down:* the tie rod tilts and steers the right wheel
  2.0°. Bump steer with both wheels up is 0.59°.
- *Fixed-hands launch:* drifts 1.0 m over 900 m, the smallest of the sprung
  cars (the Mercedes 10.8 m, the Model T 7.1 m), which is what the
  second-order geometry predicts.
- *Ackermann:* up to 1.6° over ideal at full lock.
- *Bonnet:* it sits above the pillar arms with no valance below it, a
  cosmetic gap.

## Bumpy road: what the independent front buys

```bash
.venv/bin/python experiments/011_lancia_lambda_1922/bumpy_road.py [--record]
```

The Model T's bump course (bars of 30–50 mm, some under one wheel only, then
a 15 mm washboard), driven in first gear at about 8.4 m/s with the
pure-pursuit driver. Three cars: the Lambda as built, the Lambda with its
pillars and rear axle welded to the hull, and the Model T with its beam axles
(in low gear at 6.4 m/s: a lighter, slower car, so its column is for
comparison only). The checks were fixed before the first run and all pass
(`output/full/physics/bumpy_road.json`, video `bumpy_road.mp4`).

| | Lambda, as built | Lambda, suspension locked | Model T (beam axles) |
| --- | --- | --- | --- |
| RMS vertical acceleration at the driver's seat | 0.21 g | 1.25 g | 0.22 g |
| Peak vertical acceleration at the seat | 0.84 g | 8.4 g | 1.21 g |
| Time on the course with a wheel off the ground | 25% | 79% | 25% |
| Peak body roll over the one-sided bars | 0.9° | 2.1° | 1.4° |
| Peak travel used, front / rear | 77% / 83% | — | 50% / 86% |

The one-sided bars are where the sliding pillars show. A beam axle must tilt
the whole axle, and with it the body, when one wheel rises; the Lambda's body
rolls 0.9° against the Model T's 1.4°, at a higher speed and with a longer
wheelbase. The peak jolt at the seat is 30% lower. Locking the suspension
turns the ride into 8 g peaks with the wheels off the ground most of the time.

**Caveat:** the shared tyre contact is stiffer than a real 1920s beaded-edge
tyre, so wheel lift is overstated for every car here. Seat acceleration is
the raw vertical value at the seat point, not a comfort weighting.

## Not modeled

- The damper inside the pillar, and the friction between the pillar and its
  carrier: joint damping and joint friction stand in for them.
- Leaf-spring wind-up and shackles at the rear.
- The engine and gearbox internals, the clutch's slip beyond the engagement
  ramp, and reverse.
- The unitary body's stiffness (the hull is rigid), and the doors.
- Pneumatic tyre behaviour beyond the shared compliant contact.
- Rolling resistance and drag, which matter at 30 m/s. Top speed is set by
  the engine's droop, not by drag.
- Occupants, lamps, and the windscreen.
