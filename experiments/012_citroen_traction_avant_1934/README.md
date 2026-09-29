# 012 — Citroën Traction Avant (1934): front-wheel drive, torsion bars, unitary body

The Traction Avant is the first car in this series whose **driven wheels are
also its steered wheels**. Its gearbox sits ahead of the front axle and the
engine behind it. Each front wheel hangs on wishbones sprung by a lengthwise
torsion bar, and each half-shaft ends in a double-Cardan joint centred on
the kingpin, so the wheel can steer while it is driven. The body is welded
steel with a flat floor: nothing drives the rear, so there is no tunnel.
It is built gate-first on the 006 tooling with the independent-suspension
checks from 011.

**Sourced** [S21–S24]:
- 7A of April 1934: 1303 cc, 32 PS; three speeds with the lever through the
  dashboard; about 95 km/h.
- Front-wheel drive; the transmission ahead of the engine and the front axle.
- Six universal joints in the drive shafts; the outer ones double-Cardan with
  a centring ball, at the hubs.
- Welded unitary body with a flat floor.
- Front: wishbones with lengthwise torsion bars. Rear: a beam axle on
  trailing arms with torsion bars on a 75 mm steel tube, and a Panhard rod.
- Hydraulic brakes on all four wheels.
- Worm-and-roller steering (rack and pinion only from May 1936).
- Wheelbase 2.91 m.

**Assumptions:**
- 1000 kg, no occupants; about 60% on the front tyres.
- Track 1.34 m front and 1.30 m rear; wheels of 0.34 m radius on 65 mm tyres.
- 23.5 kW rated at 3200 rpm with a droop to 3400 rpm no-load; gearbox
  3.0/1.7/1.0 with a 4.4:1 final drive (93 km/h at rated speed in top).
- Brakes: 300 N m at each front drum and 220 N m at each rear, all through
  the pedal; the hand lever works the rear drums only.
- Springs: 28 kN/m at each front wheel (4480 N m/rad of torsion-bar twist
  through the 0.40 m arm) and 38 kN/m at the rear axle, with damping at about
  0.3 of critical standing in for the shock absorbers (their type is not
  established).
- **Linkage geometry.** The wishbones are equal and parallel (a
  parallelogram). The steering box sits on the left front suspension tower;
  its sector shaft runs lengthwise, so the drop arm swings sideways and its
  ball moves across the car like a rack end. Split track rods run from that
  ball to steering arms behind the wheels. The sources do not give this
  layout.
- Left-hand drive, and all the bodywork dimensions.
- The Panhard rod is not drawn: the model's axle has no sideways freedom
  for it to locate.

## Commands

As for 011, with `012_citroen_traction_avant_1934` and `traction_avant`:

```bash
blender --background --factory-startup --python-exit-code 1 --python experiments/012_citroen_traction_avant_1934/generate.py -- \
  --config experiments/012_citroen_traction_avant_1934/parameters.json --output experiments/012_citroen_traction_avant_1934/output/chassis --stage chassis
.venv/bin/python experiments/012_citroen_traction_avant_1934/run.py --stage chassis
# ... --stage full --render, then run.py --stage full, record.py, render_views.py with views.json, and
# animate_steering.py with animation_hide.json
```

## New mechanisms and how they are checked

- **Wishbones.** Each lower arm is a body on a lengthwise hinge carrying the
  torsion-bar spring. The upper arm is a second hinged body, closed to the
  upright by a connect constraint (the real four-bar loop). The upright
  hangs from the lower arm on its own hinge, and the knuckle turns on the
  upright. The shared solver (`linkage._front_travel`) moves each upright on
  its arc: it rises by h and moves inboard by L(1 − cos a).
- **Driven, steered wheels.** The adapter's differential simply drives the
  front wheel joints; each wheel spins relative to its knuckle, which is what
  a constant-velocity joint allows. The half-shaft is drawn as a body on its
  own hinge, coupled to the lower arm so that it stays parallel to the
  wishbones. It is a swept solid in the build checks. Its outer joint sits on
  the kingpin inside the knuckle housing, a declared contact exclusion
  (`excluded_pairs`, new in the adapter).
- **Drop-arm steering.** The solver (`centre_drop_arm`) turns the drop arm
  about its lengthwise shaft and solves each track rod to its own knuckle.
  The rods are ball-jointed bodies on the drop arm, and each is closed to its
  knuckle.
- **New gate: climb load shift.** Climbing 15° in first gear, the front
  tyres' share of the load must fall, and the driven wheels must not spin.
  With front-wheel drive, the hill takes load off the driving wheels.
- **Differential check for front drive.** The expected outer-to-inner ratio
  now uses the front wheels' paths about the turn centre on the rear-axle
  line.

## Found and fixed while building

- **Wishbone legs** met at the ball joint and were not declared joined.
- **The knuckle housing sat inside the wheel's swept volume.** The kingpin
  moved 20 mm inboard and the housing was slimmed.
- **The rear suspension tower was in the wheel's path at full lock and full
  droop.** It is now narrower, with a bracket up to the longeron above the
  tyre's reach.
- **The rigid trailing arms dipped into the floor** when the axle rolls. The
  floor now starts behind their pivots.
- **The upright's webs and bosses touched the ball joints.** The balls were
  lengthened and the webs moved.
- **The two track rods were touching in MuJoCo.** The new exclusion list was
  not reaching the exported geometry; the exporter now passes it.
- **The planted loose-join test moved nothing.** Box parts' colliders carry
  no `:n` suffix, so the test's name match missed them. It now uses a rod
  part.
- **Brake anchors and the outer joint were inside the wheels' swept volume**,
  the A-pillars ran into the scuttle, the pedals were drawn twice, and the
  steering column passed 2.2 mm from an engine bearer (5 mm needed).

## Result (MuJoCo 3.14.0, full build `46aaec7c…`, procedural body after the photographs)

**Chassis stage** (`09fb4fb6…`)
- 5,920 pairs at 173 poses; all 103 joins touching; nothing disconnected.
- Gate: 6.40 m.
- All three planted defects caught.

**Full build checks**
- 1,085,749 pairs at 173 poses (the body's hull colliders included); all 204 joins touching; nothing disconnected.
- All three planted defects caught.
- Closest pairs: front brake anchor to knuckle housing 2.0 mm (static);
  front bumper iron to cross member 4.2 mm.

**Downhill gate**
- 6.40 m travelled, 0.05 mm sideways.
- Loops (upper wishbones, track rods) closed within 59 µm.
- Static deflection 92 mm front and 81 mm rear; travel used 16%.
- Blocked-wheel control caught.

| Scenario | Result |
| --- | --- |
| Steered descents ±20°, pedal at 3 s | Yaw 49.4° and −48.0°, against 50.4° and −49.4° kinematic. Stops and holds. Steering torque 47.6 N m at the sector shaft (4.0 N m at the wheel). |
| Hand brake (rear drums), 5° | Stop 3.68 m / 4.18 s; 1.4 mm hold drift; half dt 3.679 m. |
| Pedal (hydraulic, all four drums), 5° | Stop 0.89 m / 0.94 s; 1.1 mm hold drift; half dt 0.893 m. Four times shorter than the rear drums alone. |
| Four-wheel braking in a 20° turn, 5° | Yaw 30.6° against 31.3° kinematic; every wheel rolls (slip below 0.5%); stops and holds. |
| Weight transfer, pedal from third gear on the level | The front share of the load rises from 58.7% to 64.0%; the rear wheels keep rolling. |
| **Climb load shift**, first gear, 15° (new) | The front share falls from 60.3% on the level to 55.1% on the climb; driven-wheel slip 0.7%. Grip-limited grade from the measured loads: 22.7°. |
| Launch through three gears, 40 s, pure-pursuit driver | 3.195 m/s at 2 s against 3.185 reference; 771.7 m against 774.3 m. Governed 8.6, 15.2 and 25.9 m/s; final 27.4 m/s (98 km/h, no-load). 23.5 kW peak; 2 cm off the line. |
| 12° hill start, first gear | 102.1 m against 101.7 reference; no rollback. Predicted limit 16.1°. |
| 8° hill start, third gear (negative control) | Rolls back 4.47 m against 4.48 reference. Predicted limit 5.3°. |
| Powered turns, first gear, ±7° | Radius 22.7 and 23.8 m, 0.35–0.37 g. Front differential ratio 1.060 against 1.060, and 1.057 against 1.057. |
| Locked front differential (negative control) | Caught by `turns_correct_way`: the car cannot be steered (the column reaches 0.03° of the 7° asked; yaw 1.3°). |
| Engine against the hand brake, third gear | 309 N m of drive held by 440 N m of brake. |
| Road bump, 40 mm, first gear (9.1 m/s) | All wheels on the ground; peak travel 60% front and 80% rear; 0.9° tilt. |
| Half timestep, launch | Passes. |

**Findings (informational)**
- *A locked front differential makes the car unsteerable.* Because the
  kingpin is 0.12 m inboard of the tyre (the scrub radius), steering rolls
  one tyre forward and the other backward about their kingpins, and a spool
  forbids that. This is one reason front-wheel drive needed a differential
  and constant-velocity joints.
- *A top-speed weave with a short-sighted driver.* Coasting at the governor
  (about 27 m/s), a pursuit driver aiming 0.8 s ahead excites a growing
  weave: ±2 m by 55 s. At 1.2 s it still grows slowly; at 1.6 s (44 m ahead)
  the car holds within 3 cm. While the front wheels are pulling, even the
  0.8 s driver holds within 1 cm. This car uses a 1.6 s preview; the earlier
  cars use 0.8 s. Increasing the steering servo's damping made it worse, so
  it is not steering lag.
- *15° hill start in first:* it climbs with no rollback, but lands 6.6% off
  the 1-D reference (30.5 m against 28.6 m). 15° is 93% of the predicted
  grade limit, so the net force is only 7% of the drive. The simulated tyres
  roll at a radius 0.5% under nominal, and that small thrust difference is
  magnified about 13 times. The gate is at 12° (74% of the limit, like the
  Lambda's 71%).
- *Bump steer.* The track rods are unequal in length (the box sits left of
  the drive line), so both wheels steer the same way as they rise or fall:
  1.06° at full travel straight ahead. At full lock with both wheels at full
  droop it reaches 6.0°: the drop arm's arc lifts its ball 20 mm at full
  lock and tilts the short left rod. Ackermann is within 1° of ideal.
- *Hands-off launch:* drifts 13.3 m over 770 m.
- *Engine against the hand brake in first gear:* 926 N m of drive beats
  440 N m of brake.

## Procedural bodywork

The saloon body is built from parametric curves by the shared
[bodywork.py](../006_benz_velo/bodywork.py), following
[skills/procedural-bodywork](../../skills/procedural-bodywork/SKILL.md). The
design tables are in [body.py](body.py). The shapes are styled on seven
photographs of 1934 7As [S25]; no drawing or body dimension is sourced
except the wheelbase.

**What the photographs changed** (a first version was styled from general
knowledge):
- **A short, steep, rounded back.** The body now ends 0.68 m behind the rear
  axle, not 1.08 m, with a domed tail. The overall length is about 4.56 m.
- **A long flat roof with a small visor**, and a higher belt line (1.04 m)
  continuing the bonnet top, so the side glass is 0.36 m tall, not 0.44 m.
- **A bonnet that tapers in plan** from 0.43 m half-width at the scuttle to
  0.25 m at a tall, narrow grille shield. The shield has a rounded top, leans
  back 0.10 m, and has a chrome surround (the maker's chevrons are left
  off). The bonnet top is flatter (`n_up` 4.5), and its sides come down to
  the wing tops.
- **Full front wings**, reaching forward and down to bumper level, with a
  deep outboard skirt (0.16 m) and a wide, shallow inboard side that runs to
  25 mm outboard of the longerons, above the steered tyre's reach.
- **Rear flares outboard of the body** (the rear track is narrower than the
  body), with a larger crown radius (0.52 m) covering the body's wheel
  opening.
- **Pressed-steel disc wheels** with chrome hubcaps; two headlamp bowls on
  stalks through the bonnet sides; two-tone paint (light grey body, black
  wings and wheels).

| Shell | Collision patches | Watertight and outward |
| --- | --- | --- |
| Body shell | 539 | yes |
| Bonnet | 85 | yes |
| Grille | (in the build report) | yes |
| Each wing | (in the build report) | yes |

Tolerance and panel thickness are both 2 mm; each patch is collided as its
convex hull, so clearances err on the safe side. Each revision was checked
before styling went further:
- The first version's six failures were packaging facts (lower sections too
  round to reach the sills, rear wings through the body side, and others).
- The photo revision's six were real attachments, declared as joins: the
  grille surround against the bonnet, and the headlamp stalks through the
  bonnet sides.
- One loose joint appeared when the flares grew: the rear wing stays moved
  past the ends of the wheel-arch panels, and moved back.

The full build check now covers 1,085,749 pairs, all passing. No scenario
result changed.

## Not modeled

- Shock absorbers beyond viscous joint damping; friction in the torsion bars.
- The Panhard rod (the axle has no sideways freedom), and the rear tube's
  twist in roll (the trailing arms are rigid with the axle).
- The universal joints' speed fluctuation: each driven wheel spins relative
  to its knuckle at a constant ratio, as a constant-velocity joint would.
- The engine, clutch and gearbox internals, and reverse.
- Pneumatic tyre behaviour beyond the shared compliant contact, rolling
  resistance and drag.
- Doors and door lines, glass, lamps, the grille's chevrons, the body's own stiffness, and occupants.
- The rear wheel openings are plain rectangles in the body side; side-on, their corners show at each end of the flares.
- Bonnet louvre doors, the windscreen frame and wipers, horns, and the chevrons on the grille.
