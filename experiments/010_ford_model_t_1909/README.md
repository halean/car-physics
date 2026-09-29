# 010 — Ford Model T (1909 touring): transverse springs, torque tube, planetary two-speed

The Model T is the car that put the world on wheels. Each beam axle hangs from a
single **transverse** semi-elliptic spring and is located by a **ball**:
- the front axle by a wishbone of radius rods to a ball under the crankcase;
- the rear axle by the torque tube, whose ball sits behind the transmission.

Drive goes through a pedal-worked **two-speed planetary** transmission, the
torque tube and a live axle. The only service brake is the transmission brake.
It is built gate-first on the 006 tooling and the 009 suspension checks.

**Sourced** [S17], via Wikipedia:
- 2.9 L four, 20 hp; top speed 68 km/h.
- Two-speed planetary transmission; three floor pedals (low, reverse,
  transmission brake) and a lever (high-speed clutch and parking brake).
- Torque tube to the rear axle.
- Transmission brake only (no service brakes on the wheels); the lever works
  bands on the rear hubs.
- A transverse semi-elliptic spring on each beam axle.
- Wheelbase 2,540 mm, track 56 in (1,422 mm), 30 in wheels, artillery wheels.
- 540–750 kg.

**Assumptions:**
- 545 kg (the low end of the range; early tourings were lightest).
- Engine: rated at 1600 rpm with a droop to 1700 rpm no-load. The real car had
  a hand throttle, not a governor.
- Planetary low 2.75:1 and a 40:11 bevel (3.636:1). With these, high gear
  gives 67 km/h, which matches the sourced top speed.
- Brakes: 200 N m per hub band, and 600 N m at the wheels for the transmission
  brake.
- Springs: about 68 mm (front) and 78 mm (rear) static deflection, with damping
  at 0.33 of critical (interleaf friction; the car had no dampers).
- Frame width, heights, the ball positions and the steering layout.
- The whole body.
- Reverse is not simulated.

## Commands

As for 009, with `010_ford_model_t_1909` and `model_t`:

```bash
blender --background --factory-startup --python-exit-code 1 --python experiments/010_ford_model_t_1909/generate.py -- \
  --config experiments/010_ford_model_t_1909/parameters.json --output experiments/010_ford_model_t_1909/output/chassis --stage chassis
.venv/bin/python experiments/010_ford_model_t_1909/run.py --stage chassis
# ... --stage full --render, then run.py --stage full, record.py, and render_views.py with views.json
```

## New mechanisms and how they are checked

- **Axles on balls.** Each axle has two joints:
  - a swing hinge about its ball;
  - a roll hinge about the line from the ball to the axle centre, so the ball
    stays put as the axle rolls.

  The shared 3D linkage solver (`linkage.suspended_pose`) now handles a
  swinging front axle and a tilted roll axis. The Mercedes results were
  unchanged by that generalisation. Build checks run at bump, rebound and roll,
  combined with the full steering sweep (145 poses). Each ball is a declared
  join to its socket that must touch at every pose.
- **Transverse springs.** Each spring is a rigid arched tube clamped under its
  crossmember. It rests on perches on the axle (`rest_joins`), and the joints
  carry its flex. The spring preloads are calibrated to ride height.
- **Enclosed rotating parts.** The crankshaft, flywheel magneto and planetary
  gears (in the case), the drive shaft (in the torque tube) and the half-shafts
  (in the housing) are not drawn, so there are no swept solids to check. The
  wheels, hub drums and linkage are checked as before.
- **Left-hand drive.** The steering gear sits outboard of the left rail. That
  keeps the pitman and drag link clear of the wishbone rods, which converge
  under the engine. The drag link runs at the roll-axis height. The Model T's
  4:1 reduction is a planetary set under the wheel; here it is the shared
  joint coupling.

## Found and fixed while building

- **Shared inertia bug (affects every car since 006).** The first spring
  calibration asked for impossible preloads: the rear wheels lifted off the
  ground while the axle sat on its stop. The cause was the rear axle's centre
  of mass, which the model placed at x = 12 m. The shared `part_inertia` was
  missing a division by mass, so every small body's centre-of-mass offset was
  scaled by its mass in kilograms. After the fix, 006–009 were revalidated
  (all pass) and re-recorded; their validation notes give the current figures.
- **Front hubs** touched the knuckle webs (0 mm); the front hub was shortened
  from 60 to 50 mm.
- **Road bump bottomed** the front axle (100.2% of its 60 mm travel). The Model
  T was known for long axle travel, so the front travel was raised to 75 mm.
  The wishbone rods moved inboard (±0.35 m) to stay clear of the arched spring
  at full bump. The bump now uses 80%.
- **Brake-anchor clearance** was set to 7 mm, and the rear perches were moved
  inboard of the hub drums, before the first build.

## Result (MuJoCo 3.14.0, full build `a5a8710b…`)

**Chassis stage** (`e594b932…`)
- All joins touching, nothing disconnected.
- Gate: 6.11 m.
- All three planted defects caught.

**Full build checks**
- 35,964 pairs at 145 poses; all 199 joins touching; nothing disconnected.
- All three planted defects caught.
- Closest pairs: dash to floor and crossmember to radiator 5.0 mm (static);
  differential to rear spring 6.6 mm at full bump.

**Downhill gate**
- 6.12 m travelled, 0.09 mm sideways.
- Loops closed within 2.4 µm.
- Static deflection 68 mm front and 78 mm rear.
- Blocked-wheel control caught.

| Scenario | Result |
| --- | --- |
| Steered descents ±20°, foot brake at 3 s | Yaw 38.8° and −41.2°, against 40.0° and −42.7° kinematic. Stops and holds. Steering torque 22.5 N m at the pitman (5.6 N m at the wheel). |
| Hub bands (hand lever), 5° | Stop 1.56 m / 1.80 s; 1.8 mm hold drift; half dt 1.560 m. Holds before release and rolls after. Unbraked control fails. |
| Transmission brake (foot), 5° | Stop 0.91 m / 1.02 s; 2.0 mm hold drift; half dt 0.911 m. |
| Launch: low, then high at 3.7 s, 40 s, pure-pursuit driver | 4.800 m/s at 2 s against 4.788 reference; 634.9 m against 637.2 m. Governed 6.4 and 17.6 m/s; final 18.6 m/s (67 km/h, no-load). 14.9 kW peak. Lateral 0.15 mm. |
| 15° hill start, low | 106.8 m against 107.1 reference; no rollback. Predicted limit 25.9°. |
| 12° hill start, high (negative control) | Rolls back 4.54 m against 4.56 reference. Predicted limit 9.1°. |
| Powered turns, low, ±10° | Radius about 14 m, 0.33 g. Differential ratio 1.106 against 1.104, and 1.107 against 1.106. Tyre loads steady. |
| Locked differential (negative control) | 1.000 against 1.104, so caught. |
| Engine against the hub brakes, high | 323 N m of drive is held by 400 N m of brake; the car stops. |
| Road bump, 40 mm, low (4.8 m/s) | All wheels on the ground; peak travel 80% front and 81% rear; 1.2° peak tilt. |
| Half timestep, launch | Changes of 0.001% in speed and 0.03% in distance. |

**Findings (informational)**
- *Bump steer:* 2.4° at the front wheels straight ahead at full bump (75 mm),
  rising to 5.5° at full lock. Roll steer is 1.3° (1.7° worst). The drag link
  is only 0.22 m long, because the pitman sits outboard and the ball arm is
  close to the knuckle, and bump steer is second order in link length. The
  Mercedes' 0.84 m transverse link gives 0.6°. The gated scenarios, including
  the bump and the 18.6 m/s launch, stay stable. A real Model T's geometry
  differs and was not sourced.
- *Engine against the hub brakes in low:* 889 N m of drive beats 400 N m of
  brake. The driver must release the low pedal.
- *Fixed-hands launch:* drifts 7.1 m over 635 m.
- *Ackermann:* up to 1.9° over ideal at full lock.
- *Foot brake in a turn:* works through the open differential; no wheel slid.

## Not modeled

- Spring wind-up and interleaf friction. The joints locate the axles; the
  springs are joint stiffness.
- The planetary gear internals, the band slip of each speed, the magneto and
  reverse gear.
- Pneumatic tyre behaviour beyond the shared compliant contact.
- Rolling resistance and drag. Top speed is set by the engine droop.
- Occupants, the windscreen, the top and the lamps.
