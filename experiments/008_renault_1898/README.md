# 008 — Renault Type A Voiturette (1898): shaft drive to a live axle

Louis Renault's first car replaced chains with direct drive. The engine is at the
front, followed by a cone clutch, a three-speed gearbox whose third gear is a
straight-through "prise directe", then a propeller shaft with universal joints
to a live rear axle with a bevel differential. It is built gate-first on the
006 tooling.

**Sourced** [S14, S15]:
- De Dion-Bouton single at the front, 270–273 cc, 1.75 hp.
- Three speeds plus reverse, with direct drive in third.
- Cardan shaft, rear axle and differential.
- Handlebar steering ("guidon").
- One pedal that both brakes and declutches.
- 1.90 × 1.15 m overall.
- 250 kg per S14; S15 gives 200 kg.
- 32 km/h per S15; S14 gives 50 km/h.

**Assumptions:**
- Wheelbase 1.25 m, track 0.95/1.00 m, wire wheels on pneumatic tyres.
- 1500 rpm governed, giving 8.2 N m.
- Gearbox 3.0/1.8/1.0 with a 5.8 bevel, so direct third gives 32 km/h.
- The pedal works a drum on each rear wheel at 100 N m each. The Velo showed
  that a single brake through the differential skids the inner wheel in turns.
- Double-pivot front axle from 006, and the bodywork.
- Reverse gear and suspension are not simulated.

## Commands

As for 007, with `008_renault_1898` and `renault_1898`:

```bash
blender --background --factory-startup --python-exit-code 1 --python experiments/008_renault_1898/generate.py -- \
  --config experiments/008_renault_1898/parameters.json --output experiments/008_renault_1898/output/chassis --stage chassis
.venv/bin/python experiments/008_renault_1898/run.py --stage chassis
# ... --stage full --render, then run.py --stage full, record.py, and render_views.py with views.json
```

## New mechanisms and how they are checked

- **Rotating drive parts:** the propeller shaft, both universal joints and the
  pinion shaft are swept solids. They must clear everything by 5 mm, except
  where they meet inside a joint, which is declared.
- **Live axle:** the housing is part of the chassis (no suspension). The
  half-shafts spin inside it, so the axle mounts stand on top of the housing.
- **Combined pedal:** in first and in third gear, pressing the pedal must cut
  drive torque to zero before braking, then stop and hold the car.

## Found and fixed while building

- **Before the first build (hand clearance check):**
  - Front hubs touched the knuckle webs; kingpin inset 0.10 → 0.11 m.
  - At full lock the front tyres reached the bonnet sides; the bonnet now
    stands on top of the rails.
  - The pedal passed through the footboard.
- **Steering sweep:** at −30° the inner tyre came within −2.9 mm of the rail.
  The frame was narrowed from 0.48 to 0.44 m, which pushed the column bearing
  into the rail, so the column moved to y = −0.16. The pedal shaft then grazed
  the bearing, so the column moved to x = 0.82.
- **Build checks:**
  - The axle mounts pierced the spinning half-shafts.
  - The front universal joint sat 2 mm from the gearbox.
  - The shafts meeting inside the universal joints were not declared joined.
  - The pedal clipped the gearbox.
  - The engine bearers poked into the bonnet.
  - The pedal shaft did not touch the rails.
- **Powered turn moved to first gear.** In second gear this 250 kg car reaches
  about 5 m/s on a 3.6 m radius and needs about 0.56 g. It slides wide: yaw is
  16% short, the inner wheels unload, and the differential ratio no longer
  follows rolling kinematics. That run is kept as a grip-limit finding
  (`finding_turn_second_gear`); the gated turns run in first gear at about
  0.27 g. The copied scenario assumed the Panhard's 3.4 m/s.

## Result (MuJoCo 3.14.0, full build `2c3a1273…`)

**Chassis stage** (`b21842b4…`)
- 1,251 pairs checked, all 51 joins touching.
- Gate: 6.37 m.
- All three planted defects caught.

**Full build checks**
- 11,780 pairs at 121 steering angles.
- All 115 joins touching; nothing disconnected.
- All three planted defects caught.
- Closest pairs: footboard to gearbox cradle 4.9 mm (static); knuckle web to
  axle eye and brake anchor to drum 6 mm.

**Downhill gate**
- 6.37 m travelled, 0.02 mm sideways.
- Rolling error 0.25%. The tyre sags about 1 mm under load, so the rolling
  radius is slightly smaller than the nominal one.
- Loops closed within 1 µm.
- Blocked-wheel control caught.

| Scenario | Result |
| --- | --- |
| Steered descents ±20°, pedal brakes at 3 s | Yaw 66.0° and −70.7°, against 67.6° and −73.6° kinematic. Stops and holds. Handlebar torque 6.1 N m. |
| Rear drums, 5° | Stop 1.18 m / 1.3 s from 1.59 m/s; 5.9 mm hold drift; half dt 1.177 m. Unbraked control fails. |
| Launch 1st → 2nd at 4 s → direct 3rd at 8 s, heading-holding driver | 3.093 m/s at 3 s against 3.098 reference; 210.4 m against 211.3 m in 30 s. Governed 2.98, 4.97 and 8.94 m/s; final 9.27 m/s (33 km/h) at no-load. 1290 W peak. |
| 8° hill start, first gear | 42.35 m in 20 s against 42.37 reference; no rollback. Predicted limit 10.2°. |
| 5° hill start, third gear (negative control) | Rolls back 2.62 m against 2.63 reference. Predicted limit 3.4°. |
| Powered turns, first gear, ±20° | Differential ratio 1.306 against 1.317, and 1.354 against 1.370. Kinematic yaw within 4.3%. Tyre loads steady within 0.9%. |
| Locked differential (negative control) | 1.000 against 1.317, so caught. |
| Pedal pressed while driving, first and third gear | Drive torque 0 while braking; stops and holds (143 or 48 N m of drive would otherwise have been available). |
| Half timestep, launch | Changes of 0.002% in speed and 0.04% in distance. |

**Findings (informational)**
- *Second-gear turn:* 0.56 g, sliding, as above.
- *Fixed-hands launch:* drifts 0.64 m over 210 m, the same linkage asymmetry
  as 006 and 007.
- *Ackermann:* up to 1.9° over ideal at full lock.

## Not modeled

- Suspension (the live axle is rigid to the frame).
- Universal-joint speed fluctuation.
- Gear internals and reverse.
- Pneumatic tyre behaviour beyond the shared compliant contact.
- Brake cables and equaliser, which are not drawn.
- Rolling resistance and drag.
- Occupants.
