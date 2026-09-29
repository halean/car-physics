# Lessons learned: building and validating cars 001–011

What building the Motorwagen, Velo, Panhard, Renault, Mercedes 35 HP, Model T and
Lancia Lambda taught us, gathered in one place. Each lesson names the experiment where it was
learned; that experiment's README has the numbers. Read this before starting a
new car, alongside the [car-downhill-check skill](../skills/car-downhill-check/SKILL.md).

## 1. Vehicle engineering

### Brakes

- **A brake that acts through an open differential fails in a turn** (006). The
  unloaded inner wheel receives equal torque, so it skids and can spin
  backwards: 11% slip on a 20° descent. Give each wheel its own brake (drums or
  hub bands). A driveline brake is still useful in a straight line. On the
  heavier sprung cars (009, 010) the driveline brake in a turn did not slide,
  and that is kept as a finding.
- **Brake capacity has a window** (009, 010). It must exceed top-gear drive and
  the holding torque on the test slope, but stay below wheel lock. On the sprung
  Mercedes, 800 N m per drum pitched the body forward, unloaded the rear and
  locked the wheels (29% slide); 500 N m worked. Estimate lock as rear load ×
  μ × radius before choosing a capacity.
- **In low gear the engine out-pulls the brakes** (005, 007, 009, 010). This is
  true of real cars of the period too; the driver declutches. Gate the brake
  against the engine in top gear and record low gear as a finding.
- **Heavy cars with weak brakes stop slowly** (007). The 600 kg Panhard needed
  5.3 s to stop, so the test windows must be sized from each car's own
  predicted stop, not copied from the previous car.

### Drivetrain and gearing

- **Choose unsourced ratios to reproduce sourced performance** (008, 009, 010).
  With the governed engine speed and the wheel radius, pick the final ratio so
  that top gear gives the sourced top speed. Then predict the climbable grade in
  each gear and test just inside and outside it.
- **Check sourced figures against each other** (006, 009). The Velo's quoted
  torque, 4.4 N m, contradicts its own power rating. The Mercedes' quoted
  length, 2,766 mm, is shorter than its wheelbase plus both wheel radii
  (3.31 m). Do the arithmetic, keep the consistent value and record the
  conflict.
- **Grip limits the gated turns** (008). A second-gear turn needing 0.56 g slid
  wide and broke the differential-ratio check. Gate turns at about 0.25–0.33 g
  and keep faster turns as grip findings.
- **Keep chain centres constant on a sprung axle** (009). Locating the rear axle
  with radius rods pivoting on the countershaft keeps the chain length fixed as
  the axle moves.
- **Put the roll axis through the ball** (010). When an axle is located by a
  ball (the Model T's wishbone and torque tube), it rolls about the line from
  the ball to the axle centre. Model it that way, or the ball wanders sideways
  in roll.

### Steering

- **Jeantaud trapezoids under-steer the outer wheel** (006–010). With the tie
  rod aimed at the rear axle, the outer wheel ends up about 2° over ideal
  Ackermann at full lock. That is historically plausible; report it.
- **Linkage asymmetry makes a car drift with the wheel held still** (006–010).
  The drift was 0.2–0.6 m on the slow cars and 7–11 m over 700 m on the fast
  ones. Straight-line tests need a driver, and the fixed-hands run is kept as a
  finding.
- **A drag link from the frame to a sprung axle steers the wheels as the axle
  moves** (009, 010):
  - *Roll steer.* The Mercedes' first layout, a Velo-style diagonal link 0.10 m
    below the roll axis, gave 7.7° of roll steer. The car spun at 16 m/s with
    the wheel held straight; locking the axle's roll made the spin disappear,
    which confirmed the cause.
  - *The fix:* a transverse drag link at roll-axis height, which cut roll steer
    to 0.17°.
  - *Bump steer* is second order in link length. The Model T's 0.22 m link gives
    2.4° at full bump; the Mercedes' 0.84 m link gives 0.6°.
  - *In design:* keep the link long and at roll-axis height, and measure bump
    and roll steer before any powered run.
- **A straight line needs a driver** (010). With the wheel held straight, a
  launch leaves a small heading error that nothing removes: 0.76° became 6.6 m
  of drift over 600 m. Fixing its main cause, bump steer, exposed the next one,
  the steering's reversibility under linkage inertia. Steering friction traded
  that for a dead band. Without castor there is no self-centring, so the
  "driver holds the line" controller, like a real driver, is the fix. The
  experiments are tabled in the 010 README.
- **Match the drag link to the wheel's travel** (011). With a sliding pillar
  the wheel moves straight up. A lengthwise drag link whose end swings
  sideways then steers the wheel at first order (2.5° at 60 mm); a transverse
  link at the ball's height, with equal parallel pitman and drag arms, makes
  it second order (0.59°). The general rule: the link's end should move the
  way the wheel travels, so the travel only tilts the link.
- **Sliding and turning fits are joins, and must not be solid colliders one
  inside the other** (011). A knuckle collar drawn as a cylinder around the
  fixed pillar collided in MuJoCo (grandparent bodies are not excluded). A
  ring around the carrier, declared as a join, touches at every pose and
  connects the part to the rest of the car.
- **Four-wheel brakes change what to gate** (011). Braking in a turn becomes
  a gate rather than a finding, and the weight transfer that makes front
  brakes do the work is measurable from the contact forces: the front tyres'
  share rose from 46% to 50% while the rear wheels kept rolling.
- **Steered wheels need room at every suspension pose** (009, 010). At full lock
  combined with bump or roll, the Mercedes' front tyres reached the frame rails,
  so the frame was narrowed from 0.76 m to 0.66 m. Hubs have touched knuckle
  webs twice (008, 010). Check with the swept wheel at the extreme poses, not
  at rest.

### Suspension

- **Choose travel and damping with a road bump** (009, 010). A 40 mm bump took
  the Mercedes to 98% of its front travel even after its damping was doubled.
  It bottomed the Model T (100.2%) until the travel was raised to 75 mm, which
  fits that car's known long travel. Fix the design (travel, rate or damping);
  never the "does not bottom" limit.
- **Calibrate spring preload to ride height, then check the result** (009, 010).
  Springs are joint stiffness with a calibrated `springref`, and the gate
  requires 20–150 mm of static deflection. An implausible deflection, or a
  calibration that will not converge, means a model error somewhere else.
  Section 2 describes the one found this way.

## 2. Simulation and modelling pitfalls

### What a rigid-body simulation does not check

- **It cannot see disconnected parts or intersections within one body**
  (001–010). MuJoCo treats everything on one body as welded and never collides
  it with itself. Hence the declared joins, which must touch at every pose, and
  the clearance rule: every other pair stays 1 mm apart, or 5 mm if moving,
  same body included. Rotating parts are checked as swept solids. Each check
  has a planted defect that it must catch.
- **Parent and child bodies do not collide by default.** Disable
  `filterparent`, exclude only pairs connected by a joint, and check those pairs
  kinematically across the full steering and suspension range.

### Tyre contact (the shared adapter, in the order the problems appeared)

1. **Flat, stiff cylinders chatter** (006). In 16% of cornering steps there was
   no contact at all, and loads reached 2.6 × weight. Use an ellipsoid crown,
   plus a separate wide envelope that never touches the road, for clearance
   checks.
2. **Near-rigid contact jitters** (007): about 95 Hz load noise, with 0.1 mm of
   sag. Use a stated compliance per unit mass (solref −2300 −48, about 6 Hz,
   1–1.5 mm sag).
3. **Compliant friction creeps** (007): braked cars crept at 1.5 cm/s. The
   no-slip pass fixed that but zeroed a lightly loaded tyre's normal force
   (009). An elliptic friction cone with impratio 50 fixes both.
4. **A contact margin makes the tyre push from a distance** (009). In MuJoCo
   3.14, margins from the two geoms add, and `gap` does not cancel them. With
   a 2 mm margin, loaded tyres rested at +0.1 to +0.7 mm above the road. Use no
   margin on tyres. Code that assumes `contact.dist <= 0` means touching is
   wrong whenever a margin is set.

### Mass and inertia

- **Test mass calculations with a mass other than 1 kg** (010). The shared
  `part_inertia` omitted `/mass`, so every centre-of-mass offset was multiplied
  by the body's mass. The Model T's rear axle landed at x = 12 m; the Mercedes'
  about 3.8 m forward. Every car since 006 still passed with it: the
  light linkage bodies moved little, and on the Mercedes the spring calibration
  absorbed the error. Symptoms: impossible
  spring preloads, wheels lifting off the ground at rest, joints parked on
  their stops. First check: the compiled `body_ipos` of every body.
- **Don't lump linkage mass at a joint** (006). A tie rod's mass at one ball
  pushed the steering under acceleration. Distribute each small body's mass
  over its colliders.

### Measurements and metrics

- **Don't mix units in a single metric** (009). The loop-closure metric took in
  the steering box's joint coupling, in radians, as if it were metres. Count
  connect constraints only.
- **Unwrap heading** (005). Turns that complete a circle wrap the yaw back to
  zero.
- **Measure hold windows from the car's own stop** (006), not from a fixed time
  inherited from another car.
- **Compare against a 1-D reference model and repeat at half the timestep**
  (005–010). Agreement within 3%, and half-timestep changes under 1%, showed
  that the launches and hill starts are physics, not solver artefacts.

### Drivers and controls

- **The steering servo must be stiff enough** (006). kp 150 lagged 5° under
  load; kp 1000 holds.
- **Fixed-gain heading hold goes unstable at speed** (009). It was fine at
  3–9 m/s and oscillated above about 11 m/s. Pure pursuit, aiming 0.8 s ahead,
  works to 22 m/s.

### Visual evidence

- **Watch the videos; several bugs were found only there.** The force arrows
  exposed tyre chatter (006), the jitter (007) and the floating tyres (009, when
  the arrows vanished).
- **MuJoCo 3.14 draws arrows at half the length `mjv_connector` sets** (005).
  Double `size[2]`.
- **Average force arrows over 50 ms**, or instantaneous contact noise hides the
  physics. Scale them per car: 1 m per 1000 N suits a 280 kg car; a 1200 kg
  car needs about 1 m per 5000 N.
- **Check that every wheel has arrows in every panel** before accepting a
  recording.

## 3. Process

- **Gate first.** Every car's first failures (hub clearances, spring
  calibration, frame width) were cheapest to fix at the chassis stage, before
  there was any body to move.
- **The shared adapter has a blast radius.** Every car imports
  `006_benz_velo/vehicle.py`. After any change to it, rerun every car's chassis
  and full stages, re-record the videos, and note the revalidation with its
  current figures in each car's `validation.md`. This happened three times in
  009–010.
- **Fix the design or the test design, never the threshold.** Where a test was
  wrong (yaw wrap-around, windows too short, a metric with mixed units), fix it
  and say so. Where the car was wrong, change the car. Thresholds stay as they
  were set before the first run.
- **Separate gates from findings.** Behaviour that is real but historical
  (driveline brakes in turns, engines out-pulling brakes in low, grip limits,
  bump steer, drift) is reported as a finding, not tuned away.
- **When a result looks impossible, stop and find the cause.** "Rear wheels in
  the air while the axle sits on its stop" was a real bug in shared code, not
  something to calibrate around.
- **Not gated yet:** bump and roll steer are measured and reported, but they
  have no pass mark. The Model T's 2.4° suggests adding one, fixed before the
  next sprung car is built.
