---
name: car-downhill-check
description: Validate procedural car builds in Blender and MuJoCo, starting with an unpowered downhill rolling test before detailed modeling. Use when building a car, changing its geometry or mechanics, or checking a generated vehicle; includes collision, clearance, and optional braking checks.
---

# Car downhill check

Make downhill rolling the first functional milestone for a new car. Build the
minimum chassis, axles and wheels needed to test it before detailed bodywork,
trim or presentation. For an existing car, validate the changed build before
claiming it is fixed or complete. Saving draft artifacts is allowed throughout.

## Establish the assembly

- Read [car_tech/lessons_learned.md](../../car_tech/lessons_learned.md) first:
  it lists the failures earlier cars hit (brakes through a differential, roll
  and bump steer, tyre contact settings, inertia placement) and how each was
  caught.

- Identify the actual model, parameters, units, wheel layout, axle axes and
  moving/fixed parts. Never validate a previous car in place of the requested one.
- Declare mass, inertia, center of mass, tire/road friction and joint assumptions.
  Do not reuse the Motorwagen's three wheels or 135 kg blindly for another car.
- Record required attachments, intentional contacts, and forbidden intersections.
  Trace a connection from each wheel through its axle/fork to the chassis.
- Inspect side, front, top and perspective views, including a view with bodywork
  hidden. Examine wheel crowns, spokes, axle ends, frame joints and floor edges.
  A successful render, connected graph or plausible perspective is not a pass.

## Run the downhill gate

Use the saved Blender build as the source of the physics export. Preserve its
transforms and dimensions; include curve thickness and modifiers. Separate wheel
bodies from the chassis and give each wheel a free axle hinge. Release brakes;
disable motors. Start from rest with no push or prescribed velocity.

Prefer primitives or convex decomposition for collision geometry. Do not use a
single convex hull for an entire hollow frame. Check colliders against the
rendered geometry; label approximations and unrepresented mesh edits. For wheel
clearance, include the tire and the full space swept by rotating spokes.

Enable wheel/chassis collision checking explicitly: MuJoCo normally filters
parent/child body collisions. Allow only documented mating contacts such as
axle/hub interfaces; never suppress all wheel/frame collisions to obtain a pass.

Default baseline: a 5-degree slope, four seconds, and a 1 ms timestep. Require:

| Check | Default acceptance |
| --- | --- |
| Downhill progress | More than 1 m from rest |
| Wheel rotation | Every intended rolling wheel turns in the travel direction |
| Rolling rather than sliding | Each wheel's angle × radius agrees with chassis travel within 10% |
| Ground support | Every road wheel contacts the slope; no chassis dragging |
| Unwanted contacts | None between wheels and frame, floor, body or other fixed parts |
| Stability | Chassis tilt relative to slope stays below 10 degrees; finite state and no solver warnings |

For different wheel layouts or mechanisms, adapt the adapter and define any
changed conditions/thresholds before running. Explain the physical reason. Do not
relax a failed threshold or remove a collider merely to turn the report green.

## Verify that a pass is meaningful

- Run a negative control with a deliberate chassis obstruction inside a wheel's
  rotating envelope. It must fail on unwanted contact. Keep it separate from the
  deliverable and restore the valid model afterward.
- Keep geometry checks for required connections and frame/floor clearance.
  MuJoCo assumes parts assigned to one rigid body are attached and does not test
  collisions within that body. A disconnected part can therefore roll with the
  car and escape the physics test.
- Inspect the generated motion and diagnostic views. Highlight and inspect any
  failed part pair. Check the actual collision geometry, not just construction
  metadata or an attractive visual mesh.
- When brakes are present, first pass with brakes released, then test braking
  from motion, holding on the slope and release. Measure chassis motion as well
  as wheel speed. Use passive resisting torque/contact, not forced zero velocity.
  An unbraked control must not pass the stopping test.
- When introducing/changing brake or contact solver behavior, repeat the relevant
  test at half the timestep and compare stopping/rolling results. Document any
  material sensitivity rather than presenting an unstable result as validated.

## Preserve evidence and close the gate

Save the model/config identity (including a source `.blend` SHA-256), software
versions, simulation conditions, numeric report, trajectory and motion preview
with the tested build. Record the visual inspection and collision approximations
in a short validation note. Ensure the model was not changed after these checks.
Previous reports and videos do not validate a new build.

If a check fails, inspect the reported parts, fix the cause, rebuild and rerun the
affected checks plus the downhill gate. Do not continue decorative refinement of
a new chassis until it passes. If tools or dependencies prevent validation,
report the build as **unverified**, with the concrete blocker; do not infer a pass.
Do not claim manufacturing accuracy, steering clearance, or structural strength
from a straight-ahead rigid-body downhill test.

For this repository's commands, current adapter limits and brake thresholds,
read [references/current-repo.md](references/current-repo.md).
