# Procedural car builds

For every new car build or change to car geometry/mechanics, read and follow
[skills/car-downhill-check/SKILL.md](skills/car-downhill-check/SKILL.md).
This is a required repository workflow, including when automatic skill selection
does not load it. Documentation-only changes do not require a simulation rerun.

For curved bodywork (wings, bonnet, roof, any coachwork beyond box panels), read and
follow [skills/procedural-bodywork/SKILL.md](skills/procedural-bodywork/SKILL.md):
design it from parametric curves as closed shells with convex collision patches.

A new car must first pass an unpowered MuJoCo downhill rolling test with brakes
released, using its own generated geometry, before detailed bodywork or trim.
After changes, obtain a fresh passing downhill test and relevant spatial checks
before claiming the build is complete or fixed. Cars with brakes must also pass
stopping, holding and release checks.

Preserve reports and inspect motion/diagnostic views. Keep checks for disconnected
parts and same-body intersections: a rigid-body simulation does not verify those.
Do not substitute a previous model's results, disable collisions to obtain a pass,
or claim validation when the required tools could not run. Draft artifacts may
be saved while working toward a passing build.
