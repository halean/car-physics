# Car technology for procedural geometry

Start with [aerodynamic fundamentals](aerodynamics.md), then the
[geometry contract](geometry.md) and [historical reference](patent_motorwagen.md).
[Sources](sources.md) record provenance. Values marked **assumption** are modeling
choices, not historical measurements or simulated performance.

Research order: packaging → wheel and axle placement → structural connections →
body envelope → aerodynamic refinements → details. Preserve useful parameters
before committing to topology. Visual plausibility alone cannot establish drag,
strength, stability, manufacturability, or roadworthiness.

[Braking](braking.md) covers rotating/fixed brake parts, stopping forces, holding,
and the measurements used by the MuJoCo brake experiment.

[Lessons learned](lessons_learned.md) collects what building cars 001–010
taught about vehicle engineering, MuJoCo/Blender modelling pitfalls and the
validation process. Read it before designing a new car.

[Driving controls](driving_controls.md) covers how simulators display and take throttle and
brake input, and how the period cars' own controls map onto the shared web controls.
