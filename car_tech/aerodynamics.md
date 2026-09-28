# Aerodynamics: principles that change geometry

## Forces and scale

Air exerts surface pressure and viscous shear. Their surface integrals give the
net aerodynamic force: drag opposes relative motion, lift is transverse to it.
A smooth material shader has no effect on physical surface roughness. [S2]

Use SI units and air-relative velocity, including wind:

- Dynamic pressure: `q = 0.5 * rho * V²` (Pa).
- Drag: `D = q * Cd * A` (N); road cars conventionally use frontal projected area.
- Aerodynamic power at steady speed in still air: `P = D * V` (W).
- Lift: `L = q * Cl * A`; declare positive upward and the reference area.

`Cd` is not a universal property of a mesh. It depends on geometry, flow and the
chosen reference area. Compare `Cd*A` with identical test conditions. At fixed
coefficients, doubling speed quadruples drag and multiplies aero power by eight.
Example assumptions `rho=1.225 kg/m³`, `V=20 m/s`, `Cd=0.30`, `A=2 m²` give
`D=147 N`, `P=2940 W`. These are illustrative, not Motorwagen predictions. [S1]

## Boundary layer and separation

The no-slip condition slows air at the wall. The boundary layer can be laminar
or turbulent; adverse pressure gradients can cause separation and a wake.
Reynolds number `Re = rho * V * length / mu` compares inertial and viscous effects.
A scaled model at the same speed need not reproduce full-scale flow. [S3]

Modeling implications below are engineering hypotheses to test, not guaranteed
drag reductions. They translate the cited fundamentals into useful parameters:

| Geometry control | Why retain it | How to compare |
| --- | --- | --- |
| Width / roof height | Changes projected frontal area | Orthographic silhouette union |
| Nose radius / hood slope | Changes pressure distribution | Same packaging and speed |
| Roof-to-rear curvature / taper | Affects pressure recovery and separation | Sweep independently, measure wake and drag |
| Rear cutoff area | Changes base and wake geometry | Hold length or volume explicitly |
| Wheel exposure / arches | Rotating wheels and gaps disturb flow | Include wheels and ground in test |
| Ride height / underfloor / diffuser | Alters flow between car and road | Moving ground and rotating wheels |
| Cooling inlets and outlets | Internal flow interacts with external flow | Model a real flow path |
| Spoiler / wing incidence | Changes forces and pitching moments | Record lift as well as drag |

Shape comparisons require matched reference area and flow conditions. A rounded
front alone does not establish a low-drag vehicle. [S4]

## Model fidelity and experiments

The first open carriage is a geometry baseline, not an aerodynamic optimization.
Exposed tubes, wheels, bench and engine should remain distinct. Do not borrow a
modern enclosed car's coefficient. At low speed, compare aerodynamic resistance
with rolling resistance before prioritizing body streamlining.

For a later CFD study: make a separate sealed analysis surface, resolve intended
gaps, remove decorative microgeometry, check normals and intersections, define
fluid domain and boundary conditions, and demonstrate mesh/domain convergence.
Declare speed, density, viscosity, yaw, ground treatment, wheel rotation, reference
area and coefficient signs. A render is not CFD and this repository has no solver.

References: [source index](sources.md), S1–S4.
