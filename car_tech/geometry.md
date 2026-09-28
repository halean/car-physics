# Geometry contract

Use meters; +X forward, +Y left, +Z up, road at Z=0. Put the rear axle at X=0
and the front axle at X=wheelbase. Wheel radius locates each wheel center above
the road; track is between left/right wheel centers, not outside tire faces.

Derive hard points from independent parameters: wheelbase, track, rolling radii,
tire section, platform height, seat width, and tube radius. Derive paired parts
by Y reflection. Keep the wheel count/layout explicit: a three-wheeler cannot
inherit four-wheel steering or suspension assumptions.

| Assembly | Procedural representation | Check |
| --- | --- | --- |
| Frame rails / steering rods | Curves with round bevel | Endpoint continuity and tube clearance |
| Tires / rims | Tori around axle | Rolling radius and ground contact |
| Spokes | Repeated segments between hub and rim | Count, plane, radial endpoints |
| Seat / floor | Beveled boxes and repeated planks | Fit between rear wheels |
| Engine / flywheel | Cylinders, torus, radial arms | Packaging behind/below seat |
| Later enclosed body | Lofted cross sections or subdivision cage | Tangent continuity and wheel clearance |

Keep model parts in semantic collections and staging separate. Export only the
vehicle. Preserve input JSON and Blender version in every report. Use orthographic
front/side/top views for dimensional checks; perspective is for presentation.
Render geometry may contain intersecting components. It is not automatically a
watertight 3D-printing mesh, simulation boundary, or manufacturable assembly.
