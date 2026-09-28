# 001 — Patent-Motorwagen study

A deterministic primitive-and-curve assembly with three spoked wheels, tubular
chassis, timber platform, upholstered bench, tiller steering and exposed engine.
See [historical assumptions](../../car_tech/patent_motorwagen.md).

From the repository root:

```bash
.venv/bin/python scripts/build.py --render
.venv/bin/python scripts/build.py --config experiments/001_patent_motorwagen/parameters.json --output /tmp/car-variant
```

Edit `parameters.json` or copy it to make variants. Dimensions are meters. Wheel
radius means outer rolling radius; tire radius is the round tire cross section.
The generator validates inputs before clearing the scene. Run it in a fresh
Blender process; it deliberately replaces the current scene.

Output: `patent_motorwagen.blend`, `patent_motorwagen.glb`, `report.json`, and
optional `preview.png`. The blend retains editable curves, named parts,
materials, lights and camera; GLB contains only vehicle geometry. Rebuilding
replaces files in the specified output directory.

Checks enforce three tire objects, their ground contact, mirrored rear centers,
finite bounds, and a front/rear wheel clearance constraint. They do not establish
mechanical fidelity, mesh watertightness or aerodynamic performance.

## Build validation

Built with Blender 5.2.2 LTS and Python 3.12.12 on 2026-09-28. The default
configuration produces 197 vehicle objects, approximately 2.59 × 1.32 × 1.32 m.
Generation and a Cycles render completed successfully; the preview was visually
inspected. Tire count, road contact, rear symmetry and finite bounds passed. All 34 structural members form one connected assembly;
a detached-fork regression also passed after reopening the saved scene.
The GLB was parsed independently: all 197 vehicle nodes were present, including
converted frame curves, with no studio objects. Python syntax compilation passed.

![First generated model](output/preview.png)

Frame rails and forks share a steering-head attachment point. Crossmembers follow
the rail taper; axle, spring, engine and floor mounts bridge the supported parts.
Members remain separate editable objects with contacting geometry, not a single
watertight mesh. Run the connectivity regression from the repository root:

```bash
blender --background experiments/001_patent_motorwagen/output/patent_motorwagen.blend --python-exit-code 1 --python experiments/001_patent_motorwagen/check_frame.py
```

Wheel clearance is checked against a conservative solid rotating-disk envelope
(tire and spoke thickness), excluding the intended axle/hub connections. The fork
crown and rail junction sit above the tire; rail bends begin beyond the floor.
The check script also verifies rail/floor clearance and rejects the previous
low-crown geometry. These checks cover the straight-ahead pose, not suspension
articulation; steering travel clearance is swept in
[experiment 004](../004_steering/README.md). Side and top diagnostic renders are saved
in `output/side.png` and `output/top.png` for this revision.

The current model includes experimental rear drums and a hand-brake linkage.
See [experiment 003](../003_brakes/README.md) for the simulated brake behavior.

It also has a belt, countershaft, differential and chain drive from the flywheel
shaft to sprockets on the rear drums (see [experiment 005](../005_engine_drive/README.md)).
The frame check covers the engine bed, reservoir and drive path. It also checks
that the belt and chains sit on their pulleys' pitch paths and that wheel
sprockets are mounted on their hub sleeves. Mesh cylinders are sampled as flat
cylinders in the wheel-clearance test.
