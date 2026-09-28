# Procedural cars

A workshop for building car geometry from parameters and mechanical constraints.

- `.venv/`: local Python environment (created, intentionally untracked).
- `car_tech/`: sourced aerodynamic fundamentals and geometry design rules.
- `experiments/`: reproducible approaches, starting with an early motor car.
- `scripts/build.py`: runs the installed Blender from ordinary Python.

## First model

```bash
source .venv/bin/activate
python scripts/build.py --render
blender experiments/001_patent_motorwagen/output/patent_motorwagen.blend
```

The build produces an editable `.blend`, a vehicle-only `.glb`, a geometry report,
and, with `--render`, a studio PNG. No pip dependencies are required: Blender
supplies `bpy` and its own Python; the virtual environment runs the launcher.
Blender 4.2+ is targeted; this workspace was verified with 5.2.2 LTS.
If Blender is elsewhere, pass `--blender /path/to/blender`.
Recreate the environment with `python3 -m venv .venv`.

The first experiment is **inspired by the 1886 Benz Patent-Motorwagen**, an early
purpose-built petrol automobile. “First car” depends on definition; earlier
steam vehicles existed. This is an approximate geometric study, not a measured
museum reconstruction or working mechanical assembly.

Read [the technical index](car_tech/README.md), then
[the experiment](experiments/001_patent_motorwagen/README.md).

![First procedural car](experiments/001_patent_motorwagen/output/preview.png)

Generated outputs are local and ignored by Git; rebuild them using the command above.

## Downhill physics test

[Experiment 002](experiments/002_downhill/README.md) exports the saved Blender car
to MuJoCo and checks rolling, unwanted contacts and stability on a slope.

```bash
.venv/bin/python experiments/002_downhill/run.py
```

MuJoCo is installed in this workspace; installation instructions are in the experiment.

## Braking test

[Experiment 003](experiments/003_brakes/README.md) adds rear drum brakes and compares
coasting, stopping/holding, and releasing the brakes on the same slope.

```bash
.venv/bin/python experiments/003_brakes/run.py --viewer
```

## Steering test

[Experiment 004](experiments/004_steering/README.md) sweeps the tiller through its
±25° travel for clearance, then steers left and right down the slope and brakes mid-turn.

```bash
.venv/bin/python experiments/004_steering/run.py --viewer
```

## Engine drive

[Experiment 005](experiments/005_engine_drive/README.md) connects the engine to the
rear wheels through a belt, countershaft differential and chains, then tests a
flat launch, hill starts, powered turns and braking against the engine.

```bash
.venv/bin/python experiments/005_engine_drive/run.py --viewer
```

## Second car: Benz Velo

[Experiment 006](experiments/006_benz_velo/README.md) builds the 1894 Benz Velo
gate-first. It has a four-wheeled chassis with a real double-pivot steering
linkage in physics, a two-speed belt drive, and band and drum brakes. Every
declared join is checked to touch, and every other part pair to stay apart,
across the full steering range.

```bash
.venv/bin/python experiments/006_benz_velo/run.py --stage full --viewer
```

![Benz Velo study](experiments/006_benz_velo/output/full/preview.png)

## Required build validation

[Car Downhill Check](skills/car-downhill-check/SKILL.md) defines the build gate:
a new chassis must roll downhill under its own gravity-driven motion before
detailed modeling. [AGENTS.md](AGENTS.md) requires the skill for future car builds
and geometry/mechanics changes. It also covers spatial checks, negative controls,
braking tests, and recording evidence for the exact model tested.
