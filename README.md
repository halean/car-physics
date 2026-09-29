# Procedural cars

**Try it in your browser: [halean.github.io/car-physics](https://halean.github.io/car-physics/).** Lesson 1, *Rolling downhill*, puts an 1891 Panhard on a slope with its forces drawn, and the garage lets you drive every car. For students aged 14 to 18. The physics runs locally on MuJoCo.

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

## Third car: Panhard et Levassor 1891

[Experiment 007](experiments/007_panhard_1891/README.md) builds the "Système
Panhard": a front engine with clutch, three-speed gearbox, differential and
chains, a tiller and rear rim-block brakes. It reuses the Velo's validated
adapter, checks and double-pivot front axle.

```bash
.venv/bin/python experiments/007_panhard_1891/run.py --stage full
```

## Fourth car: Renault Type A 1898

[Experiment 008](experiments/008_renault_1898/README.md) replaces chains with a
propeller shaft and universal joints to a live axle, with a direct-drive third
gear and a single pedal that declutches and brakes.

```bash
.venv/bin/python experiments/008_renault_1898/run.py --stage full
```

## Fifth car: Mercedes 35 HP 1901

[Experiment 009](experiments/009_mercedes_35hp_1901/README.md) is the first
sprung car: leaf springs on both axles, a raked steering wheel through a
steering box, a four-speed gate change and chain drive. Build checks run at
bump, rebound and roll poses as well as across the steering range.

```bash
.venv/bin/python experiments/009_mercedes_35hp_1901/run.py --stage full
```

## Sixth car: Ford Model T 1909

[Experiment 010](experiments/010_ford_model_t_1909/README.md) hangs each beam
axle from a transverse leaf spring and locates it with a ball: a wishbone at
the front, the torque tube at the rear. It drives through a pedal-worked
two-speed planetary transmission. Building it exposed a shared inertia bug;
every earlier car was revalidated after the fix.

```bash
.venv/bin/python experiments/010_ford_model_t_1909/run.py --stage full
```

## Seventh car: Lancia Lambda 1922

[Experiment 011](experiments/011_lancia_lambda_1922/README.md) is the first
car with independent front suspension (sliding pillars with coil springs and
hydraulic dampers), a load-bearing hull, and brakes on all four wheels. The
build checks pose each front wheel separately, and two new gates test
four-wheel braking in a turn and the weight transfer under braking.

```bash
.venv/bin/python experiments/011_lancia_lambda_1922/run.py --stage full
```

## Eighth car: Citroën Traction Avant 1934

[Experiment 012](experiments/012_citroen_traction_avant_1934/README.md) is the
first car whose driven wheels are also its steered wheels: front-wheel drive
through double-Cardan joints on the kingpins, wishbones sprung by torsion
bars, and a flat-floored welded body. A new gate shows the front-drive
effect on a hill: climbing takes load off the driving wheels.

```bash
.venv/bin/python experiments/012_citroen_traction_avant_1934/run.py --stage full
```

## Required build validation

[Car Downhill Check](skills/car-downhill-check/SKILL.md) defines the build gate:
a new chassis must roll downhill under its own gravity-driven motion before
detailed modeling. [AGENTS.md](AGENTS.md) requires the skill for future car builds
and geometry/mechanics changes. It also covers spatial checks, negative controls,
braking tests, and recording evidence for the exact model tested.

## Licence

- **Code:** MIT, see [LICENSE](LICENSE).
- **Written material** (docs, lesson texts): CC BY 4.0, see
  [LICENSE-docs.md](LICENSE-docs.md). Teachers are welcome to reuse and adapt
  it with credit.
- **Third-party components** (MuJoCo, three.js, fonts):
  [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

