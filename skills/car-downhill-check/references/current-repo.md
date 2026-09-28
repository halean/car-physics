# Current repository workflow

Run from the `3d-cars` repository root. `.venv` runs the simulation; Blender uses
its own Python for generation and geometry checks. Install missing simulation
dependencies with `.venv/bin/python -m pip install -r experiments/002_downhill/requirements.txt`.

## Current Motorwagen build

```bash
.venv/bin/python scripts/build.py --render
blender --background experiments/001_patent_motorwagen/output/patent_motorwagen.blend --python-exit-code 1 --python experiments/001_patent_motorwagen/check_frame.py
.venv/bin/python experiments/002_downhill/run.py
.venv/bin/python experiments/002_downhill/record.py
```

`check_frame.py` checks structural-member connectivity (including the engine bed
and drive path), wheel envelopes, rail/floor clearance and belt/chain seating.
It also checks that detached-fork, low-fork-crown and displaced belt, chain,
hanger and reservoir defects are rejected. It does not check every possible part pair. Inspect brake and body
attachments separately when they change.

`002_downhill/run.py` re-exports the saved Blender model and writes
`experiments/002_downhill/output/report.json`, `trajectory.csv`, `geometry.json`,
`downhill.xml` and visual OBJ meshes. It exits nonzero on failure and includes a
wheel-obstruction negative control. Read the report, not just the exit status.
`record.py` writes `downhill.mp4`; recording requires ffmpeg and EGL.
Use `run.py --viewer` for a live desktop preview when recording is unavailable.
Neither missing video dependencies nor a working GUI establishes a physics pass.

## Brakes on the current build

After the unbraked downhill gate:

```bash
.venv/bin/python experiments/003_brakes/run.py
.venv/bin/python experiments/003_brakes/record.py
```

Read `experiments/003_brakes/output/report.json` and inspect
`brake_comparison.mp4`. The test compares coasting, braking/holding, and release.
It applies a 0.25-second ramp at 2 seconds, up to 75 N m per rear wheel; the
release scenario releases at 5 seconds. Acceptance includes:

- Chassis speed below 0.02 m/s after braking.
- At least one second of holding after a 0.25-second settling interval, with
  speed below 0.02 m/s and less than 0.01 m total drift.
- Less than half the coast distance after six seconds.
- Speed above 1 m/s after release; the unbraked control must fail to stop.
- At half timestep, stopping distance changes by less than 0.03 m, and holding
  drift remains below 0.01 m.

The brake is equivalent joint dry friction. The decorative lever/linkage does
not actuate simulated shoes or cable tension. Do not describe it as doing so.

## Steering on the current build

After the downhill and brake tests:

```bash
.venv/bin/python experiments/004_steering/run.py
.venv/bin/python experiments/004_steering/record.py
blender --background experiments/001_patent_motorwagen/output/patent_motorwagen.blend --python-exit-code 1 --python experiments/004_steering/render_views.py -- experiments/004_steering/output
```

`run.py` sweeps the steering through its full limit in 0.5-degree steps, requiring
a conservative 5 mm gap between steering/front colliders and everything else, and
a planted sweep blocker must be detected. It then runs 6-second left/right descents
with a 15-degree tiller command and rear braking from 3 seconds. Each must turn the
commanded way, match rear-axle kinematic yaw within max(1 degree, 5%), roll without
sliding, avoid forbidden contacts and stop/hold. The left run is repeated at half
timestep. Inspect `steering_comparison.mp4` and `output/views/` (steering orange),
then read `validation.md`. There is no caster/trail or slip-angle model.

## Engine drive on the current build

After the downhill, brake and steering tests:

```bash
.venv/bin/python experiments/005_engine_drive/run.py
.venv/bin/python experiments/005_engine_drive/record.py
blender --background experiments/001_patent_motorwagen/output/patent_motorwagen.blend --python-exit-code 1 --python experiments/005_engine_drive/render_views.py -- experiments/005_engine_drive/output
```

The 002–004 models have no drive actuator (belt disengaged), so the downhill gate
stays unpowered. `005/run.py` checks rotating drive parts as swept solids against
every other collider, including same-body parts except listed mates, with a
planted belt-path blocker. It runs a flat launch, 3° and 7° hill starts,
left/right powered turns with a locked-differential control, braking against the
engine and a half-timestep launch. Speed and distance are compared with a 1-D
reference model. Limits are in `LIMITS` at the top of the runner. The drivetrain
is massless; there is no rolling resistance or drag.

## Benz Velo (experiment 006)

The Velo has its own body-tagged generator and MuJoCo adapter. Do not run the
Motorwagen commands on it.

```bash
blender --background --factory-startup --python-exit-code 1 --python experiments/006_benz_velo/generate.py -- --config experiments/006_benz_velo/parameters.json --output experiments/006_benz_velo/output/chassis --stage chassis
.venv/bin/python experiments/006_benz_velo/run.py --stage chassis
blender --background --factory-startup --python-exit-code 1 --python experiments/006_benz_velo/generate.py -- --config experiments/006_benz_velo/parameters.json --output experiments/006_benz_velo/output/full --stage full --render
.venv/bin/python experiments/006_benz_velo/run.py --stage full
.venv/bin/python experiments/006_benz_velo/record.py
```

- **Part tags.** Each part carries `body`, `joins` and `mates` properties.
- **Checks in `run.py`.** Every declared join must touch at every column angle.
  Every other collider pair, same-body pairs included, must stay 1 mm apart, or
  5 mm when the parts move relative to each other. Rotating drive parts count as
  their swept solids. Three planted defects must be caught.
- **Gate.** The unpowered gate follows those checks, with its blocked-wheel
  control.
- **Full stage.** It also runs the scenarios in `dynamics.py`, with thresholds
  in `LIMITS`.
- **Adapter.** Linkage loops are closed with connect constraints. Tyres are
  compliant ellipsoid crowns, and the road never touches the wheel clearance
  envelopes.
- **Review.** Read `output/full/validation.md` and the README's open finding.

## Panhard et Levassor (experiment 007)

Built on experiment 006's tooling. Replace `006_benz_velo` with
`007_panhard_1891` and `benz_velo` with `panhard_1891` in the commands above.
The runner imports 006's adapter and supplies its own `vehicle.json`, controls,
scenarios and planted defects.

The shared adapter models tyres as a stated compliance: k 2300 s⁻² and b 48 s⁻¹
per unit mass (about 6 Hz, 1–1.5 mm sag), building over 10 mm (solimp
0.5–0.95), with MuJoCo's no-slip friction pass. A rigid contact made tyre loads
jitter at ~95 Hz in turns; a regression guard now checks powered-turn tyre
loads.

## Renault Type A (experiment 008)

As for 007: replace `007_panhard_1891` with `008_renault_1898` and
`panhard_1891` with `renault_1898`. Scenarios include the combined
clutch/brake pedal, which must cut drive to zero while braking. The gated
powered turns run in first gear; second gear exceeds the tyres' grip and is
reported as a finding.

## New cars and adapter limitations

These commands currently target experiment 001. The exporter expects its named
vehicle collection, wheel names, parameters and source report. The simulation
assumes one rigid chassis, three axle hinges and a front fork/tiller body on a
limited yaw hinge (held straight ahead in 002/003).
The braking test assumes two rear brakes. Adapt these for the new car rather
than running them unchanged and attributing their pass to a different model.

Collision proxies use rod/curve construction metadata, box dimensions and
conservative wheel envelopes; arbitrary mesh edits may change the visuals
without updating the colliders. Verify or replace the adapter when these inputs
no longer represent the model. For steering/suspension changes, test the added
motion range separately; a straight-ahead descent cannot certify it.

## Evidence for the final build

Compute the fingerprint after saving and before validating; confirm it remains
unchanged afterward:

```bash
sha256sum experiments/001_patent_motorwagen/output/patent_motorwagen.blend
```

Store it with the configuration, reports and a short `validation.md` in the
build's output directory. Record source/output paths, commands and results,
inspected views/video, numeric travel/rotation/stop metrics, assumptions and
remaining untested behavior. The existing runners do not create that complete
validation note automatically. Render missing orthographic or chassis-only views
from the current saved scene; old diagnostic images are not fresh evidence.
