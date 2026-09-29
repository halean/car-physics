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
0.5–0.95), no contact margin, and an elliptic friction cone with impratio 50
(the no-slip pass it replaced could zero a lightly loaded tyre's normal force). A rigid contact made tyre loads
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

## Mercedes 35 HP (experiment 009)

As for 007: replace `007_panhard_1891` with `009_mercedes_35hp_1901` and
`panhard_1891` with `mercedes_35hp`. This is the first sprung car. The
geometry declares `suspension` (joints, travel and `check_poses`), and the build
checks run at every pose combined with the steering sweep. Leaf springs are
`rest_joins`, which are checked at rest only. The runner calibrates spring
preloads to ride height, then gates on 20–150 mm static deflection, suspension
travel, and bump and roll steer. `record.py` reads the calibrated preloads from
`report.json`, so run it after `run.py --stage full`.

## Ford Model T (experiment 010)

As for 009: replace `009_mercedes_35hp_1901` with `010_ford_model_t_1909` and
`mercedes_35hp` with `model_t`. Both axles swing about a ball and roll about the
line from the ball to the axle centre. `suspension` then declares
`front_swing_joint`, `front_pivot` and `front_roll_axis`, which
`linkage.suspended_pose` supports alongside the heave-type front axle.

Watch for this: `vehicle.part_inertia` places each small body's mass from its
colliders. It had a missing `/mass` until 010, which scaled centre-of-mass
offsets by the body's mass. If a sprung car calibrates to implausible spring
preloads, check the compiled `body_ipos` of the axle bodies first.

## Lancia Lambda (experiment 011)

As for 010: replace `010_ford_model_t_1909` with `011_lancia_lambda_1922` and
`model_t` with `lancia_lambda`. Independent front suspension: each wheel
carrier is a `pillar_<side>` body on a vertical slide joint, and the knuckle is
its child. `suspension` declares `independent_front`, `front_left_joint`,
`front_right_joint`, and `check_poses` with `front_left` / `front_right`
travel (up positive), including one-sided poses. The tie rod is a ball joint.
A propeller shaft is two bodies (`shafts` in `suspension`: ball at the front
universal joint, slide for the spline) closed to the axle by a connect
constraint. The pedal brakes all four wheels (`braked` on every wheel; the
controller sets front and rear joint friction); the scenarios add four-wheel
braking in a turn and a weight-transfer measurement from the contact forces.

## Citroën Traction Avant (experiment 012)

As for 011: replace the experiment with `012_citroen_traction_avant_1934` and
the scene with `traction_avant`. Front-wheel drive: `driven` is set on the
front wheels, whose parents are the knuckles. Wishbones: `suspension.wishbones`
gives the arm length and, per side, the joints that turn with the arm angle
(lower arm +1, upper arm +1, upright −1, half-shaft +1). The springs are on the
`lower_arm_<side>_swing` hinges, and the check poses use `front_left` and
`front_right` travel. Steering `type: centre_drop_arm`: a drop arm on a
lengthwise sector shaft, with `track_rod_<side>` ball-jointed bodies closed
to each knuckle. `excluded_pairs` in the report lists body pairs whose contact
MuJoCo must ignore (the half-shaft and the knuckle its outer joint sits in).
Box parts' colliders have no `:n` suffix, so planted defects that match geoms
by name must use rod parts.

