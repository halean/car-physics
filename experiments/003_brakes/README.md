# 003 — Rear brakes: stop, hold, release

An experimental pair of rear drum brakes added to the procedural car. This is a
functional study, not a reconstruction of the historical Motorwagen brake.

The Blender model now has brass-colored drums connected to the rear wheel hubs by sleeves,
stationary backing plates and frame anchors, a cross shaft, pull rod and a red
hand lever. The inboard drums clear both the wheel spokes and the chassis. The
exporter attaches the drums' visuals and cylinder colliders to their rotating
wheel bodies; the backing plates and linkage remain on the chassis.

## Run

Use the environment and dependencies from experiment 002:

```bash
.venv/bin/python scripts/build.py --render
.venv/bin/python experiments/003_brakes/run.py
# Watch the braking run:
.venv/bin/python experiments/003_brakes/run.py --viewer
# Record a side-by-side comparison (ffmpeg and EGL required):
.venv/bin/python experiments/003_brakes/record.py
```

The test exports the saved Blender model before running. Outputs include
`report.json`, separate `coast.csv`, `brake.csv`, `release.csv` trajectories,
`downhill.xml`, visual meshes and `brake_comparison.mp4` when recorded. The XML
stores the released mechanism; the Python controller supplies the brake schedule.

## Physics and controls

`control.py` ramps the maximum dry-friction torque from 0 to **75 N m per rear
wheel**, starting at 2 seconds and reaching full command after 0.25 seconds.
MuJoCo's joint `frictionloss` constraint supplies resisting torque and static
holding torque. No motor or direct position/velocity reset is used. The release
test removes braking torque at 5 seconds.

The lever and linkage are visual representations. Their movement, cable tension,
internal brake shoes, contact pressure, heat and wear are not simulated. Joint
friction is the equivalent brake model. The two drums are not clamped by fake
geometry intersections. Total mass remains the assumed 135 kg, including brake
hardware, so coast and brake runs use identical mass and inertia.

Dry-friction solver settings are `solreffriction=".004 1"` and
`solimpfriction=".999 .999 .001 .5 2"`, with a 1 ms timestep. These reduce numerical
creep under static load; they do not make the brake infinitely strong. Residual
creep is measured and reported rather than forcibly zeroed.

## Required outcomes

- Standard downhill collision, wheel contact, rolling and stability checks pass.
- Braking reduces chassis speed below 0.02 m/s after it has begun moving.
- After a 0.25-second settling interval, it holds for at least a second with
  speed below 0.02 m/s and total drift below 0.01 m.
- Braked travel is less than half the unbraked travel at six seconds.
- The release run stays stopped until release, then exceeds 1 m/s.
- With no brake, the car fails the stopping criterion (negative control).
- Halving the timestep to 0.5 ms preserves the pass and changes stopping distance
  by less than 0.03 m; holding drift must still stay below 0.01 m.

On the default 5-degree slope, the car reaches 1.54 m/s before braking. It stops
in approximately 1.44 seconds over 1.28 m after application, with about 0.53 mm of
subsequent holding drift. These are simulation results under the stated mass,
friction and collision assumptions, not measured braking performance.

[MuJoCo dry-friction formulation](https://mujoco.readthedocs.io/en/latest/computation/)
and [joint solver parameters](https://mujoco.readthedocs.io/en/latest/XMLreference.html).
