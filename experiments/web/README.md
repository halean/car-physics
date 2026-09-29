# Brass Era Garage: the cars in the browser

A small site for students aged 14–18: how early cars were developed, what they looked
like, how they drove, and the physics underneath. Pages:

- `index.html`: the start page.
- `downhill.html` (**1 · Rolling downhill**): the validated 1891 Panhard on a slope.
  It shows the weight, its components, the normal force, the road friction and the
  net force as arrows from the simulation, and checks ΣF = ma live. The student sets
  the slope θ and the rim-block friction coefficient μ (F = μP per block), then
  works through three predict-then-check challenges.
- `garage.html`: every car with shared controls (below). `local.html` is an alias.
- Both pages have occasional **cows** (`cows.js`), seeded so a restart brings the
  same herd back. They are scenery, not physics: a cow on the road ambles to the
  verge when a car comes near, and trots if the car is almost on it. On the
  downhill page, "stop for the cow" is challenge 4 (a stopping distance).
- On the downhill page a driver holds the car on its line. The run pauses at
  30 m/s: with no drag in the model, a steep hill would otherwise build
  unlimited speed (the driver loses the line only above about 97 m/s).

`downhill_test.mjs` checks every number the lesson states, using the page's own
setup:
- the car's validated 5° runs (6.233 m rolled in 4 s; a 4.53 m brake stop);
- ΣF = ma to 0.00 N at every sample;
- each challenge's answer.

The slope tilts gravity on the flat road, which is the same physics as tilting
the road. One lesson learned: MuJoCo's `mj_subtreeVel` uses the velocities of
the last forward pass, so pair each step's contact forces with the velocity
change over that same step.

```bash
.venv/bin/python experiments/web/build.py --serve      # then open localhost:8321/
~/.nvm/versions/node/v24.21.0/bin/node experiments/web/parity_test.mjs
~/.nvm/versions/node/v24.21.0/bin/node experiments/web/downhill_test.mjs
```

This is a browser simulator for every validated car:
- Panhard 1891, Benz Velo 1894, Renault 1898, Mercedes 1901, Model T 1909;
- the Model T with aftermarket friction shocks.

All cars share one set of controls.

- **Physics:** MuJoCo's official WebAssembly build, pinned to the same version
  as the Python validation. Each car runs its validated full model (its last
  `run.py` output, `velo.xml`), with only the mesh paths shortened. The bump
  course adds the bars of `010/bumpy_road.py` as road geoms.
- **`cars.py`:** how each car's real controls map onto the shared ones. That is
  the gear names; which physical brake the pedal and the lever work, with the
  same devices and capacities as the car's `control.py`; the steering type and
  box ratio; suspension joints; shock joints.
- **`sim.js`:** the shared core, a port of the cars' `control.py`: governed
  engine, clutch or belt ramp, joint and tendon brakes, the Renault's pedal
  declutching before it brakes, and the column servo. It also covers live
  springs, dampers and shock friction.
- **`input.js`:** one action layer for keyboard (ramped), gamepad (analog
  triggers, deadzone) and on-screen pedals (analog by pointer height). The
  reasons are in [car_tech/driving_controls.md](../../car_tech/driving_controls.md).
- **`app.js`, `page.html`:** the page.

**Parity:** for each car, `build.py` runs the car's own Python `control.py` on
the same model: first gear at 0.5 s, pedal brake from 4 s, 8 s in all.
`parity_test.mjs` runs the same inputs through `sim.js` on the WebAssembly
build. Every car matches at every half-second sample; the limit is 1 mm and
1 mm/s, set before the first run.

| Car | Distance at 8 s | Difference |
| --- | --- | --- |
| Panhard 1891 | 14.387 m | 0 |
| Benz Velo 1894 | 9.773 m | 0 |
| Renault 1898 | 9.589 m | 0 |
| Mercedes 1901 | 37.950 m | 0 |
| Model T 1909 | 42.098 m | 0 |
| Model T + shocks | 42.095 m | 0 |

**What is not validated:**
- The hand throttle scales engine torque below the governor; it is a common
  control, not every car's mechanism.
- Changed spring, damper and shock settings are for exploring.
- The pure-pursuit "driver holds the line" assist was validated only on the
  Mercedes and the Model T.

**Hosting:** the site is static files, about 28 MB, 10 MB of it
`mujoco.wasm`. Any static host that serves `.wasm` works. It is not published
as an artifact here, because that would mean publishing an engine binary that
has not been reviewed.
