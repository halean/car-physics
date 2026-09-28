# Experiments

| Experiment | Approach | State |
| --- | --- | --- |
| [001](001_patent_motorwagen/README.md) | Parametric primitives and swept tubes; early three-wheeler | Implemented |
| [002](002_downhill/README.md) | MuJoCo downhill rolling and contact test | Implemented |
| [003](003_brakes/README.md) | Rear brake: stop, hold and release in MuJoCo | Implemented |
| [004](004_steering/README.md) | Tiller steering: travel clearance, steered descent and braking in a turn | Implemented |
| [005](005_engine_drive/README.md) | Engine drive: belt, countershaft, differential and chains; powered launch, climb, turns | Implemented |
| [006](006_benz_velo/README.md) | Benz Velo (1894): second car, gate-first; double-pivot steering linkage, two-speed drive, two brakes | Implemented |
| [007](007_panhard_1891/README.md) | Panhard et Levassor (1891): front engine, clutch, three-speed gearbox, chains, rim-block brakes; built on 006 tooling | Implemented (known limitation: tyre-force jitter in powered turns) |
| 008 (planned) | Lofted body cross sections; enclosed car envelope | Not implemented |
| 009 (planned) | Geometry Nodes instancing for wheels and trim | Not implemented |
| 010 (planned) | Simplified aero geometry and controlled parameter sweeps | Not implemented |

Give each experiment an input configuration, generator, README, generated output,
and validation report. Keep source inputs separate from rebuildable artifacts.
