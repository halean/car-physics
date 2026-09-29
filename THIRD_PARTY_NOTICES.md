# Third-party notices

This repository's own code is MIT-licensed and its written material is
CC BY 4.0 (see [LICENSE](LICENSE) and [LICENSE-docs.md](LICENSE-docs.md)).
It depends on the following work by others. None of it is committed to the
repository, but the built web site includes MuJoCo and loads the rest.

| Component | Used for | Licence | How it reaches the site |
| --- | --- | --- | --- |
| [MuJoCo](https://github.com/google-deepmind/mujoco) (Google DeepMind), `@mujoco/mujoco` 3.14.0 | Physics engine: Python validation, and the browser build | Apache License 2.0 | `mujoco.wasm` and `mujoco.js` are copied into the built site; the JavaScript also loads from jsDelivr |
| [three.js](https://github.com/mrdoob/three.js) 0.181.0 | 3D rendering in the browser | MIT | Loaded from jsDelivr, not copied |
| [Barlow, Barlow Condensed](https://github.com/jpt/barlow), [JetBrains Mono](https://github.com/JetBrains/JetBrainsMono) | Page typefaces | SIL Open Font License 1.1 | Loaded from Google Fonts, not copied |
| [Blender](https://www.blender.org) | Generates the car geometry (not part of the output) | GNU GPL (the program only; its output is not covered) | Not included |

The Apache License 2.0 requires a copy of the licence with any redistribution.
The site build (`experiments/web/build.py`) therefore writes this file and the
Apache licence text next to `mujoco.wasm`.

Historical facts about the cars come from the sources listed in
`car_tech/sources.md`. The descriptions are this project's own wording; no
text or images are reproduced from those sources.
