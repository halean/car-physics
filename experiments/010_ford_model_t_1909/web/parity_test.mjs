// Check the browser port against the Python validation: run the sprung bumpy-road scenario of
// bumpy_road.py with sim.js on MuJoCo's WebAssembly build and compare its figures.
// node experiments/010_ford_model_t_1909/web/parity_test.mjs   (Node 22+, after build.py)
import { readFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));
const out = join(here, '../output/web');
const physics = join(here, '../output/full/physics');
const { ModelTSim } = await import(pathToFileURL(join(here, 'sim.js')));
const { default: loadMujoco } = await import(pathToFileURL(join(out, 'mujoco.js')));
const mujoco = await loadMujoco({ wasmBinary: readFileSync(join(out, 'mujoco.wasm')) });
const config = JSON.parse(readFileSync(join(out, 'config.json')));
const files = Object.fromEntries(config.files.map((f) => [f, new Uint8Array(readFileSync(join(out, f)))]));
const python = JSON.parse(readFileSync(join(physics, 'bumpy_road.json')));
if (python.source_sha256 !== config.source_sha256) throw new Error('bumpy_road.json is from another build');

const sim = new ModelTSim(mujoco, 'model_t_bumps.xml', files, config);
const d = sim.data, dt = sim.model.opt.timestep;
const bars = config.bars, start = bars[0].x - 3, end = 58.0;
const rows = [];
let onCourse = 0, lifted = 0;
const t0 = performance.now();
while (d.time < 13.0) {
  if (d.time >= 0.5 - 1e-9 && sim.ui.gear !== 1) { sim.set({ gear: 1 }); sim.gear = 1; sim.since = 0.5; }
  sim.step(1);
  const x = d.qpos[0];
  if (x > start && x < end) {
    onCourse++;
    if (sim.wheelsOnGround().size < 4) lifted++;
  }
  rows.push([d.time, x, sim.seatVz()]);
}
const wall = (performance.now() - t0) / 1000;
// Seat acceleration exactly as bumpy_road.py: 20-sample moving average (np.convolve 'same'), np.gradient.
const n = Math.max(1, Math.round(0.02 / dt)), N = rows.length, off = Math.floor((n - 1) / 2);
const v = rows.map((_, i) => {
  let s = 0;
  for (let k = 0; k < n; k++) { const j = i + off - k; if (j >= 0 && j < N) s += rows[j][2]; }
  return s / n;
});
const acc = v.map((_, i) => i === 0 ? (v[1] - v[0]) / (rows[1][0] - rows[0][0])
  : i === N - 1 ? (v[i] - v[i - 1]) / (rows[i][0] - rows[i - 1][0])
    : (v[i + 1] - v[i - 1]) / (rows[i + 1][0] - rows[i - 1][0]));
let sum = 0, cnt = 0, peak = 0;
rows.forEach((r, i) => { if (r[1] > start && r[1] < end) { sum += acc[i] ** 2; cnt++; peak = Math.max(peak, Math.abs(acc[i])); } });
const js = {
  rms_seat_vertical_acc_g: Math.sqrt(sum / cnt) / 9.81, peak_seat_vertical_acc_g: peak / 9.81,
  wheel_off_ground_fraction: lifted / onCourse, final_x_m: d.qpos[0], final_speed_mps: Math.hypot(d.qvel[0], d.qvel[1]),
};
const py = python.results.sprung;
const rel = (a, b) => Math.abs(a - b) / Math.abs(b);
const limits = { rms_seat_vertical_acc_g: 0.05, final_x_m: 0.01, final_speed_mps: 0.02, wheel_off_ground_fraction: 0.15 };
const table = Object.keys(js).map((k) => ({ metric: k, python: +py[k].toFixed(4), wasm: +js[k].toFixed(4),
  rel_diff: +rel(js[k], py[k]).toFixed(4), limit: limits[k] ?? 'report' }));
console.table(table);
console.log(`MuJoCo ${mujoco.mj_versionString?.() ?? '?'} (WASM), ${13 / wall < 1 ? 'slower' : 'faster'} than real time: ` +
  `${(13 / wall).toFixed(2)}x (13 s simulated in ${wall.toFixed(1)} s, single thread, contact check every step)`);
const pass = Object.entries(limits).every(([k, lim]) => rel(js[k], py[k]) < lim);
console.log(pass ? 'PARITY_OK' : 'PARITY_FAIL');
process.exit(pass ? 0 : 1);
