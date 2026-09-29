// For every car in the site, the browser port must reproduce the car's own Python control.py on
// the same validated model: launch in first gear at 0.5 s, pedal brake from 4 s (ramped as the
// car's Brake), steering held straight, 8 s. build.py stores the Python trajectory in config.json.
// node experiments/web/parity_test.mjs   (Node 22+, after build.py)
// Limits fixed before the first run: every half-second sample within 1 mm of position and 1 mm/s of speed.
import { readFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));
const site = join(here, 'output/site');
const { CarSim } = await import(pathToFileURL(join(here, 'sim.js')));
const { default: loadMujoco } = await import(pathToFileURL(join(site, 'mujoco.js')));
const mujoco = await loadMujoco({ wasmBinary: readFileSync(join(site, 'mujoco.wasm')) });
const manifest = JSON.parse(readFileSync(join(site, 'manifest.json')));
const ramp = (t, start, secs) => Math.max(0, Math.min(1, (t - start) / secs));

let allPass = true;
const table = [];
for (const { id } of manifest.cars) {
  const dir = join(site, 'cars', id);
  const config = JSON.parse(readFileSync(join(dir, 'config.json')));
  const files = Object.fromEntries(config.files.map((f) => [f, new Uint8Array(readFileSync(join(dir, f)))]));
  const sim = new CarSim(mujoco, config, files, 'flat');
  const p = config.parity;
  sim.reset({ ...CarSim.defaults(config), gear: 0, driver: false, steer: 0, pedal: 0, lever: 0, throttle: 1 });
  const d = sim.data, got = [];
  const t0 = performance.now();
  while (d.time < p.seconds - 1e-9) {
    if (d.time >= p.first_gear_at - 1e-9 && sim.gear === 0) { sim.set({ gear: 1 }); sim.gear = 1; sim.since = p.first_gear_at; }
    sim.set({ pedal: ramp(d.time, p.pedal_at, config.engine.brake_ramp_seconds) });
    sim.step(1);
    if (Math.abs(d.time / 0.5 - Math.round(d.time / 0.5)) < 1e-6) got.push([d.time, d.qpos[0], d.qpos[1], Math.hypot(d.qvel[0], d.qvel[1])]);
  }
  const wall = (performance.now() - t0) / 1000;
  let dx = 0, dv = 0;
  p.rows.forEach((row, i) => {
    dx = Math.max(dx, Math.hypot(row[1] - got[i][1], row[2] - got[i][2]));
    dv = Math.max(dv, Math.abs(row[3] - got[i][3]));
  });
  const pass = got.length === p.rows.length && dx < 1e-3 && dv < 1e-3;
  allPass &&= pass;
  const last = p.rows[p.rows.length - 1], peak = Math.max(...p.rows.map((r) => r[3]));
  table.push({ car: id, python_x_8s: +last[1].toFixed(3), wasm_x_8s: +got[got.length - 1][1].toFixed(3),
    peak_speed_mps: +peak.toFixed(2), max_pos_diff_mm: +(dx * 1000).toFixed(4), max_speed_diff_mmps: +(dv * 1000).toFixed(4),
    x_realtime: +(p.seconds / wall).toFixed(1), pass });
  sim.dispose();
}
console.table(table);
console.log(allPass ? 'PARITY_OK' : 'PARITY_FAIL');
process.exit(allPass ? 0 : 1);
