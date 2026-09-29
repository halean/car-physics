// Checks every number the Rolling downhill page states, with the page's own physics setup
// (sim.js, tilted gravity, rim-block capacity mu * P * r). node experiments/web/downhill_test.mjs
// Limits fixed before the first run.
import { readFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));
const site = join(here, 'output/site');
const { CarSim } = await import(pathToFileURL(join(here, 'sim.js')));
const { default: load } = await import(pathToFileURL(join(site, 'mujoco.js')));
const mj = await load({ wasmBinary: readFileSync(join(site, 'mujoco.wasm')) });
const dir = join(site, 'cars/panhard_1891');
const config = JSON.parse(readFileSync(join(dir, 'config.json')));
const files = Object.fromEntries(config.files.map((f) => [f, new Uint8Array(readFileSync(join(dir, f)))]));
const report = JSON.parse(readFileSync(join(here, '../007_panhard_1891/output/full/physics/report.json')));
const CAP = config.pedal.devices[0].capacity, P = CAP / (0.5 * config.rear_radius);

function drive({ deg, mu = 0.5, brakeAt = null, seconds }) {
  const cfg = structuredClone(config);
  cfg.pedal.devices[0].capacity = mu * P * cfg.rear_radius;
  const sim = new CarSim(mj, cfg, files, 'flat');
  sim.reset({ ...CarSim.defaults(cfg), gear: 0, throttle: 0, driver: false, steer: 0, pedal: 0, lever: 0 });
  const th = deg * Math.PI / 180, g = sim.model.opt.gravity;
  g[0] = 9.81 * Math.sin(th); g[1] = 0; g[2] = -9.81 * Math.cos(th);
  let mass = 0;
  for (let b = 0; b < sim.model.nbody; b++) mass += sim.model.body_mass[b];
  const d = sim.data, out = { rows: [], mass };
  let xa = null;
  // mj_subtreeVel uses the body velocities of the latest forward pass, i.e. the state at the start of
  // the last step. So after step j it gives V_j, and efc_force holds F_j, the forces that act over
  // step j; the matching acceleration (V_{j+1} - V_j) / dt is known one step later.
  const comVx = () => { mj.mj_subtreeVel(sim.model, d); return d.subtree_linvel[3 * sim.chassis]; };
  const roadFx = () => {
    const vec = d.contact, ef = d.efc_force;
    let Tx = 0;
    for (let i = 0; i < d.ncon; i++) {
      const c = vec.get(i), fr = c.frame, a = c.efc_address;
      const sign = sim.tyre.has(c.geom2) ? 1 : sim.tyre.has(c.geom1) ? -1 : 0;
      if (sign && a >= 0) Tx += sign * (ef[a + 1] * fr[3] + ef[a + 2] * fr[6]);
    }
    vec.delete();
    return Tx;
  };
  const along = mass * 9.81 * Math.sin(th);
  let prev = null;   // { V, F } after the previous step
  while (d.time < seconds - 1e-9) {
    if (brakeAt !== null) sim.set({ pedal: Math.max(0, Math.min(1, (d.time - brakeAt) / cfg.engine.brake_ramp_seconds)) });
    sim.step(1);
    if (brakeAt !== null && xa === null && d.time >= brakeAt - 1e-9) xa = d.qpos[0];
    const now = { V: comVx(), F: -roadFx() };
    if (prev && Math.round(d.time * 1000) % 100 === 0) {
      const aCom = (now.V - prev.V) / sim.model.opt.timestep;          // acceleration over the step that prev.F acted on
      out.rows.push({ t: d.time, x: d.qpos[0], v: d.qvel[0], a: aCom, F: prev.F, along, aChassis: d.qacc[0] });
    }
    prev = now;
  }
  out.x = d.qpos[0]; out.v = d.qvel[0]; out.xa = xa;
  sim.dispose();
  return out;
}

const results = [];
const check = (name, value, pass, expect) => results.push({ check: name, value, expect, pass });
const gate = drive({ deg: 5, seconds: 4 });
check('5 deg, 4 s rolled (validated gate)', +gate.x.toFixed(4), Math.abs(gate.x / report.downhill_gate.forward_m - 1) < 0.01,
  `${report.downhill_gate.forward_m.toFixed(4)} m ±1%`);
const stop = drive({ deg: 5, brakeAt: 2.0, seconds: 12 });
const ref = report.dynamics.scenarios.rim_brakes.stopping.stopping_distance_m;
check('5 deg, brake at 2 s: distance to stop', +(stop.x - stop.xa).toFixed(4), Math.abs((stop.x - stop.xa) / ref - 1) < 0.01, `${ref.toFixed(4)} m ±1%`);
const worst = Math.max(...gate.rows.concat(stop.rows).filter((r) => r.t > 0.5).map((r) => Math.abs(r.along - r.F - gate.mass * r.a)));
check('sum F = m a (all samples, both runs)', +worst.toFixed(3), worst < 1, '< 1 N');
const at = gate.rows.find((r) => Math.abs(r.t - 3) < 1e-6);
check('challenge 1: free-rolling a at 5 deg', +at.a.toFixed(3), at.a > 0.75 && at.a < 0.81, 'about 0.78 m/s²');
check('challenge 1: road force F, no brake', +at.F.toFixed(1), at.F > 30 && at.F < 60, 'about 45 N');
for (const [deg, holds] of [[6.5, true], [7.5, false]]) {
  const r = drive({ deg, brakeAt: 0, seconds: 8 });
  const moving = Math.abs(r.v) > 0.02;
  check(`challenge 2: ${deg} deg, brake from the start`, `${r.v.toFixed(3)} m/s at 8 s`, holds ? !moving : moving && r.v > r.rows[40].v,
    holds ? 'holds' : 'creeps, getting faster');
}
for (const [mu, holds] of [[0.65, false], [0.75, true]]) {
  const r = drive({ deg: 10, mu, brakeAt: 0, seconds: 8 });
  const moving = Math.abs(r.v) > 0.02;
  check(`challenge 3: 10 deg, mu = ${mu}`, `${r.v.toFixed(3)} m/s at 8 s`, holds ? !moving : moving, holds ? 'holds' : 'creeps');
}
check('block force P', +P.toFixed(1), Math.abs(P - 720) < 0.5, '720 N (page text)');
console.table(results);
const ok = results.every((r) => r.pass);
console.log(ok ? 'DOWNHILL_OK' : 'DOWNHILL_FAIL');
process.exit(ok ? 0 : 1);
