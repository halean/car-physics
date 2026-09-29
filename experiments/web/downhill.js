// Rolling downhill: the validated Panhard (1891) on a slope, with its forces drawn.
// Inlined after sim.js by build.py. The slope tilts gravity on the flat road (the same physics as
// tilting the road; checked against the car's validated 5-degree runs). The rim-block brake is
// friction F = mu * P at the tyre, so its torque per rear wheel is mu * P * r.
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const CAR = 'panhard_1891';
const MU_DESIGN = 0.5;          // assumed block friction; with P below it gives the validated 180 N m
const SCALE = 1 / 3000;         // arrow metres per newton
const $ = (id) => document.getElementById(id);
const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
const MINUS = '−';
const newtons = (v) => `${Math.round(v).toLocaleString('en-GB')} N`;
const signedN = (v) => `${Math.round(v) < 0 ? MINUS : ''}${Math.abs(Math.round(v)).toLocaleString('en-GB')} N`;
function status(text, isError = false) { const el = $('status'); el.hidden = !text; el.textContent = text || ''; el.classList.toggle('error', isError); }

async function fetchBytes(name, onProgress) {
  const res = await fetch(name);
  if (!res.ok) throw new Error(`Could not load ${name} (HTTP ${res.status}).`);
  const total = +res.headers.get('content-length') || 0;
  if (!onProgress || !res.body || !total) return new Uint8Array(await res.arrayBuffer());
  const reader = res.body.getReader(), parts = [];
  let got = 0;
  for (;;) { const { done, value } = await reader.read(); if (done) break; parts.push(value); got += value.length; onProgress(got / total); }
  const out = new Uint8Array(got);
  let at = 0;
  for (const p of parts) { out.set(p, at); at += p.length; }
  return out;
}

// ---------- scene: everything physical sits in `hill`, rotated by the slope angle ----------
const viewport = $('viewport');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
viewport.prepend(renderer.domElement);
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(34, 1, 0.05, 400);
camera.up.set(0, 0, 1);
const orbit = new OrbitControls(camera, renderer.domElement);
orbit.enableDamping = true;
orbit.minDistance = 3;
orbit.maxDistance = 25;
scene.add(new THREE.HemisphereLight(0xf4f1e6, 0x4a5540, 1.4));
const sun = new THREE.DirectionalLight(0xfff6e2, 2.3);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, { left: -6, right: 6, top: 6, bottom: -6, near: 1, far: 30 });
scene.add(sun, sun.target);
const hill = new THREE.Group();
scene.add(hill);
const ground = new THREE.Mesh(new THREE.PlaneGeometry(700, 120), new THREE.MeshStandardMaterial({ roughness: 1 }));
ground.position.set(300, 0, -0.003);
ground.receiveShadow = true;
const road = new THREE.Mesh(new THREE.PlaneGeometry(700, 3.2), new THREE.MeshStandardMaterial({ roughness: 0.95 }));
road.position.set(300, 0, -0.001);
road.receiveShadow = true;
hill.add(ground, road);
const postMat = new THREE.MeshStandardMaterial({ color: 0xe9e4d4, roughness: 0.8 });
for (let x = 0; x <= 400; x += 5) {   // posts every 5 m, taller every 10 m
  const h = x % 10 ? 0.35 : 0.6;
  const post = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.06, h), postMat);
  post.position.set(x, 1.9, h / 2);   // far side of the road, behind the car
  post.castShadow = true;
  hill.add(post);
}
function paintScene() {
  scene.background = new THREE.Color(css('--sky'));
  scene.fog = new THREE.Fog(css('--sky'), 50, 180);
  ground.material.color.set(css('--field'));
  road.material.color.set(css('--road'));
  for (const [arrow, token] of arrowColors) arrow.setColor(new THREE.Color(css(token)));
}
function resize() {
  const w = viewport.clientWidth, h = viewport.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(viewport);

// Free-body arrows at the centre of mass (in the hill's frame: gravity tilts, the road is flat).
const arrows = {}, arrowColors = [];
function makeArrow(name, token, faint = false) {
  const a = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 1, 0x000000, 0.18, 0.1);
  a.line.material.linewidth = 2;
  if (faint) { a.line.material.transparent = a.cone.material.transparent = true; a.line.material.opacity = a.cone.material.opacity = 0.45; }
  a.cone.material.depthTest = a.line.material.depthTest = false;
  a.renderOrder = 10;
  hill.add(a);
  arrows[name] = a;
  arrowColors.push([a, token]);
}
makeArrow('weight', '--weight'); makeArrow('along', '--weight', true); makeArrow('into', '--weight', true);
makeArrow('normal', '--normal'); makeArrow('friction', '--friction'); makeArrow('net', '--net');
function setArrow(name, origin, v) {
  const a = arrows[name], len = v.length() * SCALE;
  a.visible = len > 0.02;
  if (!a.visible) return;
  a.position.copy(origin);
  a.setDirection(v.clone().normalize());
  a.setLength(len, Math.min(0.18, len * 0.4), Math.min(0.1, len * 0.25));
}

// ---------- simulation ----------
let mujoco, config, files, sim, bodyGroups = {}, mass = 0, blockP = 0;
let braking = false, brakeCmd = 0, slow = false, lastCar = null;
const history = [];          // [t, speed, braking]
const velHist = [];          // [t, vx] for the measured acceleration

function slope() { return +$('slope').value * Math.PI / 180; }
function applySettings() {
  const th = slope(), g = sim.model.opt.gravity;
  g[0] = 9.81 * Math.sin(th); g[1] = 0; g[2] = -9.81 * Math.cos(th);
  hill.rotation.y = th;
  config.pedal.devices[0].capacity = +$('mu').value * blockP * config.rear_radius;   // mu * P * r per rear wheel
  $('slopeOut').textContent = `${(+$('slope').value).toFixed(1)}°`;
  $('muOut').textContent = (+$('mu').value).toFixed(2);
}
function restart() {
  sim.reset({ ...CarSim.defaults(config), gear: 0, throttle: 0, driver: false, steer: 0, pedal: 0, lever: 0 });
  braking = false; brakeCmd = 0;
  $('brake').setAttribute('aria-pressed', 'false');
  history.length = 0; velHist.length = 0;
  applySettings();
}
function setBrake(on) { braking = on; $('brake').setAttribute('aria-pressed', String(on)); }

function contactForces() {
  // Normal and friction force of the road on the tyres, from the constraint solver (elliptic cone:
  // rows normal, tangent 1, tangent 2 in the contact frame), in the hill frame.
  const d = sim.data, vec = d.contact, ef = d.efc_force, N = new THREE.Vector3(), T = new THREE.Vector3();
  try {
    for (let i = 0; i < d.ncon; i++) {
      const c = vec.get(i), fr = c.frame, a = c.efc_address;
      const sign = sim.tyre.has(c.geom2) ? 1 : sim.tyre.has(c.geom1) ? -1 : 0;
      if (sign && a >= 0) {
        for (let k = 0; k < 3; k++) {
          N.setComponent(k, N.getComponent(k) + sign * ef[a] * fr[k]);
          T.setComponent(k, T.getComponent(k) + sign * (ef[a + 1] * fr[3 + k] + ef[a + 2] * fr[6 + k]));
        }
      }
      c.delete?.();
    }
  } finally { vec.delete(); }
  return { N, T };
}

function drawChart() {
  const canvas = $('vt'), ctx = canvas.getContext('2d');
  const w = canvas.clientWidth, h = canvas.clientHeight, dpr = Math.min(devicePixelRatio, 2);
  if (canvas.width !== Math.round(w * dpr)) { canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr); }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  const pad = { l: 58, r: 8, t: 8, b: 22 };
  const tMax = Math.max(10, history.length ? history[history.length - 1][0] : 0);
  const vMax = Math.max(2, ...history.map((p) => p[1])) * 1.1;
  const x = (t) => pad.l + t / tMax * (w - pad.l - pad.r);
  const y = (v) => pad.t + (1 - v / vMax) * (h - pad.t - pad.b);
  ctx.fillStyle = css('--friction'); ctx.globalAlpha = 0.12;
  for (let i = 1; i < history.length; i++) if (history[i][2]) ctx.fillRect(x(history[i - 1][0]), pad.t, x(history[i][0]) - x(history[i - 1][0]) + 0.5, h - pad.t - pad.b);
  ctx.globalAlpha = 1;
  ctx.strokeStyle = css('--line'); ctx.fillStyle = css('--muted');
  ctx.font = '11px "JetBrains Mono", ui-monospace, monospace';
  const vStep = vMax > 8 ? 2 : vMax > 4 ? 1 : 0.5;
  ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
  for (let v = 0; v <= vMax + 1e-9; v += vStep) { ctx.beginPath(); ctx.moveTo(pad.l, y(v)); ctx.lineTo(w - pad.r, y(v)); ctx.stroke(); ctx.fillText(`${+v.toFixed(1)} m/s`, pad.l - 5, y(v)); }
  ctx.textAlign = 'center'; ctx.textBaseline = 'alphabetic';
  const tStep = tMax > 30 ? 10 : tMax > 12 ? 5 : 2;
  for (let t = 0; t <= tMax; t += tStep) ctx.fillText(`${t} s`, x(t), h - 6);
  ctx.strokeStyle = css('--green'); ctx.lineWidth = 2.5;
  ctx.beginPath();
  history.forEach((p, i) => (i ? ctx.lineTo(x(p[0]), y(p[1])) : ctx.moveTo(x(p[0]), y(p[1]))));
  ctx.stroke();
  ctx.lineWidth = 1;
}

let lastFrame = 0, lastPanel = 0;
function frame(now) {
  requestAnimationFrame(frame);
  if (!sim) { renderer.render(scene, camera); return; }
  const wall = Math.min(0.05, (now - (lastFrame || now)) / 1000) * (slow ? 0.25 : 1);
  lastFrame = now;
  const d = sim.data, dt = sim.model.opt.timestep;
  for (let i = 0, n = Math.min(60, Math.round(wall / dt)); i < n; i++) {
    brakeCmd = Math.max(0, Math.min(1, brakeCmd + (braking ? 1 : -1) * dt / config.engine.brake_ramp_seconds));
    sim.set({ pedal: brakeCmd });
    sim.step(1);
  }
  sim.mj.mj_subtreeVel(sim.model, d);   // whole car's centre of mass: sum F = m a holds for it
  const vx = d.subtree_linvel[3 * sim.chassis];
  velHist.push([d.time, vx]);
  while (velHist.length > 2 && d.time - velHist[0][0] > 0.2) velHist.shift();
  history.push([d.time, Math.hypot(d.qvel[0], d.qvel[1]), braking]);
  if (history.length > 4000) history.splice(0, history.length - 4000);
  // Car meshes follow their bodies (hill frame).
  const poses = sim.poses();
  for (const [name, p] of Object.entries(poses)) {
    const g = bodyGroups[name];
    if (g) { g.position.set(p[0], p[1], p[2]); g.quaternion.set(p[4], p[5], p[6], p[3]); }
  }
  // Forces at the whole car's centre of mass.
  const c = sim.chassis, sc = d.subtree_com, com = new THREE.Vector3(sc[3 * c], sc[3 * c + 1], sc[3 * c + 2]);
  const th = slope(), W = mass * 9.81;
  const { N, T } = contactForces();
  const first = velHist[0], last = velHist[velHist.length - 1];
  const a = last[0] > first[0] ? (last[1] - first[1]) / (last[0] - first[0]) : 0;
  setArrow('weight', com, new THREE.Vector3(W * Math.sin(th), 0, -W * Math.cos(th)));
  setArrow('along', com, new THREE.Vector3(W * Math.sin(th), 0, 0));
  setArrow('into', com, new THREE.Vector3(0, 0, -W * Math.cos(th)));
  setArrow('normal', com, N);
  setArrow('friction', com, new THREE.Vector3(T.x, 0, 0));
  setArrow('net', com.clone().add(new THREE.Vector3(0, 0, 0.9)), new THREE.Vector3(mass * a, 0, 0));
  // Camera follows the car across the slope; the viewer can still orbit.
  const target = hill.localToWorld(new THREE.Vector3(poses.chassis[0] + 0.8, 0, 0.5));
  if (lastCar) { const delta = target.clone().sub(lastCar); camera.position.add(delta); orbit.target.add(delta); }
  lastCar = target;
  sun.position.set(target.x - 3, target.y - 6, target.z + 9);
  sun.target.position.copy(target);
  orbit.update();
  renderer.render(scene, camera);
  if (now - lastPanel > 120) {
    lastPanel = now;
    const speed = Math.hypot(d.qvel[0], d.qvel[1]);
    $('speed').textContent = speed.toFixed(2);
    $('dist').textContent = d.qpos[0].toFixed(1);
    $('time').textContent = d.time.toFixed(1);
    const F = -T.x, along = W * Math.sin(th);
    $('fW').textContent = newtons(W);
    $('fWx').textContent = newtons(along);
    $('fWz').textContent = newtons(W * Math.cos(th));
    $('fN').textContent = newtons(N.length());
    $('fF').textContent = `${signedN(F)} uphill`;
    $('fNet').textContent = `${signedN(along - F)} downhill`;
    $('cSum').textContent = signedN(along - F);
    $('cMa').textContent = signedN(mass * a);
    $('cA').textContent = `${a < 0 ? MINUS : ''}${Math.abs(a).toFixed(3)} m/s²`;
    const diff = Math.abs(along - F - mass * a);
    $('cOk').textContent = diff < Math.max(15, 0.03 * Math.abs(mass * a)) ? `They agree (within ${Math.round(diff)} N).` : 'Settling… (the car is changing speed quickly)';
    const r = config.rear_radius, v = Math.abs(d.qvel[0]);
    const spin = sim.rearDofs.reduce((s, dof) => s + Math.abs(d.qvel[dof]), 0) / sim.rearDofs.length;
    $('skid').hidden = !(v > 0.2 && 1 - spin * r / v > 0.5);
    drawChart();
  }
}

async function start() {
  resize();
  requestAnimationFrame(frame);
  $('slope').addEventListener('input', () => sim && applySettings());
  $('mu').addEventListener('input', () => sim && applySettings());
  $('brake').addEventListener('click', () => setBrake(!braking));
  $('restart').addEventListener('click', () => sim && restart());
  $('slow').addEventListener('click', () => { slow = !slow; $('slow').setAttribute('aria-pressed', String(slow)); $('slow').textContent = slow ? 'Normal speed' : 'Slow motion'; });
  addEventListener('keydown', (e) => {
    if (e.target.closest?.('button, input') && e.key !== 'b' && e.key !== 'r') return;
    if (e.key === 'b' || e.key === 'B') { setBrake(!braking); e.preventDefault(); }
    if ((e.key === 'r' || e.key === 'R') && sim) { restart(); e.preventDefault(); }
  });
  try {
    if (typeof WebAssembly !== 'object') throw new Error('This browser has no WebAssembly support.');
    const manifest = await (await fetch('manifest.json')).json();
    const base = `cars/${CAR}/`;
    config = await (await fetch(base + 'config.json')).json();
    const [wasm, gltf] = await Promise.all([
      fetchBytes('mujoco.wasm', (f) => status(`Loading the MuJoCo physics engine… ${Math.round(f * 100)}%`)),
      new GLTFLoader().loadAsync(base + 'car.glb'),
    ]);
    files = Object.fromEntries(await Promise.all(config.files.map(async (f) => [f, await fetchBytes(base + f)])));
    for (const node of [...gltf.scene.children]) {
      const body = node.name.split('|')[0];
      if (!bodyGroups[body]) { bodyGroups[body] = new THREE.Group(); hill.add(bodyGroups[body]); }
      node.traverse((o) => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });
      bodyGroups[body].add(node);
    }
    status('Starting MuJoCo…');
    const { default: loadMujoco } = await import(`https://cdn.jsdelivr.net/npm/@mujoco/mujoco@${manifest.mujoco_version}/mujoco.js`);
    mujoco = await loadMujoco({ wasmBinary: wasm.buffer });
    sim = new CarSim(mujoco, config, files, 'flat');
    for (let b = 0; b < sim.model.nbody; b++) mass += sim.model.body_mass[b];
    blockP = config.pedal.devices[0].capacity / (MU_DESIGN * config.rear_radius);   // the validated torque at mu = 0.5
    $('pOut').textContent = Math.round(blockP).toLocaleString('en-GB');
    paintScene();
    matchMedia('(prefers-color-scheme: dark)').addEventListener('change', paintScene);
    new MutationObserver(paintScene).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });
    restart();
    const target = new THREE.Vector3(0.8, 0, 0.5).applyEuler(hill.rotation);
    orbit.target.copy(target);
    camera.position.set(target.x - 0.8, target.y - 7.5, target.z + 1.0);
    lastCar = target;
    status('');
  } catch (err) {
    console.error(err);
    status(`Could not start the simulation: ${err.message}`, true);
  }
}
start();
