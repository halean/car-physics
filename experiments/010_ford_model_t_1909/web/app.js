// Model T Spring Lab: page logic. Inlined after sim.js by build.py; uses THREE, GLTFLoader and
// OrbitControls from the import map and MuJoCo's official WebAssembly build (3.14.0).
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const $ = (id) => document.getElementById(id);
const status = (text, isError = false) => {
  const el = $('status');
  el.hidden = !text;
  el.textContent = text || '';
  el.classList.toggle('error', isError);
};

async function fetchBytes(name, onProgress) {
  const res = await fetch(name);
  if (!res.ok) throw new Error(`Could not load ${name} (HTTP ${res.status}).`);
  const total = +res.headers.get('content-length') || 0;
  if (!onProgress || !res.body || !total) return new Uint8Array(await res.arrayBuffer());
  const reader = res.body.getReader(), parts = [];
  let got = 0;
  for (;;) {
    const { done, value } = await reader.read();
    if (done) break;
    parts.push(value);
    got += value.length;
    onProgress(got / total);
  }
  const out = new Uint8Array(got);
  let at = 0;
  for (const p of parts) { out.set(p, at); at += p.length; }
  return out;
}

// ---------- 3D scene ----------
const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
const viewport = $('viewport');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
viewport.prepend(renderer.domElement);
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(38, 1, 0.05, 400);
camera.up.set(0, 0, 1);
camera.position.set(-1.6, -4.6, 1.9);
const orbit = new OrbitControls(camera, renderer.domElement);
orbit.target.set(1.3, 0, 0.6);
orbit.enableDamping = true;
orbit.maxPolarAngle = Math.PI * 0.49;
orbit.minDistance = 2.5;
orbit.maxDistance = 30;
scene.add(new THREE.HemisphereLight(0xf4f1e6, 0x4a5540, 1.4));
const sun = new THREE.DirectionalLight(0xfff6e2, 2.4);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, { left: -6, right: 6, top: 6, bottom: -6, near: 1, far: 30 });
scene.add(sun, sun.target);

function paintScene() {
  scene.background = new THREE.Color(css('--sky'));
  scene.fog = new THREE.Fog(css('--sky'), 40, 160);
  ground.material.color.set(css('--field'));
  road.material.color.set(css('--road'));
}
const ground = new THREE.Mesh(new THREE.PlaneGeometry(600, 200), new THREE.MeshStandardMaterial({ roughness: 1 }));
ground.position.set(150, 0, -0.002);
ground.receiveShadow = true;
const road = new THREE.Mesh(new THREE.PlaneGeometry(600, 3.6), new THREE.MeshStandardMaterial({ roughness: 0.95 }));
road.position.set(150, 0, -0.001);
road.receiveShadow = true;
scene.add(ground, road);
// Distance posts every 10 m, so speed reads against the roadside.
const postMat = new THREE.MeshStandardMaterial({ color: 0xe9e4d4, roughness: 0.8 });
for (let x = 0; x <= 300; x += 10) {
  const post = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.06, 0.5), postMat);
  post.position.set(x, -2.3, 0.25);
  post.castShadow = true;
  scene.add(post);
}
const barGroup = new THREE.Group();
scene.add(barGroup);
const barMat = new THREE.MeshStandardMaterial({ color: 0x8a6a45, roughness: 0.85 });
function drawBars(bars) {
  barGroup.clear();
  for (const b of bars) {
    const [y0, y1] = { full: [-1.3, 1.3], left: [0.25, 1.3], right: [-1.3, -0.25] }[b.side];
    const bar = new THREE.Mesh(new THREE.CapsuleGeometry(b.height, y1 - y0, 6, 16), barMat);
    bar.position.set(b.x, (y0 + y1) / 2, 0);
    bar.receiveShadow = true;
    barGroup.add(bar);
  }
}

const bodyGroups = {};
async function loadCar() {
  const gltf = await new GLTFLoader().loadAsync('model_t.glb');
  const parts = [...gltf.scene.children];
  for (const node of parts) {
    const body = node.name.split('|')[0];
    if (!bodyGroups[body]) { bodyGroups[body] = new THREE.Group(); scene.add(bodyGroups[body]); }
    node.traverse((o) => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });
    bodyGroups[body].add(node);
  }
}

function resize() {
  const w = viewport.clientWidth, h = viewport.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(viewport);

// ---------- strip chart: axle travel as % of the stop ----------
const chart = $('chart'), ctx = chart.getContext('2d');
const history = [];
function drawChart() {
  const w = chart.clientWidth, h = chart.clientHeight, dpr = Math.min(devicePixelRatio, 2);
  if (chart.width !== w * dpr) { chart.width = w * dpr; chart.height = h * dpr; }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  const pad = { l: 38, r: 8, t: 8, b: 18 }, span = 8;
  const y = (v) => pad.t + (1 - (v + 120) / 240) * (h - pad.t - pad.b);
  const now = history.length ? history[history.length - 1][0] : 0;
  const x = (t) => pad.l + (1 - (now - t) / span) * (w - pad.l - pad.r);
  ctx.font = '11px "JetBrains Mono", ui-monospace, monospace';
  ctx.fillStyle = css('--muted');
  ctx.textAlign = 'right';
  ctx.textBaseline = 'middle';
  for (const v of [-100, 0, 100]) {
    ctx.strokeStyle = v ? css('--warn') : css('--line');
    ctx.setLineDash(v ? [4, 4] : []);
    ctx.beginPath(); ctx.moveTo(pad.l, y(v)); ctx.lineTo(w - pad.r, y(v)); ctx.stroke();
    ctx.fillText(v ? `${v > 0 ? '+' : '−'}100%` : '0', pad.l - 5, y(v));
  }
  ctx.setLineDash([]);
  ctx.textAlign = 'left';
  ctx.fillText('stop (compressed)', pad.l + 4, y(100) - 8);
  ctx.textBaseline = 'alphabetic';
  ctx.fillText('last 8 s', pad.l + 4, h - 4);
  for (const [i, key] of [[1, '--brass'], [2, '--green']]) {
    ctx.strokeStyle = css(key);
    ctx.lineWidth = 2;
    ctx.beginPath();
    history.forEach((p, k) => { const px = x(p[0]), py = y(p[i]); k ? ctx.lineTo(px, py) : ctx.moveTo(px, py); });
    ctx.stroke();
    ctx.lineWidth = 1;
  }
}

// ---------- simulation ----------
let mujoco, config, files, sim, course = 'bumps';

function buildSim(name) {
  if (sim) { sim.data.delete(); sim.model.delete(); sim.vfs.delete(); }
  sim = new ModelTSim(mujoco, `model_t_${name}.xml`, files, config);
  sim.set(readControls());
  drawBars(name === 'bumps' ? config.bars : []);
  history.length = 0;
  lastCar = null;
}

function readControls() {
  return {
    gear: +document.querySelector('input[name="gear"]:checked').value,
    throttle: +$('throttle').value / 100, footBrake: +$('footBrake').value / 100, handBrake: +$('handBrake').value / 100,
    driver: $('driver').checked ? 1 : 0, steerDeg: +$('steer').value,
    frontRate: +$('frontRate').value, rearRate: +$('rearRate').value,
    frontDamping: +$('frontDamping').value, rearDamping: +$('rearDamping').value,
    keepRideHeight: $('keepRide').checked ? 1 : 0,
  };
}

function showValues() {
  const c = readControls();
  $('throttleOut').textContent = `${Math.round(c.throttle * 100)}%`;
  $('footBrakeOut').textContent = `${Math.round(c.footBrake * 100)}%`;
  $('handBrakeOut').textContent = `${Math.round(c.handBrake * 100)}%`;
  $('steerOut').textContent = `${c.steerDeg > 0 ? '+' : ''}${c.steerDeg}°`;
  $('frontRateOut').textContent = `${c.frontRate} kN/m`;
  $('rearRateOut').textContent = `${c.rearRate} kN/m`;
  $('frontDampingOut').textContent = `${c.frontDamping.toFixed(2)} kN·s/m`;
  $('rearDampingOut').textContent = `${c.rearDamping.toFixed(2)} kN·s/m`;
  $('steer').disabled = !!c.driver;
}

function onControl() {
  showValues();
  if (sim) sim.set(readControls());
}
document.querySelectorAll('#controls input').forEach((el) => el.addEventListener('input', onControl));

const PRESETS = {
  design: { frontRate: 28, rearRate: 26, frontDamping: 1.56, rearDamping: 1.55, keepRide: true },
  soft: { frontRate: 10, rearRate: 10, frontDamping: 1.56, rearDamping: 1.55, keepRide: true },
  sag: { frontRate: 10, rearRate: 10, frontDamping: 1.56, rearDamping: 1.55, keepRide: false },
  stiff: { frontRate: 100, rearRate: 100, frontDamping: 1.56, rearDamping: 1.55, keepRide: true },
  bouncy: { frontRate: 28, rearRate: 26, frontDamping: 0.2, rearDamping: 0.2, keepRide: true },
};
document.querySelectorAll('[data-preset]').forEach((btn) => btn.addEventListener('click', () => {
  const p = PRESETS[btn.dataset.preset];
  for (const k of ['frontRate', 'rearRate', 'frontDamping', 'rearDamping']) $(k).value = p[k];
  $('keepRide').checked = p.keepRide;
  onControl();
}));
document.querySelectorAll('input[name="course"]').forEach((el) => el.addEventListener('change', () => {
  course = el.value;
  if (mujoco) buildSim(course);
}));
$('restart').addEventListener('click', () => { if (sim) { sim.reset(); sim.set(readControls()); history.length = 0; lastCar = null; } });

// ---------- readouts ----------
const fmt = (v, d = 1) => v.toFixed(d);
function showReadout(r) {
  $('speed').textContent = fmt(r.speed * 3.6);
  $('distance').textContent = fmt(r.x);
  $('seatRms').textContent = fmt(r.seatRmsG, 2);
  for (const axle of ['front', 'rear']) {
    const a = r.axles[axle];
    const pct = Math.round(a.used * 100);
    $(`${axle}Travel`).textContent = `${pct}%`;
    $(`${axle}Offset`).textContent = `${a.offsetMm >= 0 ? '+' : '−'}${fmt(Math.abs(a.offsetMm))} mm`;
    $(`${axle}Bar`).style.width = `${Math.min(100, pct)}%`;
    $(`${axle}Gauge`).classList.toggle('on-stops', a.used >= 1);
    $(`${axle}Peak`).textContent = `peak ${Math.round(a.peak * 100)}%`;
  }
  for (const w of ['front_left', 'front_right', 'rear_left', 'rear_right']) $(w).classList.toggle('down', r.wheels.has(w));
  $('wheelsText').textContent = `${r.wheels.size} of 4 on the road`;
}

// ---------- main loop ----------
let lastCar = null, lastFrame = 0, lastReadout = 0, simSeconds = 0, wallSeconds = 0;
function frame(now) {
  requestAnimationFrame(frame);
  if (!sim) { renderer.render(scene, camera); return; }
  const wall = Math.min(0.05, (now - (lastFrame || now)) / 1000);
  lastFrame = now;
  const dt = sim.model.opt.timestep;
  const t0 = performance.now();
  const steps = Math.min(60, Math.round(wall / dt));
  sim.step(steps);
  simSeconds += steps * dt; wallSeconds += wall;
  const poses = sim.poses();
  for (const [name, p] of Object.entries(poses)) {
    const g = bodyGroups[name];
    if (g) { g.position.set(p[0], p[1], p[2]); g.quaternion.set(p[4], p[5], p[6], p[3]); }
  }
  // Follow the car, keeping the viewer's orbit.
  const car = new THREE.Vector3(poses.chassis[0] + 1.3, poses.chassis[1], 0.6);
  if (lastCar) { const delta = car.clone().sub(lastCar); camera.position.add(delta); orbit.target.add(delta); }
  lastCar = car;
  sun.position.set(car.x - 4, car.y - 6, 9);
  sun.target.position.copy(car);
  orbit.update();
  renderer.render(scene, camera);
  const r = now - lastReadout > 100 ? sim.readout() : null;   // readouts at 10 Hz (contacts are copied)
  const axles = sim.data.qpos, j = sim.joints;
  history.push([sim.data.time,
    -axles[j.front_swing.qpos] / j.front_swing.range * 100, axles[j.rear_swing.qpos] / j.rear_swing.range * 100]);
  while (history.length && history[history.length - 1][0] - history[0][0] > 8.2) history.shift();
  drawChart();
  if (r) {
    lastReadout = now;
    showReadout(r);
    const load = (performance.now() - t0) / Math.max(wall * 1000, 1);
    $('rate').textContent = wallSeconds > 1 ? `${fmt(simSeconds / wallSeconds, 2)}× real time` : '';
    if (wallSeconds > 3) { simSeconds = 0; wallSeconds = 0; }
    $('rate').dataset.load = load.toFixed(2);
  }
}

// ---------- start ----------
async function start() {
  resize();
  paintScene();
  matchMedia('(prefers-color-scheme: dark)').addEventListener('change', paintScene);
  new MutationObserver(paintScene).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });
  showValues();
  requestAnimationFrame(frame);
  try {
    if (typeof WebAssembly !== 'object') throw new Error('This browser has no WebAssembly support.');
    status('Loading the car…');
    config = await (await fetch('config.json')).json();
    const [wasm] = await Promise.all([
      fetchBytes('mujoco.wasm', (f) => status(`Loading MuJoCo physics engine… ${Math.round(f * 100)}%`)),
      loadCar(),
    ]);
    files = Object.fromEntries(await Promise.all(config.files.map(async (f) => [f, await fetchBytes(f)])));
    status('Starting MuJoCo…');
    const { default: loadMujoco } = await import(`https://cdn.jsdelivr.net/npm/@mujoco/mujoco@${config.mujoco_version}/mujoco.js`);
    mujoco = await loadMujoco({ wasmBinary: wasm.buffer });
    buildSim(course);
    status('');
    $('engine').textContent = `MuJoCo ${config.mujoco_version}, WebAssembly`;
  } catch (err) {
    console.error(err);
    const blocked = /WebAssembly|wasm|CompileError|Content Security/i.test(String(err));
    status(blocked
      ? `The physics engine could not start here: ${err.message}. This page needs WebAssembly. Run it locally with the command under "About this simulation".`
      : `Could not start the simulation: ${err.message}`, true);
  }
}
start();
