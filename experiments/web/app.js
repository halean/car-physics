// Brass Era Garage: page logic. Inlined after sim.js and input.js by build.py.
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const $ = (id) => document.getElementById(id);
const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
const fmt = (v, d = 1) => v.toFixed(d);
const pct = (v) => `${Math.round(v * 100)}%`;
const MINUS = '−';
const signed = (v, d = 1) => { const s = fmt(Math.abs(v), d); return `${+s === 0 ? '' : v > 0 ? '+' : MINUS}${s}`; };
function status(text, isError = false) { const el = $('status'); el.hidden = !text; el.textContent = text || ''; el.classList.toggle('error', isError); }
let toastTimer;
function toast(text) { const el = $('toast'); el.textContent = text; el.hidden = false; clearTimeout(toastTimer); toastTimer = setTimeout(() => { el.hidden = true; }, 1800); }

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

// ---------- 3D scene ----------
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
const orbit = new OrbitControls(camera, renderer.domElement);
orbit.enableDamping = true;
orbit.maxPolarAngle = Math.PI * 0.49;
orbit.minDistance = 2;
orbit.maxDistance = 30;
scene.add(new THREE.HemisphereLight(0xf4f1e6, 0x4a5540, 1.4));
const sun = new THREE.DirectionalLight(0xfff6e2, 2.4);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, { left: -6, right: 6, top: 6, bottom: -6, near: 1, far: 30 });
scene.add(sun, sun.target);
const ground = new THREE.Mesh(new THREE.PlaneGeometry(600, 200), new THREE.MeshStandardMaterial({ roughness: 1 }));
ground.position.set(150, 0, -0.002);
ground.receiveShadow = true;
const road = new THREE.Mesh(new THREE.PlaneGeometry(600, 3.6), new THREE.MeshStandardMaterial({ roughness: 0.95 }));
road.position.set(150, 0, -0.001);
road.receiveShadow = true;
scene.add(ground, road);
const postMat = new THREE.MeshStandardMaterial({ color: 0xe9e4d4, roughness: 0.8 });
for (let x = 0; x <= 300; x += 10) {
  const post = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.06, 0.5), postMat);
  post.position.set(x, -2.3, 0.25);
  post.castShadow = true;
  scene.add(post);
}
function paintScene() {
  scene.background = new THREE.Color(css('--sky'));
  scene.fog = new THREE.Fog(css('--sky'), 40, 160);
  ground.material.color.set(css('--field'));
  road.material.color.set(css('--road'));
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
function resize() {
  const w = viewport.clientWidth, h = viewport.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(viewport);

// ---------- strip charts ----------
function stripChart(canvas, series, { lo, hi, ticks, fmtTick, span = 8, fills = [] }) {
  const ctx = canvas.getContext('2d');
  const w = canvas.clientWidth, h = canvas.clientHeight, dpr = Math.min(devicePixelRatio, 2);
  if (canvas.width !== Math.round(w * dpr)) { canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr); }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  const pad = { l: 40, r: 6, t: 8, b: 16 };
  const y = (v) => pad.t + (1 - (v - lo) / (hi - lo)) * (h - pad.t - pad.b);
  const rows = series.rows, now = rows.length ? rows[rows.length - 1][0] : 0;
  const x = (t) => pad.l + (1 - (now - t) / span) * (w - pad.l - pad.r);
  ctx.font = '11px "JetBrains Mono", ui-monospace, monospace';
  ctx.textAlign = 'right';
  ctx.textBaseline = 'middle';
  for (const t of ticks) {
    ctx.strokeStyle = t.warn ? css('--warn') : css('--line');
    ctx.setLineDash(t.warn ? [4, 4] : []);
    ctx.beginPath(); ctx.moveTo(pad.l, y(t.v)); ctx.lineTo(w - pad.r, y(t.v)); ctx.stroke();
    ctx.fillStyle = css('--muted');
    ctx.fillText(fmtTick(t.v), pad.l - 5, y(t.v));
  }
  ctx.setLineDash([]);
  series.lines.forEach(([i, color], k) => {
    if (!rows.length) return;
    ctx.strokeStyle = css(color);
    ctx.lineWidth = 2;
    ctx.beginPath();
    rows.forEach((p, n) => { const px = x(p[0]), py = y(p[i]); n ? ctx.lineTo(px, py) : ctx.moveTo(px, py); });
    ctx.stroke();
    if (fills[k]) {
      ctx.lineTo(x(rows[rows.length - 1][0]), y(lo)); ctx.lineTo(x(rows[0][0]), y(lo)); ctx.closePath();
      ctx.globalAlpha = 0.14; ctx.fillStyle = css(color); ctx.fill(); ctx.globalAlpha = 1;
    }
  });
}

// ---------- state ----------
const inputs = new Inputs();
inputs.bindPedal($('padBrake'), 'pedal');
inputs.bindPedal($('padThrottle'), 'throttle');
let mujoco, manifest, sim, car, carId, roadName = 'bumps';
const cache = {};                 // per car: config, files, gltf scene
let bodyGroups = {}, carGroup = null, lastCar = null;
const trace = [], travel = [];
let gearWanted = 1;

async function loadCarAssets(id) {
  if (cache[id]) return cache[id];
  const base = `cars/${id}/`;
  const config = await (await fetch(base + 'config.json')).json();
  const [files, gltf] = await Promise.all([
    Promise.all(config.files.map(async (f) => [f, await fetchBytes(base + f)])).then(Object.fromEntries),
    new GLTFLoader().loadAsync(base + 'car.glb'),
  ]);
  cache[id] = { config, files, gltf };
  return cache[id];
}

function mountCarMeshes(gltf) {
  if (carGroup) scene.remove(carGroup);
  carGroup = new THREE.Group();
  bodyGroups = {};
  for (const node of [...gltf.scene.children]) {
    const body = node.name.split('|')[0];
    if (!bodyGroups[body]) { bodyGroups[body] = new THREE.Group(); carGroup.add(bodyGroups[body]); }
    node.traverse((o) => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });
    bodyGroups[body].add(node);
  }
  scene.add(carGroup);
}

function currentInput() {
  const springs = {};
  if (car?.suspension) {
    springs.front = { rate: +$('frontRate').value, damping: +$('frontDamping').value };
    springs.rear = { rate: +$('rearRate').value, damping: +$('rearDamping').value };
  }
  return { throttle: inputs.throttle, pedal: 0, lever: car?.lever ? inputs.lever : 0, steer: +$('steer').value / 100,
    driver: $('driver').checked, gear: gearWanted, springs, keepRideHeight: $('keepRide').checked, shockFriction: +$('shock').value };
}

function newSim() {
  if (sim) sim.dispose();
  const { config, files } = cache[carId];
  sim = new CarSim(mujoco, config, files, roadName);
  sim.reset(currentInput());
  drawBars(roadName === 'bumps' ? manifest.bars : []);
  trace.length = 0;
  travel.length = 0;
}

async function selectCar(id) {
  carId = id;
  document.querySelectorAll('#cars button').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.id === id)));
  status(`Loading the ${manifest.cars.find((c) => c.id === id).short}…`);
  const assets = await loadCarAssets(id);
  car = assets.config;
  mountCarMeshes(assets.gltf);
  fillPanel();
  gearWanted = 1;
  renderGears();
  newSim();
  const reach = Math.max(4.5, car.wheelbase * 1.9);
  lastCar = null;
  orbit.target.set(car.wheelbase / 2, 0, 0.6);
  camera.position.set(car.wheelbase / 2 - reach * 0.55, -reach, reach * 0.42);
  status('');
}

function fillPanel() {
  $('carName').textContent = car.name;
  const nb = (t) => t.replace(/ /g, '\u00a0');   // keep each figure with its unit
  const power = car.engine.rated_power_w >= 10000 ? `${fmt(car.engine.rated_power_w / 1000, 1)} kW` : `${Math.round(car.engine.rated_power_w)} W`;
  $('carMeta').textContent = [`${car.year}`, `${car.mass_kg} kg`, `${power} at ${car.engine.governed_rpm} rpm`,
    `wheelbase ${fmt(car.wheelbase, 2)} m`].map(nb).join(' · ');
  $('carControls').textContent = car.controls;
  $('pedalName').textContent = car.pedal ? car.pedal.label.replace('Combined pedal', 'Pedal') : 'Pedal';
  $('lgPedal').textContent = car.pedal ? car.pedal.label : 'Brake';
  $('pedalHint').textContent = car.pedal
    ? `${car.pedal.label} (S, ↓ or the on-screen pedal): ${car.pedal.detail.toLowerCase()}, ${car.pedal.devices.map(devText).join(' + ')}.`
    : '';
  $('leverLabel').textContent = car.lever ? car.lever.label : 'Brake lever';
  $('leverName').textContent = car.lever ? car.lever.label.replace('Hand ', '') : 'Lever';
  $('lgLever').textContent = car.lever ? car.lever.label : 'Lever';
  $('lever').disabled = !car.lever;
  $('barLever').classList.toggle('off', !car.lever);
  $('leverHint').textContent = car.lever ? `${car.lever.detail}, ${car.lever.devices.map(devText).join(' + ')}. Space toggles it.` : car.lever_note;
  $('steerKind').textContent = car.steering.box_ratio > 1 ? `${car.steering.kind}, ${car.steering.box_ratio}:1` : car.steering.kind;
  $('springs').hidden = !car.suspension;
  $('shockSet').hidden = !car.shocks;
  $('noSusp').hidden = !!car.suspension;
  for (const id of ['chart', 'frontGauge', 'rearGauge']) $(id).hidden = !car.suspension;
  if (car.suspension) {
    PRESETS.design = { front: [car.suspension.front.design_rate_kn, car.suspension.front.design_damping_kns],
      rear: [car.suspension.rear.design_rate_kn, car.suspension.rear.design_damping_kns], keep: true };
    applyPreset('design');
    $('springHint').textContent = `Rates at the axle. Design ${fmt(car.suspension.front.design_rate_kn, 0)} and ${fmt(car.suspension.rear.design_rate_kn, 0)} kN/m; roll stiffness follows each rate.`;
  }
  if (car.shocks) $('shock').value = car.shocks.design_nm;
  showValues();
}
const devText = (d) => d.kind === 'tendon' ? `${d.capacity} N·m at the wheels` : `${d.capacity} N·m per rear wheel`;

function renderGears() {
  const box = $('gears');
  box.replaceChildren();
  [{ label: 'Neutral' }, ...car.gears].forEach((g, i) => {
    const b = document.createElement('button');
    b.type = 'button';
    b.textContent = g.label;
    b.setAttribute('aria-pressed', String(i === gearWanted));
    b.addEventListener('click', () => setGear(i));
    box.append(b);
  });
}
function setGear(i) {
  gearWanted = Math.max(0, Math.min(car.gears.length, i));
  [...$('gears').children].forEach((b, k) => b.setAttribute('aria-pressed', String(k === gearWanted)));
  sim?.set({ gear: gearWanted });
}

const PRESETS = {
  soft: { scale: [0.36, 1], keep: true }, sag: { scale: [0.36, 1], keep: false },
  stiff: { scale: [3.6, 1], keep: true }, bouncy: { scale: [1, 0.13], keep: true },
};
function applyPreset(name) {
  const s = car.suspension, p = PRESETS[name];
  const val = (axle, i) => p.front ? p[axle][i] : s[axle][i ? 'design_damping_kns' : 'design_rate_kn'] * p.scale[i];
  $('frontRate').value = Math.round(val('front', 0)); $('rearRate').value = Math.round(val('rear', 0));
  $('frontDamping').value = val('front', 1).toFixed(2); $('rearDamping').value = val('rear', 1).toFixed(2);
  $('keepRide').checked = p.keep;
  onControl();
}
document.querySelectorAll('[data-preset]').forEach((b) => b.addEventListener('click', () => applyPreset(b.dataset.preset)));

function showValues() {
  $('throttleOut').textContent = pct(inputs.throttle);
  $('throttle').value = Math.round(inputs.throttle * 100);
  $('leverOut').textContent = car?.lever ? pct(inputs.lever) : 'none';
  $('lever').value = Math.round(inputs.lever * 100);
  $('steerSlider').textContent = $('driver').checked ? 'driver' : `${signed(+$('steer').value, 0)}%`;
  $('steer').disabled = $('driver').checked;
  $('frontRateOut').textContent = `${$('frontRate').value} kN/m`;
  $('rearRateOut').textContent = `${$('rearRate').value} kN/m`;
  $('frontDampingOut').textContent = `${(+$('frontDamping').value).toFixed(2)} kN·s/m`;
  $('rearDampingOut').textContent = `${(+$('rearDamping').value).toFixed(2)} kN·s/m`;
  $('shockOut').textContent = `${$('shock').value} N·m`;
}
function onControl() { showValues(); if (sim) sim.set(currentInput()); }
document.querySelectorAll('#controls input').forEach((el) => el.addEventListener('input', onControl));
$('throttle').addEventListener('input', () => { inputs.throttle = +$('throttle').value / 100; onControl(); });
$('lever').addEventListener('input', () => { inputs.lever = +$('lever').value / 100; onControl(); });
document.querySelectorAll('input[name="road"]').forEach((el) => el.addEventListener('change', () => { roadName = el.value; if (sim) newSim(); }));
$('restart').addEventListener('click', restart);
function restart() { if (!sim) return; sim.reset(currentInput()); sim.set({ gear: gearWanted }); trace.length = 0; travel.length = 0; }

function handleEvents(events) {
  for (const e of events) {
    if (e.type === 'gear') setGear(e.value);
    else if (e.type === 'gearStep') setGear(gearWanted + e.value);
    else if (e.type === 'driver') { $('driver').checked = !$('driver').checked; toast($('driver').checked ? 'Driver holds the line' : 'You steer'); onControl(); }
    else if (e.type === 'restart') restart();
    else if (e.type === 'lever') { if (!car.lever) { inputs.lever = 0; toast(car.lever_note); } onControl(); }
  }
}

// ---------- readouts ----------
function showReadout(r) {
  $('speed').textContent = fmt(r.speed * 3.6);
  $('distance').textContent = fmt(r.x);
  $('gearHud').textContent = r.gear ? car.gears[r.gear - 1].label : 'Neutral';
  $('rpm').textContent = Math.round(Math.max(0, r.rpm));
  $('seatRms').textContent = fmt(r.seatRmsG, 2);
  for (const w of ['front_left', 'front_right', 'rear_left', 'rear_right']) $(w).classList.toggle('down', r.wheels.has(w));
  $('wheelsText').textContent = `${r.wheels.size} of 4 on the road`;
  for (const axle of ['front', 'rear']) {
    const a = r.axles[axle];
    if (!a) continue;
    $(`${axle}Travel`).textContent = pct(a.used);
    $(`${axle}Offset`).textContent = `${signed(a.offsetMm)} mm`;
    $(`${axle}Bar`).style.width = `${Math.min(100, a.used * 100)}%`;
    $(`${axle}Gauge`).classList.toggle('on-stops', a.used >= 1);
  }
}
function showInputs(a, columnDeg) {
  const set = (id, v) => { $(id).querySelector('i').style.height = `${Math.round(v * 100)}%`; };
  set('barThrottle', a.throttle); set('barPedal', a.pedal); set('barLever', a.lever);
  $('outThrottle').textContent = pct(a.throttle); $('outPedal').textContent = pct(a.pedal); $('outLever').textContent = pct(a.lever);
  $('padBrake').querySelector('.fill').style.height = `${a.pedal * 100}%`;
  $('padThrottle').querySelector('.fill').style.height = `${Math.max(0, a.throttle - inputs.throttle) * 100}%`;
  const wheelDeg = columnDeg * car.steering.box_ratio;
  $('wheelRot').setAttribute('transform', `rotate(${-wheelDeg})`);
  $('steerOut').textContent = `${signed(wheelDeg, 0)}°`;
}

// ---------- main loop ----------
let lastFrame = 0, lastReadout = 0, simSeconds = 0, wallSeconds = 0;
function frame(now) {
  requestAnimationFrame(frame);
  const wall = Math.min(0.05, (now - (lastFrame || now)) / 1000);
  lastFrame = now;
  const a = inputs.update(wall);
  if (!sim) { renderer.render(scene, camera); return; }
  if (a.steering && $('driver').checked) { $('driver').checked = false; toast('You steer: driver off (T to hand back)'); }
  handleEvents(a.events);
  if (a.steering) $('steer').value = Math.round(a.steer * 100);
  const lever = car.lever ? a.lever : 0;
  sim.set({ throttle: a.throttle, pedal: car.pedal ? a.pedal : 0, lever, driver: $('driver').checked,
    steer: a.steering ? a.steer : +$('steer').value / 100 });
  const dt = sim.model.opt.timestep;
  const steps = Math.min(60, Math.round(wall / dt));
  sim.step(steps);
  simSeconds += steps * dt; wallSeconds += wall;
  const poses = sim.poses();
  for (const [name, p] of Object.entries(poses)) {
    const g = bodyGroups[name];
    if (g) { g.position.set(p[0], p[1], p[2]); g.quaternion.set(p[4], p[5], p[6], p[3]); }
  }
  const target = new THREE.Vector3(poses.chassis[0] + car.wheelbase / 2, poses.chassis[1], 0.6);
  if (lastCar) { const delta = target.clone().sub(lastCar); camera.position.add(delta); orbit.target.add(delta); }
  lastCar = target;
  sun.position.set(target.x - 4, target.y - 6, 9);
  sun.target.position.copy(target);
  orbit.update();
  renderer.render(scene, camera);
  const t = sim.data.time;
  trace.push([t, a.throttle, car.pedal ? a.pedal : 0, lever]);
  while (trace.length && t - trace[0][0] > 8.2) trace.shift();
  stripChart($('trace'), { rows: trace, lines: [[1, '--throttle'], [2, '--brake'], [3, '--lever']] },
    { lo: 0, hi: 1.08, ticks: [{ v: 0 }, { v: 1 }], fmtTick: (v) => (v ? '100%' : '0'), fills: [true, true, false] });
  if (car.suspension) {
    const fa = sim.axles.front.joints[0], ra = sim.axles.rear.joints[0], q = sim.data.qpos;
    travel.push([t, q[fa.qpos] / fa.range * 100 * sim.axles.front.sign, q[ra.qpos] / ra.range * 100 * sim.axles.rear.sign]);
    while (travel.length && t - travel[0][0] > 8.2) travel.shift();
    stripChart($('chart'), { rows: travel, lines: [[1, '--brass'], [2, '--green']] },
      { lo: -120, hi: 120, ticks: [{ v: -100, warn: true }, { v: 0 }, { v: 100, warn: true }], fmtTick: (v) => (v ? `${v > 0 ? '+' : MINUS}100%` : '0') });
  }
  const r = now - lastReadout > 100 ? sim.readout() : null;
  showInputs({ throttle: a.throttle, pedal: car.pedal ? a.pedal : 0, lever }, r ? r.columnDeg : lastColumn);
  if (r) {
    lastReadout = now;
    lastColumn = r.columnDeg;
    showReadout(r);
    showValues();
    if (wallSeconds > 1) $('rate').textContent = `${fmt(simSeconds / wallSeconds, 2)}× real time`;
    if (wallSeconds > 3) { simSeconds = 0; wallSeconds = 0; }
  }
}
let lastColumn = 0;

// ---------- start ----------
async function start() {
  resize();
  paintScene();
  matchMedia('(prefers-color-scheme: dark)').addEventListener('change', paintScene);
  new MutationObserver(paintScene).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });
  $('keys').replaceChildren(...KEYMAP.flatMap(([k, v]) => { const dt = document.createElement('dt'), dd = document.createElement('dd'); dt.textContent = k; dd.textContent = v; return [dt, dd]; }));
  requestAnimationFrame(frame);
  try {
    if (typeof WebAssembly !== 'object') throw new Error('This browser has no WebAssembly support.');
    manifest = await (await fetch('manifest.json')).json();
    $('mjv').textContent = manifest.mujoco_version;
    const nav = $('cars');
    for (const c of manifest.cars) {
      const b = document.createElement('button');
      b.type = 'button';
      b.dataset.id = c.id;
      b.setAttribute('aria-pressed', 'false');
      b.innerHTML = `<span></span><small></small>`;
      b.querySelector('span').textContent = c.short.replace(/ \d{4}$/, '');
      b.querySelector('small').textContent = String(c.year);
      b.addEventListener('click', () => { if (mujoco && c.id !== carId) selectCar(c.id).catch(fail); });
      nav.append(b);
    }
    const wasm = await fetchBytes('mujoco.wasm', (f) => status(`Loading the MuJoCo physics engine… ${Math.round(f * 100)}%`));
    status('Starting MuJoCo…');
    const { default: loadMujoco } = await import(`https://cdn.jsdelivr.net/npm/@mujoco/mujoco@${manifest.mujoco_version}/mujoco.js`);
    mujoco = await loadMujoco({ wasmBinary: wasm.buffer });
    const first = manifest.cars.find((c) => c.id === 'model_t_1909') || manifest.cars[0];
    await selectCar(first.id);
  } catch (err) { fail(err); }
}
function fail(err) {
  console.error(err);
  const blocked = /WebAssembly|wasm|CompileError|Content Security/i.test(String(err));
  status(blocked ? `The physics engine could not start here: ${err.message}. Run it locally with the command under "About this simulation".`
    : `Could not start the simulation: ${err.message}`, true);
}
start();
