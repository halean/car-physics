// Occasional cows along the road: scenery only (not in the physics model; they never touch the
// car). A cow standing on the road ambles off to the verge when a car comes near, and hurries if
// the car is fast. Positions are seeded, so a restart brings the same herd back.
// THREE comes from the page's module (build.py inlines this file next to the page script).

function mulberry32(seed) {
  return () => {
    seed |= 0; seed = (seed + 0x6d2b79f5) | 0;
    let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function hideTexture(rng, base, spot) {
  const c = document.createElement('canvas');
  c.width = 256; c.height = 128;
  const g = c.getContext('2d');
  g.fillStyle = base; g.fillRect(0, 0, 256, 128);
  g.fillStyle = spot;
  for (let i = 0; i < 7; i++) {
    g.beginPath();
    const x = rng() * 256, y = rng() * 128, r = 12 + rng() * 26;
    for (let k = 0; k <= 10; k++) {
      const a = k / 10 * Math.PI * 2, rr = r * (0.7 + 0.5 * rng());
      k ? g.lineTo(x + Math.cos(a) * rr, y + Math.sin(a) * rr * 0.8) : g.moveTo(x + rr, y);
    }
    g.fill();
  }
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  return tex;
}

// A cow about 2.1 m long and 1.4 m at the withers, facing +x, feet at z = 0.
function makeCow(rng) {
  const cow = new THREE.Group();
  const brown = rng() < 0.3;
  const hide = new THREE.MeshStandardMaterial({ map: hideTexture(rng, brown ? '#8a5a36' : '#f2efe6', brown ? '#f2efe6' : '#1e1c1a'), roughness: 0.9 });
  const dark = new THREE.MeshStandardMaterial({ color: 0x2a2522, roughness: 0.9 });
  const pink = new THREE.MeshStandardMaterial({ color: 0xd99a92, roughness: 0.8 });
  const horn = new THREE.MeshStandardMaterial({ color: 0xe8dcc0, roughness: 0.6 });
  const mesh = (geo, mat, x, y, z, parent = cow) => { const m = new THREE.Mesh(geo, mat); m.position.set(x, y, z); m.castShadow = true; parent.add(m); return m; };
  // Body: one rounded barrel along x (a capsule is built along y, so turn it).
  mesh(new THREE.CapsuleGeometry(0.4, 1.15, 6, 14).rotateZ(Math.PI / 2), hide, 0, 0, 1.06).scale.set(1, 0.92, 1.05);
  mesh(new THREE.SphereGeometry(0.2, 10, 6), pink, -0.2, 0, 0.62);                       // udder
  const legs = [];
  for (const [x, y] of [[0.62, 0.24], [0.62, -0.24], [-0.62, 0.24], [-0.62, -0.24]]) {
    const pivot = new THREE.Group();
    pivot.position.set(x, y, 0.72);
    cow.add(pivot);
    mesh(new THREE.CylinderGeometry(0.075, 0.065, 0.66, 8).rotateX(Math.PI / 2), hide, 0, 0, -0.33, pivot);
    mesh(new THREE.CylinderGeometry(0.075, 0.08, 0.08, 8).rotateX(Math.PI / 2), dark, 0, 0, -0.68, pivot);
    legs.push(pivot);
  }
  const neck = new THREE.Group();
  neck.position.set(0.95, 0, 1.2);
  cow.add(neck);
  mesh(new THREE.BoxGeometry(0.45, 0.34, 0.36), hide, 0.25, 0, 0.02, neck);
  mesh(new THREE.BoxGeometry(0.2, 0.3, 0.24), pink, 0.52, 0, -0.06, neck);             // muzzle
  for (const s of [1, -1]) {
    mesh(new THREE.ConeGeometry(0.035, 0.18, 6).rotateX(-s * Math.PI / 2.4), horn, 0.18, s * 0.19, 0.22, neck);
    mesh(new THREE.BoxGeometry(0.06, 0.16, 0.1), hide, 0.12, s * 0.24, 0.1, neck);     // ears
  }
  // Tail hangs from the rump: a cylinder is built along y, so stand it on z; pivot at the top.
  const tail = new THREE.Group();
  tail.position.set(-0.98, 0, 1.28);
  cow.add(tail);
  mesh(new THREE.CylinderGeometry(0.022, 0.018, 0.62, 6).rotateX(Math.PI / 2), hide, 0, 0, -0.31, tail);
  mesh(new THREE.SphereGeometry(0.05, 8, 6), dark, 0, 0, -0.64, tail).scale.set(1, 1, 1.8);   // switch
  tail.rotation.y = -0.12;
  return { cow, legs, neck, tail };
}

export class Herd {
  // parent: the group the road lives in (x along the road, y across it, z up)
  constructor(parent, { seed = 7, from = 40, to = 2000, gap = [45, 110], roadHalf = 1.6 } = {}) {
    this.parent = parent;
    this.opts = { seed, from, to, gap, roadHalf };
    this.cows = [];
    this.reset();
  }

  reset() {
    for (const c of this.cows) this.parent.remove(c.obj.cow);
    this.cows = [];
    const { seed, from, to, gap, roadHalf } = this.opts;
    const rng = mulberry32(seed);
    for (let x = from + rng() * gap[0]; x < to; x += gap[0] + rng() * (gap[1] - gap[0])) {
      const onRoad = rng() < 0.6;
      const side = rng() < 0.5 ? -1 : 1;
      const y = onRoad ? (rng() * 2 - 1) * (roadHalf - 0.5) : side * (roadHalf + 2 + rng() * 6);
      const obj = makeCow(rng);
      const heading = rng() * Math.PI * 2;
      obj.cow.position.set(x, y, 0);
      obj.cow.rotation.z = heading;
      obj.cow.scale.setScalar(0.92 + rng() * 0.16);
      this.parent.add(obj.cow);
      this.cows.push({ obj, x, y, heading, phase: rng() * 10, state: 'graze', target: null, speed: 0, hurried: false });
    }
  }

  clear() {
    for (const c of this.cows) this.parent.remove(c.obj.cow);
    this.cows = [];
  }

  // A cow that walks into the road for the reaction test: it stays until the car is nearly on it.
  spawn(x, y) {
    const obj = makeCow(mulberry32((Math.random() * 1e9) | 0));
    this.parent.add(obj.cow);
    const c = { obj, x, y, heading: Math.PI / 2, phase: 0, state: 'graze', target: null, speed: 0, hurried: false, stubborn: true };
    this.cows.push(c);
    return c;
  }

  onRoad(c) { return Math.abs(c.y) < this.opts.roadHalf + 0.6; }

  // carX: front of the car along the road; carSpeed in m/s. Returns the nearest cow still on the
  // road ahead (or null) and whether any cow had to hurry this frame.
  update(dt, carX, carSpeed) {
    let nearest = null, hurried = false;
    for (const c of this.cows) {
      const ahead = c.x - carX;
      const warn = c.stubborn ? 2 + 0.3 * carSpeed : 6 + 1.6 * carSpeed;   // most notice a car about 1.6 s away
      if (c.state === 'graze' && this.onRoad(c) && ahead > -2 && ahead < warn) {
        const side = c.y === 0 ? 1 : Math.sign(c.y);
        c.target = side * (this.opts.roadHalf + 2.5);
        c.state = 'walk';
        c.speed = c.stubborn || ahead < 3 + 0.6 * carSpeed ? 3.2 : 1.3;   // trots if the car is almost on it
        c.hurried = c.speed > 2;
        if (c.hurried) hurried = true;
        c.heading = side > 0 ? Math.PI / 2 : -Math.PI / 2;
      }
      c.phase += dt;
      const { cow, legs, neck, tail } = c.obj;
      if (c.state === 'walk') {
        const step = Math.sign(c.target - c.y) * c.speed * dt;
        if (Math.abs(c.target - c.y) <= Math.abs(step)) { c.y = c.target; c.state = 'graze'; }
        else c.y += step;
        const swing = Math.sin(c.phase * c.speed * 5) * 0.45;
        legs.forEach((l, i) => { l.rotation.y = (i % 3 === 0 ? swing : -swing); });
        neck.rotation.y = -0.1;
      } else {
        legs.forEach((l) => { l.rotation.y = 0; });
        neck.rotation.y = 0.55 + 0.08 * Math.sin(c.phase * 2.2);   // head down, grazing
      }
      tail.rotation.x = 0.18 * Math.sin(c.phase * 3);   // swishes side to side
      cow.position.set(c.x, c.y, 0);
      cow.rotation.z += ((c.heading - cow.rotation.z + Math.PI * 3) % (Math.PI * 2) - Math.PI) * Math.min(1, dt * 4);
      if (this.onRoad(c) && c.state === 'graze' && c.x > carX - 1 && (!nearest || c.x < nearest.x)) nearest = c;
    }
    return { nearest, hurried };
  }
}
