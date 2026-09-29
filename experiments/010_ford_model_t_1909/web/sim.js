// Model T simulation core for the browser and Node: no DOM. Mirrors control.py (engine,
// gears, brakes, pure-pursuit driver) and interactive.py (live spring settings) exactly.

export const DESIGN = {
  gear: 0, throttle: 1, footBrake: 0, handBrake: 0, driver: 1, steerDeg: 0,
  frontRate: 28, rearRate: 26, frontDamping: 1.56, rearDamping: 1.55, keepRideHeight: 1,
};

export class ModelTSim {
  // mujoco: the loaded module; files: {name: Uint8Array} including the XML; config: config.json
  constructor(mujoco, xmlName, files, config) {
    this.mj = mujoco;
    this.cfg = config;
    this.vfs = new mujoco.MjVFS();
    for (const [name, bytes] of Object.entries(files)) this.vfs.addBuffer(name, bytes);
    this.model = mujoco.MjModel.mj_loadXML(xmlName, this.vfs);
    this.data = new mujoco.MjData(this.model);
    const id = (type, name) => {
      const i = mujoco.mj_name2id(this.model, mujoco.mjtObj[type].value, name);
      if (i < 0) throw new Error(`${name} not in model`);
      return i;
    };
    this.id = id;
    const m = this.model;
    this.chassis = id('mjOBJ_BODY', 'chassis');
    this.bodies = config.bodies.map((name) => ({ name, id: id('mjOBJ_BODY', name) }));
    this.joints = {};
    for (const axle of ['front', 'rear']) {
      for (const part of ['swing', 'roll']) {
        const j = id('mjOBJ_JOINT', `${axle}_axle_${part}`);
        this.joints[`${axle}_${part}`] = {
          id: j, dof: m.jnt_dofadr[j], qpos: m.jnt_qposadr[j], range: m.jnt_range[2 * j + 1],
          k0: m.jnt_stiffness[j], c0: m.dof_damping[m.jnt_dofadr[j]], ref0: m.qpos_spring[m.jnt_qposadr[j]],
        };
      }
    }
    this.drives = Object.keys(config.engine.gearbox_ratios).map((g) => id('mjOBJ_ACTUATOR', `drive_${g}`));
    this.column = id('mjOBJ_ACTUATOR', 'column');
    this.diff = id('mjOBJ_TENDON', 'differential');
    this.rearDofs = ['rear_left_joint', 'rear_right_joint'].map((n) => m.jnt_dofadr[id('mjOBJ_JOINT', n)]);
    // Road and tyre geoms, for "wheels on ground".
    this.road = new Set();
    this.tyre = new Map();
    for (let g = 0; g < m.ngeom; g++) {
      const name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM.value, g) || '';
      if (name === 'slope' || name.startsWith('road_')) this.road.add(g);
      if (name.endsWith('_rolling')) this.tyre.set(g, name.replace('_rolling', ''));
    }
    const e = config.engine;
    this.governed = e.governed_rpm * 2 * Math.PI / 60;
    this.noLoad = e.no_load_rpm * 2 * Math.PI / 60;
    this.maxTorque = e.rated_power_w / this.governed;
    this.reset();
  }

  reset() {
    this.mj.mj_resetData(this.model, this.data);
    this.mj.mj_forward(this.model, this.data);
    this.ui = { ...DESIGN };
    this.gear = 0;
    this.since = 0;
    this.seat = [];          // [time, seat vertical velocity], last 2 s
    this.peak = { front: 0, rear: 0 };
    this.applySprings();
  }

  set(values) {
    Object.assign(this.ui, values);
    this.applySprings();
  }

  engineTorque(speed) {
    if (speed <= this.governed) return this.maxTorque;
    return this.maxTorque * Math.max(0, (this.noLoad - speed) / (this.noLoad - this.governed));
  }

  applySprings() {
    const m = this.model;
    const stiffness = m.jnt_stiffness, damping = m.dof_damping, spring = m.qpos_spring;
    for (const axle of ['front', 'rear']) {
      const sk = this.ui[`${axle}Rate`] / DESIGN[`${axle}Rate`];
      const sc = this.ui[`${axle}Damping`] / DESIGN[`${axle}Damping`];
      for (const part of ['swing', 'roll']) {
        const j = this.joints[`${axle}_${part}`];
        const k = j.k0 * sk;
        stiffness[j.id] = k;
        damping[j.dof] = j.c0 * sc;
        // Same preload torque (ride height kept) or the same free shape (a softer spring sags).
        spring[j.qpos] = this.ui.keepRideHeight ? j.ref0 * j.k0 / k : j.ref0;
      }
    }
  }

  pursuit() {
    // Pure pursuit to a point on the starting line ~0.8 s ahead (control.Steer, hold_heading).
    const d = this.data, x = d.xmat, b = 9 * this.chassis, qv = d.qvel;
    const yaw = Math.atan2(x[b + 3], x[b]);
    const look = Math.max(4, this.cfg.preview_s * Math.hypot(qv[0], qv[1]));
    const error = d.qpos[1] + look * Math.sin(yaw);
    return -Math.atan(2 * this.cfg.wheelbase * error / (look * look));
  }

  control() {
    const m = this.model, d = this.data, e = this.cfg.engine;
    const gear = Math.round(this.ui.gear);
    if (gear !== this.gear) { this.gear = gear; this.since = d.time; }
    const ctrl = d.ctrl, vel = d.actuator_velocity;
    this.drives.forEach((act, i) => {
      ctrl[act] = 0;
      if (gear === i + 1) {
        const engage = Math.min(1, (d.time - this.since) / e.clutch_seconds);
        ctrl[act] = engage * this.ui.throttle * this.engineTorque(vel[act]);
      }
    });
    m.tendon_frictionloss[this.diff] = this.ui.footBrake * e.foot_brake_capacity_nm_at_wheels;
    const fl = m.dof_frictionloss;
    for (const dof of this.rearDofs) fl[dof] = this.ui.handBrake * e.rear_drum_capacity_nm_each;
    ctrl[this.column] = this.ui.driver ? this.pursuit() : this.ui.steerDeg * Math.PI / 180;
  }

  seatVz() {
    const d = this.data, b = 9 * this.chassis, R = d.xmat, qv = d.qvel;
    const bp = this.model.body_pos, s = this.cfg.seat;
    const r = [s[0] - bp[3 * this.chassis], s[1] - bp[3 * this.chassis + 1], s[2] - bp[3 * this.chassis + 2]];
    const wl = [qv[3], qv[4], qv[5]];   // free-joint angular velocity, body frame
    const w = [0, 1, 2].map((i) => R[b + 3 * i] * wl[0] + R[b + 3 * i + 1] * wl[1] + R[b + 3 * i + 2] * wl[2]);
    const rw = [0, 1, 2].map((i) => R[b + 3 * i] * r[0] + R[b + 3 * i + 1] * r[1] + R[b + 3 * i + 2] * r[2]);
    return qv[2] + (w[0] * rw[1] - w[1] * rw[0]);
  }

  step(n = 1) {
    for (let i = 0; i < n; i++) {
      this.control();
      this.mj.mj_step(this.model, this.data);
      const d = this.data;
      this.seat.push([d.time, this.seatVz()]);
      if (this.seat.length > 2000) this.seat.shift();
      for (const axle of ['front', 'rear']) {
        const j = this.joints[`${axle}_swing`];
        this.peak[axle] = Math.max(this.peak[axle], Math.abs(d.qpos[j.qpos]) / j.range);
      }
    }
  }

  wheelsOnGround() {
    const d = this.data, found = new Set(), vec = d.contact;
    try {
      for (let i = 0; i < d.ncon; i++) {
        const c = vec.get(i);
        const [a, b] = [c.geom1, c.geom2];
        if (this.road.has(a) && this.tyre.has(b)) found.add(this.tyre.get(b));
        if (this.road.has(b) && this.tyre.has(a)) found.add(this.tyre.get(a));
        c.delete?.();
      }
    } finally {
      vec.delete();
    }
    return found;
  }

  seatRms() {
    const s = this.seat, n = 20;
    if (s.length < 3 * n) return 0;
    const v = [];
    for (let i = n - 1; i < s.length; i++) {
      let sum = 0;
      for (let k = i - n + 1; k <= i; k++) sum += s[k][1];
      v.push([s[i][0], sum / n]);
    }
    let acc2 = 0;
    for (let i = 1; i < v.length; i++) {
      const a = (v[i][1] - v[i - 1][1]) / (v[i][0] - v[i - 1][0]);
      acc2 += a * a;
    }
    return Math.sqrt(acc2 / (v.length - 1)) / 9.81;
  }

  readout() {
    const d = this.data, out = {
      time: d.time, x: d.qpos[0], speed: Math.hypot(d.qvel[0], d.qvel[1]), gear: this.gear,
      seatRmsG: this.seatRms(), wheels: this.wheelsOnGround(), axles: {},
    };
    for (const axle of ['front', 'rear']) {
      const j = this.joints[`${axle}_swing`];
      const q = d.qpos[j.qpos];
      // Axle travel relative to the body, up (compressed) positive: a positive swing lowers
      // the front axle (ahead of its ball) and raises the rear one (behind its ball).
      out.axles[axle] = {
        offsetMm: q * this.cfg.levers[axle] * (axle === 'rear' ? 1 : -1) * 1000,
        used: Math.abs(q) / j.range, peak: this.peak[axle],
      };
    }
    return out;
  }

  // Body poses for rendering: {name: [x, y, z, qw, qx, qy, qz]}
  poses() {
    const d = this.data, p = d.xpos, q = d.xquat, out = {};
    for (const { name, id } of this.bodies) {
      out[name] = [p[3 * id], p[3 * id + 1], p[3 * id + 2], q[4 * id], q[4 * id + 1], q[4 * id + 2], q[4 * id + 3]];
    }
    return out;
  }
}
