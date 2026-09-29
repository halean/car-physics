// Shared simulation core for every car in the web simulator: no DOM, runs in the browser and Node.
// It mirrors the cars' validated control.py exactly: governed engine (full torque below governed
// speed, droop to no-load), clutch/belt engagement ramp, brakes as joint or tendon friction at the
// capacities in each car's engine.json, a position servo at the steering column. The config comes
// from build.py (cars.py + each car's own files).

export class CarSim {
  constructor(mujoco, config, files, road) {
    this.mj = mujoco;
    this.cfg = config;
    this.vfs = new mujoco.MjVFS();
    for (const [name, bytes] of Object.entries(files)) this.vfs.addBuffer(name, bytes);
    this.model = mujoco.MjModel.mj_loadXML(`model_${road}.xml`, this.vfs);
    this.data = new mujoco.MjData(this.model);
    const m = this.model;
    const id = (type, name) => {
      const i = mujoco.mj_name2id(m, mujoco.mjtObj[type].value, name);
      if (i < 0) throw new Error(`${name} is not in the ${config.short} model`);
      return i;
    };
    this.chassis = id('mjOBJ_BODY', 'chassis');
    this.bodies = config.bodies.map((name) => ({ name, id: id('mjOBJ_BODY', name) }));
    this.drives = config.gears.map((g) => id('mjOBJ_ACTUATOR', `drive_${g.name}`));
    this.column = id('mjOBJ_ACTUATOR', 'column');
    this.diff = id('mjOBJ_TENDON', 'differential');
    this.rearDofs = ['rear_left_joint', 'rear_right_joint'].map((n) => m.jnt_dofadr[id('mjOBJ_JOINT', n)]);
    this.columnJoint = id('mjOBJ_JOINT', 'column_joint');
    const joint = (name) => {
      const j = id('mjOBJ_JOINT', name);
      return { id: j, dof: m.jnt_dofadr[j], qpos: m.jnt_qposadr[j], range: m.jnt_range[2 * j + 1],
        k0: m.jnt_stiffness[j], c0: m.dof_damping[m.jnt_dofadr[j]], ref0: m.qpos_spring[m.jnt_qposadr[j]],
        f0: m.dof_frictionloss[m.jnt_dofadr[j]] };
    };
    this.axles = {};
    for (const [axle, a] of Object.entries(config.suspension || {})) this.axles[axle] = { ...a, joints: a.joints.map(joint) };
    this.shocks = config.shocks ? config.shocks.joints.map(joint) : [];
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

  static defaults(config) {
    const springs = {};
    for (const [axle, a] of Object.entries(config.suspension || {})) springs[axle] = { rate: a.design_rate_kn, damping: a.design_damping_kns };
    return { throttle: 1, pedal: 0, lever: 0, steer: 0, driver: true, gear: 1,
      springs, keepRideHeight: true, shockFriction: config.shocks ? config.shocks.design_nm : 0 };
  }

  reset(input) {
    this.mj.mj_resetData(this.model, this.data);
    this.mj.mj_forward(this.model, this.data);
    this.in = input ? { ...input } : CarSim.defaults(this.cfg);
    this.gear = 0;
    this.since = 0;
    this.seat = [];
    this.peak = {};
    this.applyChassis();
  }

  set(input) {
    Object.assign(this.in, input);
    this.applyChassis();
  }

  // Springs, dampers and shocks: joint parameters MuJoCo reads at every step.
  applyChassis() {
    const m = this.model, k = m.jnt_stiffness, c = m.dof_damping, ref = m.qpos_spring, fl = m.dof_frictionloss;
    for (const [axle, a] of Object.entries(this.axles)) {
      const s = this.in.springs[axle];
      const sk = s.rate / a.design_rate_kn, sc = s.damping / a.design_damping_kns;
      for (const j of a.joints) {
        const kj = j.k0 * sk;
        k[j.id] = kj;
        c[j.dof] = j.c0 * sc;
        // Same preload torque keeps ride height; otherwise the spring keeps its free shape and sags.
        ref[j.qpos] = this.in.keepRideHeight ? j.ref0 * j.k0 / kj : j.ref0;
      }
    }
    for (const j of this.shocks) fl[j.dof] = this.in.shockFriction;
  }

  engineTorque(speed) {
    if (speed <= this.governed) return this.maxTorque;
    return this.maxTorque * Math.max(0, (this.noLoad - speed) / (this.noLoad - this.governed));
  }

  pursuit() {
    // Pure pursuit to a point on the starting line ~0.8 s ahead (as control.Steer in 009/010).
    const d = this.data, x = d.xmat, b = 9 * this.chassis, qv = d.qvel;
    const yaw = Math.atan2(x[b + 3], x[b]);
    const look = Math.max(4, 0.8 * Math.hypot(qv[0], qv[1]));
    const error = d.qpos[1] + look * Math.sin(yaw);
    return -Math.atan(2 * this.cfg.wheelbase * error / (look * look));
  }

  control() {
    const m = this.model, d = this.data, cfg = this.cfg, inp = this.in;
    const gear = Math.round(inp.gear);
    if (gear !== this.gear) { this.gear = gear; this.since = d.time; }
    const declutched = cfg.declutch_on_pedal && inp.pedal > 0;
    const ctrl = d.ctrl, vel = d.actuator_velocity;
    this.drives.forEach((act, i) => {
      ctrl[act] = 0;
      if (gear === i + 1 && !declutched) {
        const engage = Math.min(1, Math.max(0, (d.time - this.since) / cfg.engine.shift_seconds));
        ctrl[act] = engage * inp.throttle * this.engineTorque(vel[act]);
      }
    });
    let tendon = 0, joints = 0;
    for (const [control, value] of [[cfg.pedal, inp.pedal], [cfg.lever, inp.lever]]) {
      if (!control) continue;
      for (const dev of control.devices) {
        if (dev.kind === 'tendon') tendon += value * dev.capacity;
        else joints += value * dev.capacity;
      }
    }
    m.tendon_frictionloss[this.diff] = tendon;
    const fl = m.dof_frictionloss;
    for (const dof of this.rearDofs) fl[dof] = joints;
    ctrl[this.column] = inp.driver ? this.pursuit() : inp.steer * cfg.steering.limit_deg * Math.PI / 180;
  }

  seatVz() {
    const d = this.data, b = 9 * this.chassis, R = d.xmat, qv = d.qvel, bp = this.model.body_pos, s = this.cfg.seat;
    const r = [0, 1, 2].map((i) => s[i] - bp[3 * this.chassis + i]);
    const w = [0, 1, 2].map((i) => R[b + 3 * i] * qv[3] + R[b + 3 * i + 1] * qv[4] + R[b + 3 * i + 2] * qv[5]);
    const rw = [0, 1, 2].map((i) => R[b + 3 * i] * r[0] + R[b + 3 * i + 1] * r[1] + R[b + 3 * i + 2] * r[2]);
    return qv[2] + (w[0] * rw[1] - w[1] * rw[0]);
  }

  step(n = 1) {
    const d = this.data;
    for (let i = 0; i < n; i++) {
      this.control();
      this.mj.mj_step(this.model, d);
      this.seat.push([d.time, this.seatVz()]);
      if (this.seat.length > 2000) this.seat.shift();
      for (const [axle, a] of Object.entries(this.axles)) {
        const j = a.joints[0];
        this.peak[axle] = Math.max(this.peak[axle] || 0, Math.abs(d.qpos[j.qpos]) / j.range);
      }
    }
  }

  wheelsOnGround() {
    const d = this.data, found = new Set(), vec = d.contact;
    try {
      for (let i = 0; i < d.ncon; i++) {
        const c = vec.get(i), a = c.geom1, b = c.geom2;
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
    for (let i = 1; i < v.length; i++) acc2 += ((v[i][1] - v[i - 1][1]) / (v[i][0] - v[i - 1][0])) ** 2;
    return Math.sqrt(acc2 / (v.length - 1)) / 9.81;
  }

  engineRpm() {
    const g = this.gear;
    return g > 0 ? this.data.actuator_velocity[this.drives[g - 1]] * 60 / (2 * Math.PI) : 0;
  }

  readout() {
    const d = this.data, out = {
      time: d.time, x: d.qpos[0], speed: Math.hypot(d.qvel[0], d.qvel[1]), gear: this.gear,
      rpm: this.engineRpm(), columnDeg: d.qpos[this.model.jnt_qposadr[this.columnJoint]] * 180 / Math.PI,
      seatRmsG: this.seatRms(), wheels: this.wheelsOnGround(), axles: {},
    };
    for (const [axle, a] of Object.entries(this.axles)) {
      const j = a.joints[0], q = d.qpos[j.qpos];
      out.axles[axle] = { offsetMm: q * a.lever_m * a.sign * 1000, used: Math.abs(q) / j.range, peak: this.peak[axle] || 0 };
    }
    return out;
  }

  poses() {
    const d = this.data, p = d.xpos, q = d.xquat, out = {};
    for (const { name, id } of this.bodies) {
      out[name] = [p[3 * id], p[3 * id + 1], p[3 * id + 2], q[4 * id], q[4 * id + 1], q[4 * id + 2], q[4 * id + 3]];
    }
    return out;
  }

  dispose() {
    this.data.delete();
    this.model.delete();
    this.vfs.delete();
  }
}
