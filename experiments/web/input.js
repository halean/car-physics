// One input layer for keyboard, gamepad and on-screen pedals (mouse or touch), producing the
// shared driving actions: hand throttle (a lever: stays where set), brake pedal (momentary),
// brake lever (latches), steering (-1..1, self-centring) and discrete events (gears, restart).
// Keys ramp so a digital key still gives a progressive, analog-like input; gamepad axes and
// triggers get a small deadzone; on-screen pedals read pressure from where the pointer is.

export const KEYMAP = [
  ['W / ↑', 'open the hand throttle'], ['Q', 'close the hand throttle'],
  ['S / ↓', 'brake pedal (hold)'], ['Space', 'brake lever on / off'],
  ['A D / ← →', 'steer (takes over from the driver)'], ['0–4', 'neutral, gears'],
  ['E / C', 'gear up / down'], ['T', 'driver holds the line'], ['R', 'restart from the line'],
];

const DEADZONE = 0.12;
const RATE = { throttleOpen: 0.8, throttleClose: 1.2, pedalOn: 3.0, pedalOff: 5.0, steerOn: 1.4, steerCentre: 2.8 };
const dz = (v) => (Math.abs(v) < DEADZONE ? 0 : (v - Math.sign(v) * DEADZONE) / (1 - DEADZONE));
const clamp = (v, lo = 0, hi = 1) => Math.min(hi, Math.max(lo, v));

export class Inputs {
  constructor() {
    this.keys = new Set();
    this.throttle = 1;        // the hand throttle lever
    this.keyPedal = 0;
    this.lever = 0;
    this.keySteer = 0;
    this.pointer = { pedal: 0, throttle: 0 };
    this.events = [];
    this.padPrev = [];
    this.source = 'keyboard';
    const typing = (e) => e.target.closest?.('input, select, textarea, button') && !e.target.matches('input[type="range"]');
    addEventListener('keydown', (e) => {
      if (typing(e) || e.metaKey || e.ctrlKey || e.altKey) return;
      const k = e.key.length === 1 ? e.key.toLowerCase() : e.key;
      const handled = this.onKey(k, e.repeat);
      if (handled) { e.preventDefault(); this.source = 'keyboard'; }
    });
    addEventListener('keyup', (e) => this.keys.delete(e.key.length === 1 ? e.key.toLowerCase() : e.key));
    addEventListener('blur', () => this.keys.clear());
  }

  onKey(k, repeat) {
    const held = ['w', 'ArrowUp', 'q', 's', 'ArrowDown', 'a', 'd', 'ArrowLeft', 'ArrowRight'];
    if (held.includes(k)) { this.keys.add(k); return true; }
    if (repeat) return ['0', '1', '2', '3', '4', ' ', 'e', 'c', 't', 'r'].includes(k);
    if (k >= '0' && k <= '4') { this.events.push({ type: 'gear', value: +k }); return true; }
    if (k === 'e') { this.events.push({ type: 'gearStep', value: 1 }); return true; }
    if (k === 'c') { this.events.push({ type: 'gearStep', value: -1 }); return true; }
    if (k === ' ') { this.lever = this.lever > 0 ? 0 : 1; this.events.push({ type: 'lever' }); return true; }
    if (k === 't') { this.events.push({ type: 'driver' }); return true; }
    if (k === 'r') { this.events.push({ type: 'restart' }); return true; }
    return false;
  }

  // An on-screen pedal: pressure from the pointer's height on the pedal (top = full).
  bindPedal(el, which) {
    const read = (e) => {
      const r = el.getBoundingClientRect();
      this.pointer[which] = clamp(1 - (e.clientY - r.top) / r.height, 0.15, 1);
      this.source = e.pointerType === 'touch' ? 'touch' : 'mouse';
    };
    el.addEventListener('pointerdown', (e) => { el.setPointerCapture(e.pointerId); read(e); el.classList.add('pressed'); });
    el.addEventListener('pointermove', (e) => { if (el.hasPointerCapture(e.pointerId)) read(e); });
    const release = () => { this.pointer[which] = 0; el.classList.remove('pressed'); };
    el.addEventListener('pointerup', release);
    el.addEventListener('pointercancel', release);
  }

  gamepad() {
    const pads = navigator.getGamepads ? [...navigator.getGamepads()].filter(Boolean) : [];
    const pad = pads.find((p) => p.mapping === 'standard') || pads[0];
    if (!pad) return null;
    const b = (i) => pad.buttons[i] || { value: 0, pressed: false };
    const edge = (i) => { const now = b(i).pressed, was = this.padPrev[i]; this.padPrev[i] = now; return now && !was; };
    if (edge(4)) this.events.push({ type: 'gearStep', value: -1 });
    if (edge(5)) this.events.push({ type: 'gearStep', value: 1 });
    if (edge(0)) { this.lever = this.lever > 0 ? 0 : 1; this.events.push({ type: 'lever' }); }
    if (edge(3)) this.events.push({ type: 'driver' });
    if (edge(9)) this.events.push({ type: 'restart' });
    const out = { throttle: dz(b(7).value), pedal: dz(b(6).value), steer: dz(pad.axes[0] || 0), id: pad.id };
    if (out.throttle || out.pedal || out.steer) this.source = 'gamepad';
    return out;
  }

  // Advance ramps by dt seconds; returns the actions for this frame.
  update(dt) {
    const k = this.keys, has = (...names) => names.some((n) => k.has(n));
    if (has('w', 'ArrowUp')) this.throttle = clamp(this.throttle + RATE.throttleOpen * dt);
    if (has('q')) this.throttle = clamp(this.throttle - RATE.throttleClose * dt);
    this.keyPedal = has('s', 'ArrowDown') ? clamp(this.keyPedal + RATE.pedalOn * dt) : clamp(this.keyPedal - RATE.pedalOff * dt);
    const want = (has('a', 'ArrowLeft') ? 1 : 0) - (has('d', 'ArrowRight') ? 1 : 0);   // left is positive
    if (want) this.keySteer = clamp(this.keySteer + want * RATE.steerOn * dt, -1, 1);
    else this.keySteer = Math.sign(this.keySteer) * Math.max(0, Math.abs(this.keySteer) - RATE.steerCentre * dt);
    const pad = this.gamepad();
    const steerPad = pad ? -pad.steer : 0;   // stick right steers right (negative)
    const steer = Math.abs(steerPad) > Math.abs(this.keySteer) ? steerPad : this.keySteer;
    const events = this.events;
    this.events = [];
    return {
      throttle: Math.max(this.throttle, pad ? pad.throttle : 0, this.pointer.throttle),
      pedal: Math.max(this.keyPedal, pad ? pad.pedal : 0, this.pointer.pedal),
      lever: this.lever, steer, steering: Math.abs(steer) > 0.02, events, source: this.source, pad: pad?.id,
    };
  }
}
