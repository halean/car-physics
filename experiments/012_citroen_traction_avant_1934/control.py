"""Citroën Traction Avant controls: engine through a clutch and three-speed gearbox to the front
wheels; the hydraulic pedal works all four drums (joint friction at each wheel), the hand lever
the rear drums only; steering servo at the sector shaft (the steering box output)."""
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import numpy as np

ENGINE = json.loads((Path(__file__).with_name('engine.json')).read_text())
GOVERNED = ENGINE['governed_rpm']*math.tau/60
NO_LOAD = ENGINE['no_load_rpm']*math.tau/60
MAX_TORQUE = ENGINE['rated_power_w']/GOVERNED   # N m at the crankshaft


def engine_torque(speed):
    """Clutch slips below governed speed (flywheels hold the engine); governor droop above."""
    if speed <= GOVERNED:
        return MAX_TORQUE
    return MAX_TORQUE*max(0.0, (NO_LOAD-speed)/(NO_LOAD-GOVERNED))


def ramp(t, start, seconds):
    return 0.0 if start is None else max(0.0, min(1.0, (t-start)/seconds))


@dataclass
class Brake:
    """kind='hand': the rear drums. kind='foot': all four drums, front and rear."""
    apply_at: float | None = None
    release_at: float | None = None
    kind: str = 'hand'

    def command(self, t):
        c = ramp(t, self.apply_at, ENGINE['brake_ramp_seconds'])
        return 0.0 if self.release_at is not None and t >= self.release_at else c


@dataclass
class Steer:
    """Steering target (at the pitman); hold_heading adds a driver who keeps the car on its starting line."""
    degrees: float = 0.0
    start: float = .75
    seconds: float = .75
    hold_heading: bool = False
    k_yaw: float = 1.0
    k_lateral: float = .3

    wheelbase: float = 2.91
    # 1.6 s (44 m at top speed): with 0.8 s this car weaves when coasting at 27 m/s (a finding in dynamics).
    preview_s: float = 1.6

    def target(self, t, data=None):
        angle = math.radians(self.degrees)*ramp(t, self.start, self.seconds)
        if self.hold_heading and data is not None:
            # Pure pursuit to a point on the starting line ~0.8 s ahead: the effective gain falls
            # with speed (fixed gains, fine at 3-9 m/s, went unstable above ~11 m/s).
            x_axis = data.body('chassis').xmat.reshape(3, 3)[:, 0]
            yaw = math.atan2(x_axis[1], x_axis[0])
            look = max(4.0, self.preview_s*float(np.linalg.norm(data.qvel[:2])))
            error = float(data.qpos[1])+look*math.sin(yaw)
            angle -= math.atan(2*self.wheelbase*error/look**2)
        return angle


@dataclass
class Controller:
    """gears: [(time, 'first'|'second'|'third'|None)]; a new gear re-engages the clutch."""
    gears: list = field(default_factory=list)
    brake: Brake = field(default_factory=Brake)
    steer: Steer = field(default_factory=Steer)

    def gear_at(self, t):
        current, since = None, None
        for when, gear in self.gears:
            if t >= when:
                current, since = gear, when
        return current, since

    def __call__(self, model, data):
        t = data.time
        gear, since = self.gear_at(t)
        log = dict(gear=0.0, engine_rpm=0.0, torque=0.0)
        for i, name in enumerate(ENGINE['gearbox_ratios']):
            act = model.actuator(f'drive_{name}').id if model.nu > 1 and _has(model, f'drive_{name}') else None
            if act is None:
                continue
            data.ctrl[act] = 0.0
            if name == gear:
                speed = float(data.actuator_velocity[act])
                data.ctrl[act] = ramp(t, since, ENGINE['clutch_seconds'])*engine_torque(speed)
                log.update(gear=float(i+1), engine_rpm=speed*60/math.tau, torque=float(data.ctrl[act]))
        brake = self.brake.command(t)
        front = brake if self.brake.kind == 'foot' else 0.0     # the pedal reaches the front drums too
        for wheel in ('rear_left', 'rear_right'):
            model.dof_frictionloss[model.joint(f'{wheel}_joint').dofadr[0]] = brake*ENGINE['rear_drum_capacity_nm_each']
        for wheel in ('front_left', 'front_right'):
            model.dof_frictionloss[model.joint(f'{wheel}_joint').dofadr[0]] = front*ENGINE['front_drum_capacity_nm_each']
        data.ctrl[model.actuator('column').id] = self.steer.target(t, data)
        log.update(brake=brake, steer_nm=float(data.actuator_force[model.actuator('column').id]))
        return log


def _has(model, name):
    try:
        model.actuator(name)
        return True
    except KeyError:
        return False
