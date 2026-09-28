"""Panhard controls: governed engine through a friction clutch and a sliding
three-speed gearbox, rear rim-block brakes (per wheel) and the tiller servo."""
from dataclasses import dataclass, field
import json
import math
from pathlib import Path

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
    apply_at: float | None = None
    release_at: float | None = None

    def command(self, t):
        c = ramp(t, self.apply_at, ENGINE['brake_ramp_seconds'])
        return 0.0 if self.release_at is not None and t >= self.release_at else c


@dataclass
class Steer:
    """Tiller target; hold_heading adds a driver who keeps the car on its starting line."""
    degrees: float = 0.0
    start: float = .75
    seconds: float = .75
    hold_heading: bool = False
    k_yaw: float = 1.0
    k_lateral: float = .3

    def target(self, t, data=None):
        angle = math.radians(self.degrees)*ramp(t, self.start, self.seconds)
        if self.hold_heading and data is not None:
            x_axis = data.body('chassis').xmat.reshape(3, 3)[:, 0]
            angle -= self.k_yaw*math.atan2(x_axis[1], x_axis[0])+self.k_lateral*float(data.qpos[1])
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
        for wheel in ('rear_left', 'rear_right'):
            model.dof_frictionloss[model.joint(f'{wheel}_joint').dofadr[0]] = brake*ENGINE['rim_brake_capacity_nm_each']
        data.ctrl[model.actuator('column').id] = self.steer.target(t, data)
        log.update(brake=brake, steer_nm=float(data.actuator_force[model.actuator('column').id]))
        return log


def _has(model, name):
    try:
        model.actuator(name)
        return True
    except KeyError:
        return False
