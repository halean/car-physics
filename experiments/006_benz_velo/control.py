"""Velo controls: governed engine with a two-speed belt shift, countershaft band
brake (acting through the differential) and the handwheel/column servo."""
from dataclasses import dataclass, field
import json
import math
from pathlib import Path

ENGINE = json.loads((Path(__file__).with_name('engine.json')).read_text())
GOVERNED = ENGINE['governed_rpm']*math.tau/60
NO_LOAD = ENGINE['no_load_rpm']*math.tau/60
MAX_TORQUE = ENGINE['rated_power_w']/GOVERNED   # N m at the flywheel shaft


def engine_torque(speed):
    """Belt slips below governed speed (flywheel holds the engine); governor droop above."""
    if speed <= GOVERNED:
        return MAX_TORQUE
    return MAX_TORQUE*max(0.0, (NO_LOAD-speed)/(NO_LOAD-GOVERNED))


def ramp(t, start, seconds):
    return 0.0 if start is None else max(0.0, min(1.0, (t-start)/seconds))


@dataclass
class Brake:
    """kind='band': foot brake on the countershaft, through the differential (tendon friction).
    kind='drums': hand brake, a drum on each rear wheel (joint friction)."""
    apply_at: float | None = None
    release_at: float | None = None
    kind: str = 'band'

    def command(self, t):
        c = ramp(t, self.apply_at, ENGINE['brake_ramp_seconds'])
        return 0.0 if self.release_at is not None and t >= self.release_at else c


@dataclass
class Steer:
    """Handwheel target. hold_heading=True adds a driver who keeps the car on its
    starting line: column = -k_yaw*heading error - k_lateral*sideways offset (flat road)."""
    degrees: float = 0.0
    start: float = .75
    seconds: float = .75
    hold_heading: bool = False
    k_yaw: float = 1.0          # rad of column per rad of heading error
    k_lateral: float = .3       # rad of column per m of sideways offset

    def target(self, t, data=None):
        angle = math.radians(self.degrees)*ramp(t, self.start, self.seconds)
        if self.hold_heading and data is not None:
            x_axis = data.body('chassis').xmat.reshape(3, 3)[:, 0]
            yaw = math.atan2(x_axis[1], x_axis[0])
            angle -= self.k_yaw*yaw+self.k_lateral*float(data.qpos[1])
        return angle


@dataclass
class Controller:
    """gears: [(time, 'low'|'high'|None)], applied in order; None disengages the belt."""
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
        for name in ('low', 'high'):
            try:
                act = model.actuator(f'drive_{name}').id
            except KeyError:
                continue
            data.ctrl[act] = 0.0
            if name == gear:
                speed = float(data.actuator_velocity[act])
                data.ctrl[act] = ramp(t, since, ENGINE['belt_shift_seconds'])*engine_torque(speed)
                log.update(gear=1.0 if name == 'low' else 2.0, engine_rpm=speed*60/math.tau,
                           torque=float(data.ctrl[act]))
        brake = self.brake.command(t)
        band = brake if self.brake.kind == 'band' else 0.0
        drums = brake if self.brake.kind == 'drums' else 0.0
        model.tendon_frictionloss[model.tendon('differential').id] = band*ENGINE['brake_capacity_nm_at_wheels']
        for wheel in ('rear_left', 'rear_right'):
            model.dof_frictionloss[model.joint(f'{wheel}_joint').dofadr[0]] = drums*ENGINE['rear_drum_capacity_nm_each']
        data.ctrl[model.actuator('column').id] = self.steer.target(t, data)
        log['brake'] = brake
        log['steer_nm'] = float(data.actuator_force[model.actuator('column').id])
        return log
