"""Governed engine with a belt-shift clutch, plus the existing brake and tiller."""
from dataclasses import dataclass, field
import json
import math
from pathlib import Path

ENGINE=json.loads((Path(__file__).with_name('engine.json')).read_text())
GOVERNED=ENGINE['governed_rpm']*math.tau/60
NO_LOAD=ENGINE['no_load_rpm']*math.tau/60
MAX_TORQUE=ENGINE['rated_power_w']/GOVERNED  # N m at the flywheel shaft


def engine_torque(speed):
    """Torque delivered through the belt at an engine-side speed in rad/s.

    Below governed speed the belt slips on the flywheel-driven pulley and passes
    the rated torque (the flywheel holds engine speed); above it, the governor
    cuts torque linearly to zero at no-load speed. No engine braking.
    """
    if speed <= GOVERNED:
        return MAX_TORQUE
    return MAX_TORQUE*max(0.0, (NO_LOAD-speed)/(NO_LOAD-GOVERNED))


@dataclass
class EngineControl:
    engage_at: float | None = .5
    disengage_at: float | None = None
    brake: object = None
    steering: object = None
    log: dict = field(default_factory=dict)

    def __call__(self, model, data):
        engaged = 0.0
        if self.engage_at is not None and data.time >= self.engage_at:
            engaged = min(1.0, (data.time-self.engage_at)/ENGINE['belt_shift_seconds'])
        if self.disengage_at is not None and data.time >= self.disengage_at:
            engaged = 0.0
        drive = model.actuator('drive').id
        speed = float(data.actuator_velocity[drive])
        data.ctrl[drive] = engaged*engine_torque(speed)
        brake = self.brake(model, data) if self.brake else 0.0
        if self.steering:
            self.steering(model, data)
        self.log.update(engaged=engaged, engine_speed=speed, torque=float(data.ctrl[drive]), brake=brake)
        return self.log


class Tiller:
    """Tiller ramp from experiment 004, without its brake schedule."""
    def __init__(self, direction):
        self.direction = direction

    def __call__(self, model, data):
        fraction = max(0, min(1, (data.time-.75)/.75))
        limit = model.actuator('tiller').ctrlrange
        target = self.direction*math.radians(15)*fraction
        data.ctrl[model.actuator('tiller').id] = max(limit[0], min(limit[1], target))
