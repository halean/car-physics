"""Rear-axle dry-friction brake; MuJoCo solves resisting/holding torque."""
from dataclasses import dataclass


@dataclass
class BrakeControl:
    apply_at: float = 2.0
    release_at: float | None = None
    torque_nm: float = 75.0  # maximum resisting torque PER rear wheel
    ramp_seconds: float = .25

    def __call__(self, model, data):
        command = max(0.0, min(1.0, (data.time-self.apply_at)/self.ramp_seconds))
        if self.release_at is not None and data.time >= self.release_at:
            command = 0.0
        for name in ('rear_left', 'rear_right'):
            dof = model.joint(name+'_axle').dofadr[0]
            model.dof_frictionloss[dof] = command*self.torque_nm
        return command
