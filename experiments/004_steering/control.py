"""Bounded tiller position command and the existing passive rear brake."""
import importlib.util
import math
from pathlib import Path
import sys

path=Path(__file__).resolve().parents[1]/'003_brakes/control.py'
spec=importlib.util.spec_from_file_location('rear_brake_control',path)
module=importlib.util.module_from_spec(spec)
sys.modules[spec.name]=module
spec.loader.exec_module(module)
BrakeControl=module.BrakeControl


class SteeringControl:
    def __init__(self,direction=1):
        self.direction=direction
        self.brake=BrakeControl(apply_at=3.0)

    def __call__(self,model,data):
        fraction=max(0,min(1,(data.time-.75)/.75))
        target=self.direction*math.radians(15)*fraction
        model_limit=model.actuator('tiller').ctrlrange
        data.ctrl[model.actuator('tiller').id]=max(model_limit[0],min(model_limit[1],target))
        return self.brake(model,data)
