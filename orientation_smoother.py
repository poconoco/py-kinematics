from .orientation3d import Orientation3D

import math

class SimpleLerp:
    def __init__(self, start_val: float, speed_factor: float):
        self._current_val = start_val
        self._target_val = start_val
        self._speed_factor = speed_factor

    def set_target(self, target_val: float):
        self._target_val = target_val

    def tick(self, dt: float):
        error = self._target_val - self._current_val
        decay = 1.0 - math.exp(-self._speed_factor * dt)
        self._current_val += error * decay

    def get(self) -> float:
       return self._current_val 

class OrientationSmoother():
    def __init__(self, start_orientation: Orientation3D, speed_factor: float):
        self._pitch = SimpleLerp(start_orientation.pitch, speed_factor)
        self._roll = SimpleLerp(start_orientation.roll, speed_factor)
        self._yaw = SimpleLerp(start_orientation.yaw, speed_factor)

    def set_target(self, orientation: Orientation3D):
        self._pitch.set_target(orientation.pitch)
        self._roll.set_target(orientation.roll)
        self._yaw.set_target(orientation.yaw)

    def tick(self, dt: float):
        self._pitch.tick(dt)
        self._roll.tick(dt)
        self._yaw.tick(dt)

    def get(self) -> Orientation3D:
        return Orientation3D(self._pitch.get(), self._roll.get(), self._yaw.get())
