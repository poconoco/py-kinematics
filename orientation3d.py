import math

class Orientation3D:
    def __init__(self, pitch: float = 0, roll: float = 0, yaw: float = 0):
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw

    def copy(self):
        return Orientation3D(self.pitch, self.roll, self.yaw)

    def __repr__(self) -> str:
        return f'Orientation3D({int(self.pitch)},\t{int(self.roll)},\t{int(self.yaw)})'
