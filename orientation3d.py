import math

class Orientation3D:
    def __init__(self, pitch=0, roll=0, yaw=0):
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw

    def __repr__(self):
        return f"Orientation3D({int(self.pitch)},\t{int(self.roll)},\t{int(self.yaw)})"
