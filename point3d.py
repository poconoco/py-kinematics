import math

from typing import Optional

from .orientation3d import Orientation3D

class Point3D:
    def __init__(self, x: float = 0, y: float = 0, z: float = 0):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, other: 'Point3D') -> 'Point3D':
        return Point3D(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: 'Point3D') -> 'Point3D':
        return Point3D(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> 'Point3D':
        return Point3D(self.x * scalar, self.y * scalar, self.z * scalar)

    def __truediv__(self, scalar: float) -> 'Point3D':
        return Point3D(self.x / scalar, self.y / scalar, self.z / scalar)
    
    def copy(self):
        return Point3D(self.x, self.y, self.z)

    def magnitude(self) -> float:
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def normalized(self) -> 'Point3D':
        mag = self.magnitude()
        return Point3D(0, 0, 0) if mag == 0 else self / mag

    def distance(self, other: 'Point3D') -> float:
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2)

    def cube_to_sphere(self):
        # This is to map the coordinates within a cube (-1 to 1 in all coordinates independently)
        # To sphere with radius 1

        length = math.sqrt(self.x**2 + self.y**2 + self.z**2)

        if length == 0:
            return Point3D(0, 0, 0)

        max_comp = max(abs(self.x), abs(self.y), abs(self.z))
        scale = max_comp / length

        return Point3D(self.x * scale, self.y * scale, self.z * scale)

    def __repr__(self) -> str:
        return f"Point3D({self.x:.2f}, {self.y:.2f}, {self.z:.2f})"

    def rotate(self, orientation: Orientation3D, origin: Optional['Point3D'] = None) -> 'Point3D':
        """
        Rotates this point around another 'origin' Point3D using the given Orientation3D.
        Assumes standard coordinate mapping: Pitch=X, Roll=Y, Yaw=Z.
        """
        if origin is None:
            origin = Point3D()  # Default to (0, 0, 0) if no origin provided

        # Translate the point so the rotation origin is at (0, 0, 0)
        tx = self.x - origin.x
        ty = self.y - origin.y
        tz = self.z - origin.z

        pitch = math.radians(orientation.pitch)
        roll = math.radians(orientation.roll)
        yaw = math.radians(orientation.yaw)

        # Pre-calculate sine and cosine for efficiency
        cp, sp = math.cos(pitch), math.sin(pitch)
        cr, sr = math.cos(roll), math.sin(roll)
        cy, sy = math.cos(yaw), math.sin(yaw)

        # Apply rotations (Order matters in 3D! This uses Pitch -> Roll -> Yaw)

        ## Rotate around X-axis (Pitch)
        x1 = tx
        y1 = ty * cp - tz * sp
        z1 = ty * sp + tz * cp

        ## Rotate around Y-axis (Roll)
        x2 = x1 * cr + z1 * sr
        y2 = y1
        z2 = -x1 * sr + z1 * cr

        ## Rotate around Z-axis (Yaw)
        x3 = x2 * cy - y2 * sy
        y3 = x2 * sy + y2 * cy
        z3 = z2

        ## Translate the point back
        return Point3D(x3 + origin.x, y3 + origin.y, z3 + origin.z)


    def rotate_around_z(self, cx: float, cy: float, angle: float):
        cos_theta = math.cos(angle)
        sin_theta = math.sin(angle)

        translated_x = self.x - cx
        translated_y = self.y - cy

        rotated_x = translated_x * cos_theta - translated_y * sin_theta
        rotated_y = translated_x * sin_theta + translated_y * cos_theta

        final_x = rotated_x + cx
        final_y = rotated_y + cy

        return Point3D(final_x, final_y, self.z)


    @staticmethod
    def from_dict(d: dict) -> 'Point3D':
        return Point3D(d['x'], d['y'], d['z'])
