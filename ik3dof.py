import math
from typing import Optional

from point3d import Point3D

class IK3DOF:
    def __init__(self):
        # Dimensional parameters
        self.coxa_h_offset: float = None
        self.coxa_v_offset: float = None
        self.femur_length: float = None
        self.tibia_length: float = None

        # Angular parameters
        self.coxa_sideway_angle: float = None  # Angle to send to servo for coxa to point straight sideways (left or right depending on multiplier)
        self.femur_horizontal_angle: float = None  # Angle to send to servo for femur to point horizontally
        self.tibia_femur_parallel_angle: float = None  # Angle to send to servo for tibia to point parallel to femur

        # Multipliers (to mirror, reverse servo direction or fine tune)
        self.coxa_multiplier: float = 1
        self.femur_multiplier: float = 1
        self.tibia_multiplier: float = 1

    def get_angles(self, reach_to: Point3D) -> tuple[Optional[float], Optional[float], Optional[float]]:
        if self.coxa_h_offset is None \
                or self.coxa_v_offset is None \
                or self.femur_length is None \
                or self.tibia_length is None \
                or self.coxa_sideway_angle is None \
                or self.femur_horizontal_angle is None \
                or self.tibia_femur_parallel_angle is None:
            raise ValueError('Not all parameters are set')

        # Translate to 2D xy
        x_2d = math.sqrt(reach_to.x ** 2 + reach_to.y ** 2) - self.coxa_h_offset
        y_2d = reach_to.z - self.coxa_v_offset

        possible_joints = self._circle_intersection(x_2d, y_2d, self.femur_length, self.tibia_length)
        if not possible_joints:
            # Can't reach
            return None, None, None

        # Chose the best joint
        selected_femur_angle = None
        selected_tibia_angle = None
        for joint in possible_joints:
            femur_angle = math.atan2(joint[1], joint[0]) * 180 / math.pi
            tibia_angle = math.atan2(y_2d - joint[1], x_2d - joint[0]) * 180 / math.pi

            if selected_femur_angle is None or selected_femur_angle < femur_angle:
                selected_femur_angle = femur_angle
                selected_tibia_angle = tibia_angle

        # Now we can calculate final angles
        final_coxa_angle = self.coxa_sideway_angle + (math.atan2(reach_to.y, reach_to.x) * 180 / math.pi) * self.coxa_multiplier
        final_femur_angle = self.femur_horizontal_angle + selected_femur_angle * self.femur_multiplier
        final_tibia_angle = self.tibia_femur_parallel_angle - selected_femur_angle + selected_tibia_angle * self.tibia_multiplier
        
        return final_coxa_angle, final_femur_angle, final_tibia_angle


    @staticmethod
    def _circle_intersection(x2, y2, r1, r2):
        # Distance to reach point
        d = math.sqrt(x2 ** 2 + y2 ** 2)

        if d == 0:
            # Reach point is at the start of a coxa, would not reach
            return []
        
        # Now find circles intersection point to find joint coordinates
        a = (r1 * r1 - r2 * r2 + d * d) / (2 * d)

        # Circles don't intersect
        if (r1 < a):
            return []


        h = math.sqrt(r1 * r1 - a * a)
        xt = a * (x2 / d)
        yt = a * (y2 / d)

        # Two solutions
        return [
            (
                xt + h * (y2 / d), 
                yt - h * (x2 / d)
            ),
            (
                xt - h * (y2 / d), 
                yt + h * (x2 / d)
            )
        ]
