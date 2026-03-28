import math
from typing import Optional

from .point3d import Point3D

class IK4DOF:
    def __init__(self):
        # coxa1 - servo that rotates coxa in horizontal plane
        # coxa2 - Additionalk servo that rotates coxa extension in vertical plane

        # Dimensional parameters
        self.coxa1_h_offset: float
        self.coxa1_v_offset: float
        self.coxa2_length: float
        self.coxa2_min_angle: float
        self.coxa2_max_angle: float
        self.femur_length: float
        self.tibia_length: float

        # Angular parameters
        self.coxa1_for_perpendicular: float  # Servo angle for coxa to point perpendicular to the body (to the right or left, depending on the leg side)
        self.coxa2_angle_for_horizontal: float # Angle for coxa extension to be horizontal
        self.femur_angle_for_coxa2_parallel: float  # Servo angle for femur to be parallel to the coxa extension
        self.tibia_angle_for_femur_parallel: float  # Servo angle for tibia to be parallel to femur

        # Multipliers (to mirror, reverse servo direction or fine tune)
        self.coxa1_multiplier: float = 1
        self.coxa2_multiplier: float = 1
        self.femur_multiplier: float = 1
        self.tibia_multiplier: float = 1

    def get_angles(self, reach_to: Point3D) -> tuple[Optional[float], Optional[float], Optional[float], Optional[float]]:
        if self.coxa1_h_offset is None \
                or self.coxa1_v_offset is None \
                or self.femur_length is None \
                or self.tibia_length is None \
                or self.coxa2_length is None \
                or self.coxa2_min_angle is None \
                or self.coxa2_max_angle is None \
                or self.coxa1_for_perpendicular is None \
                or self.coxa2_angle_for_horizontal is None \
                or self.femur_angle_for_coxa2_parallel is None \
                or self.tibia_angle_for_femur_parallel is None:
            raise ValueError('Not all required parameters are set')

        # Translate to 2D xy
        x_2d_sign = 1 if reach_to.x >= 0 else -1
        x_2d = math.sqrt(reach_to.x ** 2 + reach_to.y ** 2) * x_2d_sign - self.coxa1_h_offset
        y_2d = reach_to.z - self.coxa1_v_offset

        # Turn coxa2 in the direction where the target point is, and translate further 3DOF
        # solution
        coxa2_angle = math.atan2(reach_to.y, reach_to.x) * 180 / math.pi
        coxa2_angle = min(max(coxa2_angle, self.coxa2_min_angle), self.coxa2_max_angle)
        coxa2_angle_rad = coxa2_angle * math.pi / 180

        coxa2_x = self.coxa2_length * math.cos(coxa2_angle_rad)
        coxa2_y = self.coxa2_length * math.sin(coxa2_angle_rad)

        x_2d -= coxa2_x
        y_2d -= coxa2_y

        possible_joints = self._circle_intersection(x_2d, y_2d, self.femur_length, self.tibia_length)

        # Chose the best joint
        selected_femur_angle = None
        selected_tibia_angle = None
        selected_j_x = None
        for joint in possible_joints:
            j_x = joint[0]
            j_y = joint[1]
            femur_angle = math.atan2(j_y, j_x) * 180 / math.pi
            tibia_angle = math.atan2(y_2d - j_y, x_2d - j_x) * 180 / math.pi


            if selected_j_x is None or selected_j_x < j_x:
                selected_j_x = j_x
                selected_femur_angle = femur_angle
                selected_tibia_angle = tibia_angle

        if selected_femur_angle is None or selected_tibia_angle is None:
            # Can't reach
            return None, None, None

        coxa1_angle = math.atan2(reach_to.y, reach_to.x) * 180 / math.pi
        if reach_to.x < 0:
            coxa1_angle += 180

        # Now we can calculate final angles
        final_coxa1_angle = self.coxa1_for_perpendicular + coxa1_angle * self.coxa1_multiplier
        final_coxa2_angle = self.coxa2_angle_for_horizontal + coxa2_angle
        final_femur_angle = self.femur_angle_for_coxa2_parallel + (selected_femur_angle - coxa2_angle) * self.femur_multiplier
        final_tibia_angle = self.tibia_angle_for_femur_parallel + (selected_tibia_angle - selected_femur_angle - coxa2_angle) * self.tibia_multiplier

        return (
            self._normalize_deg(final_coxa1_angle),
            self._normalize_deg(final_coxa2_angle),
            self._normalize_deg(final_femur_angle),
            self._normalize_deg(final_tibia_angle)
        )


    @staticmethod
    def _normalize_deg(angle):
        while angle < -180:
            angle += 360

        while angle >= 180:
            angle -= 360

        return angle


    @staticmethod
    def _circle_intersection(x2, y2, r1, r2) -> list[tuple[float, float]]:
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
