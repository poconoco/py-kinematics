import math

from typing import Optional

from .point3d import Point3D
from .utils import normalize_ik_deg, circle_intersection

class IK3DOF:
    def __init__(self):
        # Dimensional parameters
        self.coxa_h_offset: float
        self.coxa_v_offset: float
        self.femur_length: float
        self.tibia_length: float

        # Angular parameters
        self.coxa_angle_for_perpendicular: float  # Servo angle for coxa to point perpendicular to the body (to the right or left, depending on the leg side)
        self.femur_angle_for_horizontal: float  # Servo angle for femur to be horizontal
        self.tibia_angle_for_femur_parallel: float  # Servo angle for tibia to be parallel to femur

        # Multipliers (to mirror, reverse servo direction or fine tune)
        self.coxa_multiplier: float = 1
        self.femur_multiplier: float = 1
        self.tibia_multiplier: float = 1

    def get_angles(self, reach_to: Point3D) -> tuple[Optional[float], Optional[float], Optional[float]]:
        if self.coxa_h_offset is None \
                or self.coxa_v_offset is None \
                or self.femur_length is None \
                or self.tibia_length is None \
                or self.coxa_angle_for_perpendicular is None \
                or self.femur_angle_for_horizontal is None \
                or self.tibia_angle_for_femur_parallel is None:
            raise ValueError('Not all parameters are set')

        # Translate to 2D xy
        x_2d_sign = 1 if reach_to.x >= 0 else -1
        x_2d = math.sqrt(reach_to.x ** 2 + reach_to.y ** 2) * x_2d_sign - self.coxa_h_offset
        y_2d = reach_to.z - self.coxa_v_offset

        possible_joints = circle_intersection(x_2d, y_2d, self.femur_length, self.tibia_length)

        # Chose the best joint
        selected_femur_angle = None
        selected_tibia_angle = None
        for joint in possible_joints:
            j_x = joint[0]
            j_y = joint[1]
            femur_angle = math.atan2(j_y, j_x) * 180 / math.pi
            tibia_angle = math.atan2(y_2d - j_y, x_2d - j_x) * 180 / math.pi


            if selected_femur_angle is None or (tibia_angle - femur_angle) < (selected_tibia_angle - selected_femur_angle):
                selected_femur_angle = femur_angle
                selected_tibia_angle = tibia_angle

        if selected_femur_angle is None or selected_tibia_angle is None:
            # Can't reach
            return None, None, None

        coxa_raw_angle = math.atan2(reach_to.y, reach_to.x) * 180 / math.pi
        if reach_to.x < 0:
            coxa_raw_angle += 180

        # Now we can calculate final angles
        final_coxa_angle = self.coxa_angle_for_perpendicular + coxa_raw_angle * self.coxa_multiplier
        final_femur_angle = self.femur_angle_for_horizontal + selected_femur_angle * self.femur_multiplier
        final_tibia_angle = self.tibia_angle_for_femur_parallel + (selected_tibia_angle - selected_femur_angle) * self.tibia_multiplier
        
        return (
            normalize_ik_deg(final_coxa_angle),
            normalize_ik_deg(final_femur_angle),
            normalize_ik_deg(final_tibia_angle)
        )

