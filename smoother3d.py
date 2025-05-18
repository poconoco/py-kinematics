from .point3d import Point3D

class Smoother3D:
    def __init__(self, start_pos: Point3D, max_speed: float, acceleration: float, dt_threshold: float):
        self._max_abs_speed = max_speed
        self._acceleration = acceleration
        self._dt_threshold = dt_threshold
        self._current_pos = start_pos
        self._target_pos = start_pos
        self._current_speed = Point3D()

    def write(self, target_pos):
        self._target_pos = target_pos

    def get_current_pos(self):
        return self._current_pos

    def tick(self, dt):
        target_distance = self._current_pos.distance(self._target_pos)
        target_direction = (self._target_pos - self._current_pos).normalize()
        abs_speed = self._current_speed.magnitude()
        # abs_target_distance = abs(target_distance)

        if self.near_zero(target_distance):
            self._current_speed = Point3D()
        else:
            braking_distance = (abs_speed ** 2) / (2 * self._acceleration)

            if target_distance > braking_distance:
                if self.near_zero(abs_speed - self._max_abs_speed):
                    pass  # Cruising at max speed, do nothing
                elif abs_speed < self._max_abs_speed:
                    abs_speed = self.apply_acceleration(abs_speed, self._acceleration, dt)
                elif abs_speed > self._max_abs_speed:
                    abs_speed = self.apply_acceleration(abs_speed, -self._acceleration, dt)
            else:
                required_braking_accel = (abs_speed ** 2) / (2 * target_distance)
                abs_speed = self.apply_acceleration(abs_speed, -required_braking_accel, dt)

            self._current_speed = target_direction * abs_speed

        delta_distance = self._current_speed * dt
        self._current_pos += delta_distance

        if self.near_zero(self._current_pos - self._target_pos):
            self._current_pos = self._target_pos

        self._servo.write(self._current_pos)

    def apply_acceleration(self, speed, accel, dt):
        delta_a = accel * dt
        return min(speed + delta_a, self._max_abs_speed)

    @staticmethod
    def near_zero(value, threshold=0.01):
        return abs(value) < threshold
