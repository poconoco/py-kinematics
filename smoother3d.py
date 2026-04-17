import math

from .point3d import Point3D


class Smoother3D:
    def __init__(self, start_pos: Point3D, max_speed: float, acceleration: float):
        self._max_abs_speed = max_speed
        self._acceleration = acceleration
        self._current_pos = start_pos
        self._target_pos = start_pos
        self._current_speed = Point3D()

    def set_target(self, target_pos: Point3D):
        self._target_pos = target_pos

    def get(self) -> Point3D:
        return self._current_pos

    def tick(self, dt: float):
        target_distance = self._current_pos.distance(self._target_pos)
        
        # Snap to target if we are incredibly close to prevent micro-jitter
        if target_distance < 0.001:
            self._current_pos = self._target_pos
            self._current_speed = Point3D()
            return

        target_direction = (self._target_pos - self._current_pos).normalized()
        abs_speed = self._current_speed.magnitude()

        # Maximum speed we can go and still stop in time
        # Derived from: v^2 = v0^2 + 2ad
        max_allowed_speed = math.sqrt(2.0 * self._acceleration * target_distance)
        
        # Goal speed is the lowest of either the physical limit or safe braking speed
        goal_speed = min(self._max_abs_speed, max_allowed_speed)

        # Smoothly accelerate or decelerate towards the goal speed
        if abs_speed < goal_speed:
            abs_speed = min(abs_speed + self._acceleration * dt, goal_speed)
        else:
            # max() ensures no negative speeds
            abs_speed = max(abs_speed - self._acceleration * dt, 0.0)

        self._current_speed = target_direction * abs_speed
        step_vector = self._current_speed * dt

        # Overshoot protection: Don't step further than the target distance
        if step_vector.magnitude() >= target_distance:
            self._current_pos = self._target_pos
            self._current_speed = Point3D()
        else:
            self._current_pos += step_vector
