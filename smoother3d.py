from .point3d import Point3D

class Smoother3D:
    def __init__(self, initial_point, max_speed, acceleration, deceleration, dt_threshold):
        self.position = initial_point
        self.target = initial_point
        self.velocity = Point3D()
        self.max_speed = max_speed
        self.acceleration = acceleration
        self.deceleration = deceleration
        self.dt_threshold = dt_threshold

    def set_target(self, target_point):
        self.target = target_point

    def tick(self, dt):
        if dt > self.dt_threshold:
            # If there was no ticks for a while, don't smooth, just
            # jump to the new location
            self.position = self.target
            self.velocity = Point3D()
            return

        direction = (self.target - self.position).normalize()
        distance = (self.target - self.position).magnitude()

        if distance < 0.01:
            # Close enough to stop
            self.velocity = Point3D()
            self.position = self.target
            return

        speed = self.velocity.magnitude()
        if speed < self.max_speed:
            # Acceleration
            self.velocity += direction * self.acceleration * dt
            self.velocity = self.velocity.normalize() * min(self.velocity.magnitude(), self.max_speed)

        if distance < (speed**2 / (2 * self.deceleration)):
            # Deceleration
            self.velocity -= self.velocity.normalize() * self.deceleration * dt

        self.position += self.velocity * dt

    def get(self):
        return self.position
