from .point3d import Point3D

class Smoother3D:
    def __init__(self, initialPoint, max_speed, acceleration, deceleration):
        self.position = initialPoint
        self.target = initialPoint
        self.velocity = Point3D(0, 0, 0)
        self.max_speed = max_speed
        self.acceleration = acceleration
        self.deceleration = deceleration

    def setTarget(self, targetPoint):
        self.target = targetPoint

    def tick(self, dt):
        direction = (self.target - self.position).normalize()
        distance = (self.target - self.position).magnitude()

        if distance < 0.01:
            # Close enough to stop
            self.velocity = Point3D(0, 0, 0)
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
