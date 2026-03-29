import math

def normalize_ik_deg(angle):
    while angle < -180:
        angle += 360

    while angle >= 180:
        angle -= 360

    return angle


def circle_intersection(x2, y2, r1, r2) -> list[tuple[float, float]]:

    # Distance to reach point
    d = math.sqrt(x2 ** 2 + y2 ** 2)

    if d == 0:
        # Reach point is at the start of a coxa, would not reach
        return []

    # Now find circles intersection point to find joint coordinates
    a = (r1 * r1 - r2 * r2 + d * d) / (2 * d)

    # Circles don't intersect
    if (abs(r1) < abs(a)):
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
