"""Twist2d utility class."""


class Twist2d:
    """Represents a 2D twist."""

    def __init__(self, dx=0, dy=0, dtheta=0):
        self.dx = dx
        self.dy = dy
        self.dtheta = dtheta
