"""Twist2d utility class."""


class Twist2d:
    """Represents a 2D twist."""

    def __init__(self, v_x=0, v_y=0, omega=0):
        self.v_x = v_x
        self.v_y = v_y
        self.omega = omega
