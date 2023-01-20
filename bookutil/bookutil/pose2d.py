"""Pose2d utility class."""

import math
import numpy as np


class Pose2d:
    """Represents a 2D pose."""

    def __init__(self, x=0, y=0, theta=0):
        self.x = x
        self.y = y
        self.theta = theta

    def rotate_by(self, theta):
        """Rotate the pose counterclockwise by the given angle.

        Keyword arguments:
        theta -- Angle in radians
        """
        x = math.cos(theta) * self.x - math.sin(theta) * self.y
        y = math.sin(theta) * self.x + math.cos(theta) * self.y
        return Pose2d(x, y, self.theta + theta)

    def relative_to(self, pose):
        """Returns current pose relative to provided pose.

        Keyword arguments:
        pose -- pose used as reference point
        """
        # Clockwise rotation matrix
        R = np.array(
            [
                [math.cos(pose.theta), -math.sin(pose.theta), 0],
                [math.sin(pose.theta), math.cos(pose.theta), 0],
                [0, 0, 1],
            ]
        ).T
        temp = np.array(
            [
                [self.x - pose.x],
                [self.y - pose.y],
                [Pose2d.__get_continuous_error(self.theta - pose.theta)],
            ]
        )
        temp = R @ temp
        return Pose2d(temp[0, 0], temp[1, 0], temp[2, 0])

    def exp(self, twist):
        """Apply the given twist to update the pose.

        Keyword arguments:
        twist -- the change in pose in the robot's coordinate frame since the
                 previous pose update
        """
        # Compute change in pose in local coordinate frame
        if abs(twist.dtheta) < 1e-9:
            s = 1.0 - 1.0 / 6.0 * twist.dtheta**2
            c = 0.5 * twist.dtheta
        else:
            s = math.sin(twist.dtheta) / twist.dtheta
            c = (1.0 - math.cos(twist.dtheta)) / twist.dtheta
        dpose_r = Pose2d(
            twist.dx * s - twist.dy * c,
            twist.dx * c + twist.dy * s,
            twist.dtheta,
        )

        # Transform to global coordinate frame, then apply transformation
        return Pose2d(
            self.x
            + dpose_r.x * math.cos(self.theta)
            - dpose_r.y * math.sin(self.theta),
            self.y
            + dpose_r.x * math.sin(self.theta)
            + dpose_r.y * math.cos(self.theta),
            self.theta + dpose_r.theta,
        )

    @staticmethod
    def __get_continuous_error(error):
        error = math.fmod(error, 2 * math.pi)

        if abs(error) > math.pi:
            if error > 0:
                return error - 2 * math.pi
            else:
                return error + 2 * math.pi

        return error
