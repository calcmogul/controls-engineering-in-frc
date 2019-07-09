import math
import numpy as np


class Pose2d:
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

    def exp(self, twist, dt):
        """Apply the given twist to update the pose.

        Keyword arguments:
        twist -- a Twist2d object containing the linear and angular velocities
                 between updates
        dt -- the time in seconds between updates
        """
        # Compute change in pose in local coordinate frame
        if twist.omega > 1e-9:
            s = math.sin(twist.omega * dt) / twist.omega
            c = (math.cos(twist.omega * dt) - 1.0) / twist.omega
        else:
            s = dt - dt ** 3 * twist.omega ** 2 / 6.0
            c = -dt ** 2 * twist.omega / 2.0
        dpose_r = Pose2d(
            twist.v_x * s + twist.v_y * c,
            twist.v_x * -c + twist.v_y * s,
            twist.omega * dt,
        )

        # Transform to global coordinate frame, then apply transformation
        self.x += dpose_r.x * math.cos(self.theta) - dpose_r.y * math.sin(self.theta)
        self.y += dpose_r.x * math.sin(self.theta) + dpose_r.y * math.cos(self.theta)
        self.theta += dpose_r.theta

    @staticmethod
    def __get_continuous_error(error):
        error = math.fmod(error, 2 * math.pi)

        if abs(error) > math.pi:
            if error > 0:
                return error - 2 * math.pi
            else:
                return error + 2 * math.pi

        return error
