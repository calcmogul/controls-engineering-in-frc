#!/usr/bin/env python3

"""Comparison of pose estimation methods."""

from abc import ABCMeta, abstractmethod
import math
import sys

import frccontrol as fct
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import StateSpace
from wpimath.geometry import Pose2d, Twist2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

from bookutil import latex

if "--noninteractive" in sys.argv:
    mpl.use("svg")


def linearized_differential_drive(motor, num_motors, m, r, rb, J, Gl, Gr, states):
    """
    Returns the linearized state-space model for a differential drive.

    States: [[x], [y], [theta], [left velocity], [right velocity]]
    Inputs: [[left voltage], [right voltage]]
    Outputs: [[theta], [left velocity], [right velocity]]

    Parameter ``motor``:
        Instance of DCMotor.
    Parameter ``num_motors``:
        Number of motors driving the mechanism.
    Parameter ``m``:
        Mass of robot in kg.
    Parameter ``r``:
        Radius of wheels in meters.
    Parameter ``rb``:
        Radius of robot in meters.
    Parameter ``J``:
        Moment of inertia of the differential drive in kg-m².
    Parameter ``Gl``:
        Gear ratio of left side of differential drive.
    Parameter ``Gr``:
        Gear ratio of right side of differential drive.
    Parameter ``states``:
        State vector around which to linearize model.
    Returns:
        StateSpace instance containing continuous model.
    """
    motor = fct.gearbox(motor, num_motors)

    C1 = -(Gl**2) * motor.Kt / (motor.Kv * motor.R * r**2)
    C2 = Gl * motor.Kt / (motor.R * r)
    C3 = -(Gr**2) * motor.Kt / (motor.Kv * motor.R * r**2)
    C4 = Gr * motor.Kt / (motor.R * r)
    theta = states[2, 0]
    vl = states[3, 0]
    vr = states[4, 0]
    v = (vr + vl) / 2.0
    if abs(v) < 1e-9:
        v = 1e-9
    c = math.cos(theta)
    s = math.sin(theta)
    A = np.array(
        [
            [0, 0, -v * s, 0.5 * c, 0.5 * c],
            [0, 0, v * c, 0.5 * s, 0.5 * s],
            [0, 0, 0, -0.5 / rb, 0.5 / rb],
            [0, 0, 0, (1 / m + rb**2 / J) * C1, (1 / m - rb**2 / J) * C3],
            [0, 0, 0, (1 / m - rb**2 / J) * C1, (1 / m + rb**2 / J) * C3],
        ]
    )
    B = np.array(
        [
            [0, 0],
            [0, 0],
            [0, 0],
            [(1 / m + rb**2 / J) * C2, (1 / m - rb**2 / J) * C4],
            [(1 / m - rb**2 / J) * C2, (1 / m + rb**2 / J) * C4],
        ]
    )
    C = np.array([[0, 0, 1, 0, 0], [0, 0, 0, 1, 0], [0, 0, 0, 0, 1]])
    D = np.zeros((3, 2))

    return StateSpace(A, B, C, D)


class Drivetrain(metaclass=ABCMeta):
    """An frccontrol system for a differential drive."""

    def __init__(self, dt):
        """
        Differential drive subsystem.

        Parameter ``dt``:
            Time between model/controller updates.
        """
        self.dt = dt

        # Radius of robot in meters
        self.rb = 0.59055 / 2.0

        self.plant = self.linearize(np.zeros((5, 1)))

        # Observer state estimate
        self.x_hat = np.zeros((7, 1))

        # Sim variables
        self.x = np.zeros((7, 1))
        self.u = np.zeros((2, 1))
        self.y = np.zeros((3, 1))

        self.u_min = np.array([[-12.0], [-12.0]])
        self.u_max = np.array([[12.0], [12.0]])

    def linearize(self, states):
        """
        Return differential drive model linearized around the given state.

        Parameter ``states``:
            State around which to linearize.
        """
        # Number of motors per side
        num_motors = 3.0

        # Gear ratio of differential drive
        G = 60.0 / 11.0

        # Drivetrain mass in kg
        m = 52
        # Radius of wheels in meters
        r = 0.08255 / 2.0
        # Moment of inertia of the differential drive in kg-m²
        J = 6.0

        return linearized_differential_drive(
            fct.MOTOR_CIM,
            num_motors,
            m,
            r,
            self.rb,
            J,
            G,
            G,
            states,
        )

    def f(self, x, u):
        """
        Nonlinear differential drive dynamics.

        Parameter ``x``:
            State vector.
        Parameter ``u``:
            Input vector.
        Returns:
            State derivative.
        """
        return (
            np.array(
                [
                    [(x[3, 0] + x[4, 0]) / 2.0 * math.cos(x[2, 0])],
                    [(x[3, 0] + x[4, 0]) / 2.0 * math.sin(x[2, 0])],
                    [(x[4, 0] - x[3, 0]) / (2.0 * self.rb)],
                    [self.plant.A[3, 3] * x[3, 0] + self.plant.A[3, 4] * x[4, 0]],
                    [self.plant.A[4, 3] * x[3, 0] + self.plant.A[4, 4] * x[4, 0]],
                    [x[3, 0]],
                    [x[4, 0]],
                ]
            )
            + np.block([[self.plant.B], [np.zeros((2, 2))]]) @ u
        )

    @abstractmethod
    def update_observer(self):
        """Update observer."""

    def update(self, r, next_r):
        """
        Advance the model by one timestep.

        Parameter ``r``:
            The current reference.
        Parameter ``next_r``:
            The next reference.
        """
        self.x = fct.rkdp(self.f, self.x, self.u, self.dt)

        self.update_observer()

        self.plant = self.linearize(self.x_hat[:5, :])

        # Feedforward
        rdot = (next_r - r) / self.dt
        self.u = np.linalg.pinv(self.plant.B) @ (
            rdot - self.f(np.block([[r], [np.zeros((2, 1))]]), np.zeros((2, 1)))[:5, :]
        )

        u_cap = np.max(np.abs(self.u))
        if u_cap > 12.0:
            self.u = self.u / u_cap * 12.0


class DrivetrainExact(Drivetrain):
    """An frccontrol system for a differential drive with exact observer."""

    def __init__(self, dt, x):
        Drivetrain.__init__(self, dt)

        self.x = x.copy()
        self.x_hat = x.copy()

    def update_observer(self):
        """Update observer."""
        self.x_hat = fct.rkdp(self.f, self.x_hat, self.u, self.dt)


class DrivetrainEuler(Drivetrain):
    """
    An frccontrol system for a differential drive with forward Euler observer.
    """

    def __init__(self, dt, x):
        Drivetrain.__init__(self, dt)

        self.x = x.copy()
        self.x_hat = x.copy()

    def update_observer(self):
        """Update observer."""
        v = (self.x[3, 0] + self.x[4, 0]) / 2.0
        omega = (self.x[4, 0] - self.x[3, 0]) / (2.0 * self.rb)

        self.x_hat[0, 0] += v * math.cos(self.x_hat[2, 0]) * self.dt
        self.x_hat[1, 0] += v * math.sin(self.x_hat[2, 0]) * self.dt
        self.x_hat[2, 0] += omega * self.dt
        self.x_hat[3, 0] = self.x[3, 0]
        self.x_hat[4, 0] = self.x[4, 0]
        self.x_hat[5, 0] = self.x[5, 0]
        self.x_hat[6, 0] = self.x[6, 0]


class DrivetrainSE3(Drivetrain):
    """An frccontrol system for a differential drive with SE(3) observer."""

    def __init__(self, dt, x):
        Drivetrain.__init__(self, dt)

        self.x = x.copy()
        self.x_hat = x.copy()

        self.prev_heading = x[2, 0]
        self.prev_left_pos = 0.0
        self.prev_right_pos = 0.0

    def update_observer(self):
        """Update observer."""
        heading = self.x[2, 0]
        left_pos = self.x[5, 0]
        right_pos = self.x[6, 0]

        dx = (left_pos + right_pos) / 2.0 - (
            self.prev_left_pos + self.prev_right_pos
        ) / 2.0
        dtheta = heading - self.prev_heading

        pose = Pose2d(self.x_hat[0, 0], self.x_hat[1, 0], self.x_hat[2, 0])
        pose = pose.exp(Twist2d(dx, 0.0, dtheta))

        self.x_hat[0, 0] = pose.x
        self.x_hat[1, 0] = pose.y
        self.x_hat[2, 0] = pose.rotation().radians()
        self.x_hat[3, 0] = self.x[3, 0]
        self.x_hat[4, 0] = self.x[4, 0]
        self.x_hat[5, 0] = left_pos
        self.x_hat[6, 0] = right_pos

        self.prev_heading = heading
        self.prev_left_pos = left_pos
        self.prev_right_pos = right_pos


def main():
    """Entry point."""
    dt = 0.02

    # Radius of robot in meters
    rb = 0.59055 / 2.0

    trajectory = TrajectoryGenerator.generateTrajectory(
        [Pose2d(1, 13, 0), Pose2d(10, 18, 0)],
        TrajectoryConfig(3.5, 3.5),
    )

    refs = []
    t_rec = np.arange(0, trajectory.totalTime(), dt)
    for t in t_rec:
        sample = trajectory.sample(t)
        vl = sample.velocity - sample.velocity * sample.curvature * rb
        vr = sample.velocity + sample.velocity * sample.curvature * rb
        refs.append(
            np.array(
                [
                    [sample.pose.X()],
                    [sample.pose.Y()],
                    [sample.pose.rotation().radians()],
                    [vl],
                    [vr],
                ]
            )
        )

    x = np.array(
        [
            [refs[0][0, 0]],
            [refs[0][1, 0]],
            [refs[0][2, 0]],
            [0.0],
            [0.0],
            [0.0],
            [0.0],
        ]
    )
    drivetrain_exact = DrivetrainExact(dt, x)
    drivetrain_euler = DrivetrainEuler(dt, x)
    drivetrain_se3 = DrivetrainSE3(dt, x)

    plt.figure(1)
    plt.xlabel("X position (m)")
    plt.ylabel("Y position (m)")

    x_rec_exact = np.zeros((drivetrain_exact.x.shape[0], 0))
    for i, r in enumerate(refs):
        if i < len(refs) - 1:
            next_r = refs[i + 1]
        else:
            next_r = r
        drivetrain_exact.update(r, next_r)
        x_rec_exact = np.concatenate((x_rec_exact, drivetrain_exact.x_hat), axis=1)
    plt.plot(x_rec_exact[0, :], x_rec_exact[1, :], label="Exact")

    x_rec_euler = np.zeros((drivetrain_euler.x.shape[0], 0))
    for i, r in enumerate(refs):
        if i < len(refs) - 1:
            next_r = refs[i + 1]
        else:
            next_r = r
        drivetrain_euler.update(r, next_r)
        x_rec_euler = np.concatenate((x_rec_euler, drivetrain_euler.x_hat), axis=1)
    plt.plot(x_rec_euler[0, :], x_rec_euler[1, :], label="Forward Euler")

    x_rec_se3 = np.zeros((drivetrain_se3.x.shape[0], 0))
    for i, r in enumerate(refs):
        if i < len(refs) - 1:
            next_r = refs[i + 1]
        else:
            next_r = r
        drivetrain_se3.update(r, next_r)
        x_rec_se3 = np.concatenate((x_rec_se3, drivetrain_se3.x_hat), axis=1)
    plt.plot(x_rec_se3[0, :], x_rec_se3[1, :], label="Pose exponential")

    plt.legend()

    if "--noninteractive" in sys.argv:
        latex.savefig("pose_estimation_comparison_xy")

    plt.figure(2)
    plt.xlabel("Time (s)")
    plt.ylabel("X error (cm)")
    plt.plot(
        t_rec, (x_rec_euler[0, :] - x_rec_exact[0, :]) * 1e2, label="Forward Euler"
    )
    plt.plot(
        t_rec, (x_rec_se3[0, :] - x_rec_exact[0, :]) * 1e2, label="Pose exponential"
    )
    plt.legend()

    if "--noninteractive" in sys.argv:
        latex.savefig("pose_estimation_comparison_x_error")

    plt.figure(3)
    plt.xlabel("Time (s)")
    plt.ylabel("Y error (cm)")
    plt.plot(
        t_rec, (x_rec_euler[1, :] - x_rec_exact[1, :]) * 1e2, label="Forward Euler"
    )
    plt.plot(
        t_rec, (x_rec_se3[1, :] - x_rec_exact[1, :]) * 1e2, label="Pose exponential"
    )
    plt.legend()

    if "--noninteractive" in sys.argv:
        latex.savefig("pose_estimation_comparison_y_error")

    plt.figure(4)
    plt.xlabel("Time (s)")
    plt.ylabel("Heading error (deg)")
    plt.plot(
        t_rec,
        np.degrees(x_rec_euler[2, :] - x_rec_exact[2, :]),
        label="Forward Euler",
    )
    plt.plot(
        t_rec,
        np.degrees(x_rec_se3[2, :] - x_rec_exact[2, :]),
        label="Pose exponential",
    )
    plt.legend()

    # Write max errors to .tex file
    if "--noninteractive" in sys.argv:
        with open(
            "pose_estimation_comparison_max_error.tex", "w", encoding="utf-8"
        ) as f:
            x_length = round((refs[-1][0, 0] - refs[0][0, 0]), 3)
            y_length = round((refs[-1][1, 0] - refs[0][1, 0]), 3)

            x_error = x_rec_euler[0, :] - x_rec_exact[0, :]
            x_max_error = round(x_error[np.argmax(np.abs(x_error))] * 1e2, 3)

            y_error = x_rec_euler[1, :] - x_rec_exact[1, :]
            y_max_error = round(y_error[np.argmax(np.abs(y_error))] * 1e2, 3)

            heading_error = x_rec_euler[2, :] - x_rec_exact[2, :]
            heading_max_error = round(
                math.degrees(heading_error[np.argmax(np.abs(heading_error))]), 3
            )

            f.write(
                f"The highest errors for the ${x_length}$ m by ${y_length}$ m trajectory are ${x_max_error}$ cm in $x$, ${y_max_error}$ cm in $y$, and ${heading_max_error}$ deg in heading.\n"
            )

    if "--noninteractive" in sys.argv:
        latex.savefig("pose_estimation_comparison_heading_error")
    else:
        plt.show()


if __name__ == "__main__":
    main()
