#!/usr/bin/env python3

"""
Simulates Ramsete unicycle controller with RKDP integration in chassis
coordinate frame.
"""

import math
import sys

import frccontrol as fct
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from wpimath.geometry import Pose2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

from bookutil import latex, plotutil

if "--noninteractive" in sys.argv:
    mpl.use("svg")


def ramsete(pose_desired, v_desired, omega_desired, pose, b, zeta):
    """Ramsete is a nonlinear time-varying feedback controller for unicycle
    models that drives the model to a desired pose along a two-dimensional
    trajectory.

    The reference pose, linear velocity, and angular velocity should come from
    a drivetrain trajectory.

    Keyword arguments:
    pose_desired -- the desired pose
    v_desired -- the desired linear velocity
    omega_desired -- the desired angular velocity
    pose -- the current pose
    b -- tuning parameter (b > 0) for which larger values make convergence more
         aggressive like a proportional term
    zeta -- tuning parameter (0 < zeta < 1) for which larger values provide more
            damping in response

    Returns:
    linear velocity and angular velocity commands
    """
    e = pose_desired.relativeTo(pose)

    k = 2 * zeta * math.sqrt(omega_desired**2 + b * v_desired**2)
    v = v_desired * e.rotation().cos() + k * e.x
    omega = (
        omega_desired
        + k * e.rotation().radians()
        + b * v_desired * np.sinc(e.rotation().radians()) * e.y
    )

    return v, omega


class Drivetrain:
    """An frccontrol system for a differential drive."""

    def __init__(self, dt):
        """Differential drive subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        """
        self.dt = dt

        # Radius of robot in meters
        self.rb = 0.59055 / 2.0

        # Gear ratio of differential drive
        G = 60.0 / 11.0

        # Drivetrain mass in kg
        m = 52
        # Radius of wheels in meters
        r = 0.08255 / 2.0
        # Moment of inertia of the differential drive in kg-m²
        J = 6.0

        motor = fct.models.gearbox(fct.models.MOTOR_CIM, 3.0)

        C1 = -(G**2) * motor.Kt / (motor.Kv * motor.R * r**2)
        C2 = G * motor.Kt / (motor.R * r)
        C3 = -(G**2) * motor.Kt / (motor.Kv * motor.R * r**2)
        C4 = G * motor.Kt / (motor.R * r)
        self.velocity_A = np.array(
            [
                [(1 / m + self.rb**2 / J) * C1, (1 / m - self.rb**2 / J) * C3],
                [(1 / m - self.rb**2 / J) * C1, (1 / m + self.rb**2 / J) * C3],
            ]
        )
        self.velocity_B = np.array(
            [
                [(1 / m + self.rb**2 / J) * C2, (1 / m - self.rb**2 / J) * C4],
                [(1 / m - self.rb**2 / J) * C2, (1 / m + self.rb**2 / J) * C4],
            ]
        )

        # States: x position (m), y position (m), heading (rad),
        #         left velocity (m/s), right velocity (m/s)
        # Inputs: left voltage (V), right voltage (V)
        # Outputs: heading (rad), left velocity (m/s), right velocity (m/s)
        self.observer = fct.ExtendedKalmanFilter(
            5,
            2,
            self.f,
            self.h,
            [0.5, 0.5, 10.0, 1.0, 1.0],
            [0.0001, 0.01, 0.01],
            self.dt,
        )
        self.velocity_controller = fct.LinearQuadraticRegulator(
            self.velocity_A, self.velocity_B, [0.95, 0.95], [12.0, 12.0], self.dt
        )

        # Sim variables
        self.x = np.zeros((5, 1))
        self.u = np.zeros((2, 1))
        self.y = np.zeros((3, 1))

        self.u_min = np.array([[-12.0], [-12.0]])
        self.u_max = np.array([[12.0], [12.0]])

    def f(self, x, u):
        """
        Nonlinear differential drive dynamics.

        Keyword arguments:
        x -- state vector
        u -- input vector

        Returns:
        dx/dt -- state derivative
        """
        return (
            np.array(
                [
                    [(x[3, 0] + x[4, 0]) / 2.0 * math.cos(x[2, 0])],
                    [(x[3, 0] + x[4, 0]) / 2.0 * math.sin(x[2, 0])],
                    [(x[4, 0] - x[3, 0]) / (2.0 * self.rb)],
                    [self.velocity_A[0, 0] * x[3, 0] + self.velocity_A[0, 1] * x[4, 0]],
                    [self.velocity_A[1, 0] * x[3, 0] + self.velocity_A[1, 1] * x[4, 0]],
                ]
            )
            + np.block([[np.zeros((3, 2))], [self.velocity_B]]) @ u
        )

    # pragma pylint: disable=unused-argument
    def h(self, x, u):
        """
        Nonlinear differential drive dynamics.

        Keyword arguments:
        x -- state vector
        u -- input vector

        Returns:
        dx/dt -- state derivative
        """
        return x[2:, :]

    def update(self, r, next_r):
        """
        Advance the model by one timestep.

        Keyword arguments:
        r -- the current reference
        next_r -- the next reference
        """
        # Update sim model
        self.x = fct.rkdp(self.f, self.x, self.u, self.dt)
        self.y = self.h(self.x, self.u)

        self.observer.predict(self.u, self.dt)
        self.observer.correct(self.u, self.y)

        # Feedforward
        rdot = (next_r - r) / self.dt
        u_ff = np.linalg.pinv(np.block([[np.zeros((3, 2))], [self.velocity_B]])) @ (
            rdot - self.f(r, np.zeros((2, 1)))
        )

        # Pose feedback
        v_cmd, omega_cmd = ramsete(
            Pose2d(r[0, 0], r[1, 0], r[2, 0]),
            (r[3, 0] + r[4, 0]) / 2,
            (r[4, 0] - r[3, 0]) / (2 * self.rb),
            Pose2d(
                self.observer.x_hat[0, 0],
                self.observer.x_hat[1, 0],
                self.observer.x_hat[2, 0],
            ),
            2.0,
            0.7,
        )

        # Velocity feedback
        u_fb = self.velocity_controller.calculate(
            self.observer.x_hat[3:, :],
            np.array([[v_cmd - omega_cmd * self.rb], [v_cmd + omega_cmd * self.rb]]),
        )

        self.u = u_ff + u_fb

        u_cap = np.max(np.abs(self.u))
        if u_cap > 12.0:
            self.u = self.u / u_cap * 12.0


def main():
    """Entry point."""
    dt = 0.02

    # Radius of robot in meters
    rb = 0.59055 / 2.0

    trajectory = TrajectoryGenerator.generateTrajectory(
        [Pose2d(1.330117, 13, 0), Pose2d(10.17, 18, 0)],
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
        [[refs[0][0, 0] + 0.5], [refs[0][1, 0] + 0.5], [math.pi / 2], [0], [0]]
    )
    drivetrain = Drivetrain(dt)
    drivetrain.x = x
    drivetrain.observer.x_hat = x

    # Run simulation
    r_rec, x_rec, u_rec, _ = fct.generate_time_responses(drivetrain, refs)

    fig = plt.figure(1)
    if "--noninteractive" in sys.argv:
        plotutil.plot_xy(
            fig,
            r_rec[0, :],
            r_rec[1, :],
            x_rec[0, :],
            x_rec[1, :],
        )
        latex.savefig("ramsete_traj_xy")
    else:
        anim = plotutil.animate_xy(  # pragma pylint: disable=unused-variable
            fig,
            r_rec[0, :],
            r_rec[1, :],
            x_rec[0, :],
            x_rec[1, :],
            dt,
        )

    fct.plot_time_responses(
        [
            "x position (m)",
            "y position (m)",
            "Heading (rad)",
            "Left velocity (m/s)",
            "Right velocity (m/s)",
        ],
        ["Left voltage (V)", "Right voltage (V)"],
        t_rec,
        r_rec,
        x_rec,
        u_rec,
    )

    if "--noninteractive" in sys.argv:
        latex.savefig("ramsete_traj_response")
    else:
        plt.show()


if __name__ == "__main__":
    main()
