#!/usr/bin/env python3

"""
Simulates LTV differential drive controller with RKDP integration in chassis
coordinate frame.
"""

import math
import sys

import frccontrol as fct
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

from bookutil import latex, plotutil
from bookutil.trajectory import generate_trajectory

if "--noninteractive" in sys.argv:
    mpl.use("svg")


class Drivetrain:
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

        # Gear ratio of differential drive
        G = 60.0 / 11.0

        # Drivetrain mass in kg
        m = 52
        # Radius of wheels in meters
        r = 0.08255 / 2.0
        # Moment of inertia of the differential drive in kg-m²
        J = 6.0

        motor = fct.gearbox(fct.MOTOR_CIM, 3.0)

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

        # Sim variables
        self.x = np.zeros((5, 1))
        self.u = np.zeros((2, 1))
        self.y = np.zeros((3, 1))

        self.u_min = np.array([[-12.0], [-12.0]])
        self.u_max = np.array([[12.0], [12.0]])

    def linearize_for_lqr(self, states):
        """
        Return differential drive model linearized around the given state.

        Parameter ``states``:
            State around which to linearize.
        """
        # Radius of robot in meters
        rb = 0.59055 / 2.0

        # Gear ratio of differential drive
        G = 60.0 / 11.0

        # Drivetrain mass in kg
        m = 52
        # Radius of wheels in meters
        r = 0.08255 / 2.0
        # Moment of inertia of the differential drive in kg-m²
        J = 6.0

        motor = fct.gearbox(fct.MOTOR_CIM, 3.0)

        C1 = -(G**2) * motor.Kt / (motor.Kv * motor.R * r**2)
        C2 = G * motor.Kt / (motor.R * r)
        C3 = -(G**2) * motor.Kt / (motor.Kv * motor.R * r**2)
        C4 = G * motor.Kt / (motor.R * r)
        vl = states[3, 0]
        vr = states[4, 0]
        v = (vr + vl) / 2.0
        if abs(v) < 1e-9:
            v = 1e-9
        A = np.array(
            [
                [0, 0, 0, 0.5, 0.5],
                [0, 0, v, 0, 0],
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

        return A, B

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

        Parameter ``x``:
            State vector.
        Parameter ``u``:
            Input vector.
        Returns:
            State derivative.
        """
        return x[2:, :]

    def update(self, r, next_r):
        """
        Advance the model by one timestep.

        Parameter ``r``:
            The current reference.
        Parameter ``next_r``:
            The next reference.
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

        A, B = self.linearize_for_lqr(self.observer.x_hat)

        K = fct.LinearQuadraticRegulator(
            A,
            B,
            [0.05, 0.125, 10.0, 0.95, 0.95],
            [12.0, 12.0],
            self.dt,
        ).K

        rot = self.observer.x_hat[2, 0]
        in_robot_frame = np.array(
            [
                [math.cos(rot), math.sin(rot), 0, 0, 0],
                [-math.sin(rot), math.cos(rot), 0, 0, 0],
                [0, 0, 1, 0, 0],
                [0, 0, 0, 1, 0],
                [0, 0, 0, 0, 1],
            ]
        )
        e = r - self.observer.x_hat
        u_fb = K @ in_robot_frame @ e

        self.u = u_ff + u_fb

        u_cap = np.max(np.abs(self.u))
        if u_cap > 12.0:
            self.u = self.u / u_cap * 12.0


def main():
    """Entry point."""
    dt = 0.02

    # Radius of robot in meters
    rb = 0.59055 / 2.0

    trajectory = generate_trajectory(
        [fct.Pose2d.from_triplet(1, 13, 0), fct.Pose2d.from_triplet(10, 18, 0)],
        3.5,
        3.5,
        3.5,
        3.5,
    )

    refs = []
    t_rec = np.arange(0, trajectory.total_time(), dt)
    for t in t_rec:
        sample = trajectory.sample(t)
        vl = sample.v - sample.ω * rb
        vr = sample.v + sample.ω * rb
        refs.append(
            np.array(
                [
                    [sample.x],
                    [sample.y],
                    [sample.θ],
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

    fig = plt.figure()
    if "--noninteractive" in sys.argv:
        plotutil.plot_xy(
            fig,
            r_rec[0, :],
            r_rec[1, :],
            x_rec[0, :],
            x_rec[1, :],
        )
        latex.savefig("ltv_diff_drive_traj_xy")
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
        latex.savefig("ltv_diff_drive_traj_response")
    else:
        plt.show()


if __name__ == "__main__":
    main()
