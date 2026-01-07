#!/usr/bin/env python3

"""
Simulates differential drive model predictive control with keep-out constraints.
"""

import math
import sys

import frccontrol as fct
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import numpy.typing as npt
from matplotlib import colors
from matplotlib.patches import Circle
from sleipnir import autodiff
from sleipnir.autodiff import VariableMatrix
from sleipnir.optimization import bounds

from bookutil import latex, plotutil
from bookutil.nonlinear_mpc import NonlinearMPC
from bookutil.trajectory import generate_trajectory, lerp

if "--noninteractive" in sys.argv:
    mpl.use("svg")


class DifferentialDriveMPC(NonlinearMPC):
    def __init__(self, f, u_min, u_max):
        def cost(X, U, r) -> VariableMatrix:
            # Minimize sum squared error
            Q = np.diag(1.0 / np.square([1.0, 1.0, 20.0, 10.0, 10.0]))
            return sum(
                (r - X[:, k : k + 1]).T @ Q @ (r - X[:, k : k + 1])
                for k in range(X.shape[1])
            )

        def constraints(problem, X, U):
            # Input constraints
            for k in range(U.shape[1]):
                problem.subject_to(bounds(u_min, U[:, k : k + 1], u_max))

            # Circle keep-out constraint
            for k in range(X.shape[1]):
                problem.subject_to(
                    (X[0, k] - 4.0) ** 2 + (X[1, k] - 13.5) ** 2 >= 2.0**2
                )

        def initial_guess(
            x, r, N
        ) -> tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
            X_0 = np.empty((5, N + 1))
            for k in range(N + 1):
                X_0[0, k] = lerp(x[0, 0], r[0, 0], k / N)
                X_0[1, k] = lerp(x[1, 0], r[1, 0], k / N)
                X_0[2, k] = math.atan2(r[1, 0] - x[1, 0], r[0, 0] - x[0, 0])
                X_0[3, k] = 0.0
                X_0[4, k] = 0.0
            return X_0, np.zeros((2, N))

        super().__init__(
            states=5,
            inputs=2,
            f=f,
            sample_period=0.1,
            cost=cost,
            constraints=constraints,
            initial_guess=initial_guess,
            prediction_horizon=2.0,
            timeout=0.1,
        )


class Drivetrain:
    """An frccontrol system for a differential drive."""

    def __init__(self, dt: float):
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

        # Sim variables
        self.x = np.zeros((5, 1))
        self.u = np.zeros((2, 1))
        self.y = np.zeros((5, 1))

        self.u_min = np.array([[-12.0], [-12.0]])
        self.u_max = np.array([[12.0], [12.0]])

        self.feedback = DifferentialDriveMPC(self.f_slp, self.u_min, self.u_max)

    def f_slp(self, x: VariableMatrix, u: VariableMatrix) -> VariableMatrix:
        xdot = VariableMatrix(5, 1)

        v = (x[3, 0] + x[4, 0]) / 2.0
        xdot[0, :] = v * autodiff.cos(x[2, 0])
        xdot[1, :] = v * autodiff.sin(x[2, 0])
        xdot[2, :] = (x[4, 0] - x[3, 0]) / (2.0 * self.rb)
        xdot[3:5, :] = self.velocity_A @ x[3:5, :] + self.velocity_B @ u

        return xdot

    def f(
        self, x: npt.NDArray[np.float64], u: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        xdot = np.empty((5, 1))

        v = (x[3, 0] + x[4, 0]) / 2.0
        xdot[0, :] = v * math.cos(x[2, 0])
        xdot[1, :] = v * math.sin(x[2, 0])
        xdot[2, :] = (x[4, 0] - x[3, 0]) / (2.0 * self.rb)
        xdot[3:5, :] = self.velocity_A @ x[3:5, :] + self.velocity_B @ u

        return xdot

    def update(self, r, next_r):
        # Update sim model
        self.x = fct.rkdp(self.f, self.x, self.u, self.dt)
        self.y = self.x

        self.u = self.feedback.calculate(self.x, r)


def main():
    """Entry point."""
    dt = 0.02

    # Radius of robot in meters
    rb = 0.59055 / 2.0

    initial_pose = fct.Pose2d.from_triplet(1, 13, 0)
    final_pose = fct.Pose2d.from_triplet(10, 18, 0)
    trajectory = generate_trajectory(
        [initial_pose, final_pose],
        3.5,
        3.5,
        3.5,
        3.5,
    )

    refs = []
    t_rec = np.arange(0, trajectory.total_time(), dt)
    for _ in t_rec:
        sample = trajectory.sample(trajectory.total_time())
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

    # States: x, y, heading, left velocity, right velocity
    # Inputs: left voltage, right voltage
    x = np.array(
        [
            [initial_pose.x + 0.5],
            [initial_pose.y + 0.5],
            [initial_pose.rotation.radians + math.pi / 4],
            [0],
            [0],
        ]
    )
    drivetrain = Drivetrain(dt)
    drivetrain.x = x

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
            [
                Circle(
                    (4.0, 13.5),
                    radius=2.0,
                    facecolor=colors.hsv_to_rgb((1, 0.8, 0.9)),
                    edgecolor=(1, 0.5, 0.5),
                    hatch="x",
                    label="Keep-out region",
                )
            ],
        )
        latex.savefig("diff_drive_mpc_xy")
    else:
        anim = plotutil.animate_xy(
            fig,
            r_rec[0, :],
            r_rec[1, :],
            x_rec[0, :],
            x_rec[1, :],
            dt,
            [
                Circle(
                    (4.0, 13.5),
                    radius=2.0,
                    facecolor=colors.hsv_to_rgb((1, 0.8, 0.9)),
                    edgecolor=(1, 0.5, 0.5),
                    hatch="x",
                    label="Keep-out region",
                )
            ],
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
        latex.savefig("diff_drive_mpc_response")
    else:
        plt.show()


if __name__ == "__main__":
    main()
