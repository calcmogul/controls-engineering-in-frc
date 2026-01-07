#!/usr/bin/env python3

"""Simulates flywheel model predictive control."""

import math
import sys

import frccontrol as fct
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import numpy.typing as npt
from sleipnir.autodiff import VariableMatrix
from sleipnir.optimization import bounds

from bookutil import latex
from bookutil.linear_mpc import LinearMPC

if "--noninteractive" in sys.argv:
    mpl.use("svg")


class FlywheelMPC(LinearMPC):
    def __init__(self, A, B, u_min, u_max):
        def cost(X, U, r) -> VariableMatrix:
            # Minimize sum squared error and control input
            Q = np.diag(1.0 / np.square([9.42]))
            R = np.diag(1.0 / np.square([12.0]))
            return sum(
                (r - X[:, k : k + 1]).T @ Q @ (r - X[:, k : k + 1])
                for k in range(X.shape[1])
            ) + sum(U[:, k : k + 1].T @ R @ U[:, k : k + 1] for k in range(U.shape[1]))

        def constraints(problem, X, U):
            # Input constraints
            for k in range(U.shape[1]):
                problem.subject_to(bounds(u_min, U[:, k : k + 1], u_max))

        def initial_guess(
            x, r, N
        ) -> tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
            return np.zeros((1, N + 1)), np.zeros((1, N))

        super().__init__(
            A=A,
            B=B,
            sample_period=0.1,
            cost=cost,
            constraints=constraints,
            initial_guess=initial_guess,
            prediction_horizon=0.1,
            timeout=0.1,
        )


class Flywheel:
    """An frccontrol system representing a flyhweel."""

    def __init__(self, dt: float):
        self.dt = dt

        # Number of motors
        num_motors = 1.0
        # Flywheel moment of inertia in kg-m^2
        J = 0.00032
        # Gear ratio
        G = 12.0 / 18.0
        self.plant = fct.flywheel(fct.MOTOR_775PRO, num_motors, J, G)

        # Sim variables
        self.sim = self.plant.to_discrete(self.dt)
        self.x = np.zeros((1, 1))
        self.u = np.zeros((1, 1))
        self.y = np.zeros((1, 1))

        self.u_min = np.array([[-12.0]])
        self.u_max = np.array([[12.0]])

        # States: angular velocity (rad/s)
        # Inputs: voltage (V)
        # Outputs: angular velocity (rad/s)
        self.feedback = FlywheelMPC(self.plant.A, self.plant.B, self.u_min, self.u_max)

    def update(self, r, next_r):
        # Update sim model
        self.x = self.sim.A @ self.x + self.sim.B @ self.u
        self.y = self.sim.C @ self.x + self.sim.D @ self.u

        self.u = np.clip(
            self.feedback.calculate(self.x, r),
            self.u_min,
            self.u_max,
        )


def main():
    """Entry point."""

    dt = 0.005
    flywheel = Flywheel(dt)

    # Set up graphing
    l0 = 0.1
    l1 = l0 + 5.0
    l2 = l1 + 0.1
    ts = np.arange(0, l2 + 5.0, dt)

    # Run simulation
    refs = []
    for t in ts:
        if t < l0:
            r = np.array([[0.0]])
        elif t < l1:
            r = np.array([[9000 / 60 * 2 * math.pi]])
        else:
            r = np.array([[0.0]])
        refs.append(r)
    r_rec, x_rec, u_rec, _ = fct.generate_time_responses(flywheel, refs)

    fct.plot_time_responses(
        ["Angular velocity (rad/s)"],
        ["Voltage (V)"],
        ts,
        r_rec,
        x_rec,
        u_rec,
    )
    if "--noninteractive" in sys.argv:
        latex.savefig("flywheel_mpc_response")
    else:
        plt.show()


if __name__ == "__main__":
    main()
