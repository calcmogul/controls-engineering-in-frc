#!/usr/bin/env python3

"""Demonstrates PI controller with overshoot."""

import math
import sys

import frccontrol as fct
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

from bookutil import latex

if "--noninteractive" in sys.argv:
    mpl.use("svg")


class Flywheel:
    """frccontrol system for flywheel with integrator."""

    def __init__(self, dt):
        """Flywheel subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        """
        self.dt = dt

        # Number of motors
        num_motors = 1.0
        # Flywheel moment of inertia in kg-m^2
        J = 0.00032
        # Gear ratio
        G = 12.0 / 18.0
        self.plant = fct.models.flywheel(fct.models.MOTOR_775PRO, num_motors, J, G)

        # Sim variables
        self.sim = self.plant.to_discrete(self.dt)
        self.x = np.zeros((1, 1))
        self.u = np.zeros((1, 1))
        self.y = np.zeros((1, 1))

        # States: angular velocity (rad/s)
        # Inputs: voltage (V)
        # Outputs: angular velocity (rad/s)
        self.feedback = fct.LinearQuadraticRegulator(
            self.plant.A, self.plant.B, [200.0], [12.0], self.dt
        )

        self.u_min = np.array([[-12.0]])
        self.u_max = np.array([[12.0]])

        self.integrator = 0.0
        self.Ki = 0.2

    # pylint: disable=unused-argument
    def update(self, r, next_r):
        """
        Advance the model by one timestep.

        Keyword arguments:
        r -- the current reference
        next_r -- the next reference
        """
        # Update sim model
        self.x = self.sim.A @ self.x + self.sim.B @ self.u
        self.y = self.sim.C @ self.x + self.sim.D @ self.u

        e = (r - self.x)[0, 0]
        self.integrator = np.clip(
            self.integrator + e * self.dt, self.u_min / self.Ki, self.u_max / self.Ki
        )
        self.u = np.clip(
            self.Ki * self.integrator + self.feedback.calculate(self.x, r),
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

    refs = []

    # Generate references for simulation
    for t in ts:
        if t < l0:
            r = np.array([[0]])
        elif t < l1:
            r = np.array([[9000 / 60 * 2 * math.pi]])
        else:
            r = np.array([[0]])
        refs.append(r)

    plt.figure(1)
    r_rec, x_rec, _, _ = fct.generate_time_responses(flywheel, refs)

    plt.ylabel("Angular velocity (rad/s)")
    plt.plot(ts, r_rec[0, :], label="Setpoint")
    plt.plot(ts, x_rec[0, :], label="Output")
    plt.legend()
    plt.xlabel("Time (s)")

    if "--noninteractive" in sys.argv:
        latex.savefig("pi_controller_ss_error_overshoot")
    else:
        plt.show()


if __name__ == "__main__":
    main()
