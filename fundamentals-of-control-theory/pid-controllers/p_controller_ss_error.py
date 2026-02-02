#!/usr/bin/env python3

"""Demonstrates P controller with steady-state error."""

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
    """An frccontrol system representing a flyhweel."""

    def __init__(self, dt):
        """
        Flywheel subsystem.

        Parameter ``dt``:
            Time between model/controller updates.
        """
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

        # States: angular velocity (rad/s)
        # Inputs: voltage (V)
        # Outputs: angular velocity (rad/s)
        self.feedback = fct.LinearQuadraticRegulator(
            self.plant.A, self.plant.B, [200.0], [12.0], self.dt
        )

        self.u_min = np.array([[-12.0]])
        self.u_max = np.array([[12.0]])

    def update(self, r, next_r):
        """
        Advance the model by one timestep.

        Parameter ``r``:
            The current reference.

        Parameter ``next_r``:
            The next reference.
        """
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

    fill_end = 1.75
    fill_end_idx = int(fill_end / dt)
    plt.fill_between(
        ts[:fill_end_idx],
        np.ravel(x_rec)[:fill_end_idx],
        np.ravel(r_rec)[:fill_end_idx],
        color=(0.5, 0.5, 0.5, 0.5),
        label="Error area for integral term",
    )
    plt.legend()
    plt.xlabel("Time (s)")

    annotate_t = fill_end + 0.25
    annotate_t_idx = int(annotate_t / dt)
    bottom = x_rec[0, annotate_t_idx]
    top = r_rec[0, annotate_t_idx]
    plt.annotate(
        "",
        xy=(annotate_t, bottom - 5),  # Start coord of arrow
        xycoords="data",
        xytext=(annotate_t, top + 5),  # End coord of arrow
        textcoords="data",
        arrowprops={"arrowstyle": "<->", "connectionstyle": "arc3,rad=0"},
        ha="center",
        va="center",
    )
    plt.annotate(
        "steady-state error",
        xy=(annotate_t + 0.125, (top + bottom) / 2.0),  # Start coord of arrow
        xycoords="data",
        ha="left",
        va="center",
    )

    if "--noninteractive" in sys.argv:
        latex.savefig("p_controller_ss_error")
    else:
        plt.show()


if __name__ == "__main__":
    main()
