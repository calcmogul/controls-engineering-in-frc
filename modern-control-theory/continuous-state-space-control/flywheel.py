#!/usr/bin/env python3

"""Plots flywheel step response."""

import math
import sys

import frccontrol as fct
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

from bookutil import latex

if "--noninteractive" in sys.argv:
    mpl.use("svg")


class Flywheel(fct.System):
    """An frccontrol system for a flywheel."""

    def __init__(self, dt):
        """Flywheel subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        """
        state_labels = [("Angular velocity", "rad/s")]
        u_labels = [("Voltage", "V")]
        self.set_plot_labels(state_labels, u_labels)

        fct.System.__init__(
            self,
            np.array([[-12.0]]),
            np.array([[12.0]]),
            dt,
            np.zeros((1, 1)),
            np.zeros((1, 1)),
        )

    # pragma pylint: disable=signature-differs
    def create_model(self, states, inputs):
        # Number of motors
        num_motors = 1.0
        # Flywheel moment of inertia in kg-m^2
        J = 0.00032
        # Gear ratio
        G = 12.0 / 18.0

        return fct.models.flywheel(fct.models.MOTOR_775PRO, num_motors, J, G)

    def design_controller_observer(self):
        q = [9.42]
        r = [12.0]
        self.design_lqr(q, r)
        # self.place_controller_poles([0.87])
        self.design_two_state_feedforward()

        q_vel = 1.0
        r_vel = 0.01
        self.design_kalman_filter([q_vel], [r_vel])
        # self.place_observer_poles([0.3])


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

    x_rec, ref_rec, u_rec, _ = flywheel.generate_time_responses(refs)
    flywheel.plot_time_responses(ts, x_rec, ref_rec, u_rec)
    if "--noninteractive" in sys.argv:
        latex.savefig("flywheel_response")
    else:
        plt.show()


if __name__ == "__main__":
    main()
