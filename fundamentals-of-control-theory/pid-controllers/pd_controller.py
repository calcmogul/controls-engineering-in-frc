#!/usr/bin/env python3

"""Demonstrates PD controller."""

import sys

import frccontrol as fct
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

from bookutil import latex
from bookutil.systems import Elevator

if "--noninteractive" in sys.argv:
    mpl.use("svg")
plt.rc("text", usetex=True)


class ElevatorNoFeedforward(Elevator):
    """frccontrol system for elevator without feedforward."""

    def __init__(self, dt):
        """Elevator subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        """
        Elevator.__init__(self, dt)

    def design_controller_observer(self):
        self.design_lqr([0.02, 0.4], [12.0])

        q_pos = 0.05
        q_vel = 1.0
        r_pos = 0.0001
        self.design_kalman_filter([q_pos, q_vel], [r_pos])


def main():
    """Entry point."""
    dt = 0.005
    elevator = ElevatorNoFeedforward(dt)

    # Set up graphing
    l0 = 0.1
    l1 = l0 + 5.0
    l2 = l1 + 0.1
    ts = np.arange(0, l2 + 5.0, dt)

    ts, xprof, vprof, _ = fct.generate_trapezoid_profile(
        max_v=0.5, time_to_max_v=0.5, dt=dt, goal=3
    )

    refs = []

    # Generate references for simulation
    for i, _ in enumerate(ts):
        r = np.array([[xprof[i]], [vprof[i]]])
        refs.append(r)

    x_rec, ref_rec, u_rec, _ = elevator.generate_time_responses(refs)
    latex.plot_time_responses(
        elevator, ts, x_rec, ref_rec, u_rec, 2, use_pid_labels=True
    )

    if "--noninteractive" in sys.argv:
        latex.savefig("pd_controller")
    else:
        plt.show()


if __name__ == "__main__":
    main()
