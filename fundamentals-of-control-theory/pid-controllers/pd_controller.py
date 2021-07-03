#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
import bookutil.latex as latex

import frccontrol as fct
import matplotlib.pyplot as plt
import numpy as np

from bookutil.systems import Elevator

plt.rc("text", usetex=True)


class ElevatorNoFeedforward(Elevator):
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
    dt = 0.005
    elevator = ElevatorNoFeedforward(dt)

    # Set up graphing
    l0 = 0.1
    l1 = l0 + 5.0
    l2 = l1 + 0.1
    t = np.arange(0, l2 + 5.0, dt)

    t, xprof, vprof, aprof = fct.generate_trapezoid_profile(
        max_v=0.5, time_to_max_v=0.5, dt=dt, goal=3
    )

    refs = []

    # Generate references for simulation
    for i in range(len(t)):
        r = np.array([[xprof[i]], [vprof[i]]])
        refs.append(r)

    x_rec, ref_rec, u_rec, y_rec = elevator.generate_time_responses(t, refs)
    latex.plot_time_responses(
        elevator, t, x_rec, ref_rec, u_rec, 2, use_pid_labels=True
    )

    if "--noninteractive" in sys.argv:
        latex.savefig("pd_controller")
    else:
        plt.show()


if __name__ == "__main__":
    main()
