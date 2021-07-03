#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
    import bookutil.latex as latex

import math
import matplotlib.pyplot as plt
import numpy as np

from bookutil.systems import Flywheel

plt.rc("text", usetex=True)


class FlywheelIntegrator(Flywheel):
    def __init__(self, dt):
        """Flywheel subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        """
        self.integrator = 0.0
        self.Ki = 1e-2

        Flywheel.__init__(self, dt)

    def update_controller(self, next_r):
        """Advance the controller by one timestep.

        Keyword arguments:
        next_r -- next controller reference (default: current reference)
        """
        e = (self.r - self.x_hat)[0, 0]
        self.integrator = np.clip(
            self.integrator + e * self.dt, self.u_min / self.Ki, self.u_max / self.Ki
        )
        u = self.K @ (self.r - self.x_hat) + self.Ki * self.integrator
        self.r = next_r
        self.u = np.clip(u, self.u_min, self.u_max)


def main():
    dt = 0.005
    flywheel = FlywheelIntegrator(dt)

    # Set up graphing
    l0 = 0.1
    l1 = l0 + 5.0
    l2 = l1 + 0.1
    t = np.arange(0, l2 + 5.0, dt)

    refs = []

    # Generate references for simulation
    for i in range(len(t)):
        if t[i] < l0:
            r = np.array([[0]])
        elif t[i] < l1:
            r = np.array([[9000 / 60 * 2 * math.pi]])
        else:
            r = np.array([[0]])
        refs.append(r)

    plt.figure(1)
    x_rec, ref_rec, u_rec, y_rec = flywheel.generate_time_responses(t, refs)

    plt.ylabel(flywheel.state_labels[0])
    plt.plot(t, flywheel.extract_row(x_rec, 0), label="Output")
    plt.plot(t, flywheel.extract_row(ref_rec, 0), label="Setpoint")
    plt.legend()
    plt.xlabel("Time (s)")

    if "--noninteractive" in sys.argv:
        latex.savefig("pi_controller_ss_error")
    else:
        plt.show()


if __name__ == "__main__":
    main()
