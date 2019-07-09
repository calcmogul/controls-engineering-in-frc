#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
    import utils.latex as latex

import control as cnt
from frccontrol import conv
import matplotlib.pyplot as plt
import numpy as np

plt.rc("text", usetex=True)


def make_closed_loop_plant(G, Kp):
    """Returns a TransferFunction representing a plant in negative feedback with
    a P controller that uses the given gain.

    Keyword arguments:
    G -- open-loop plant
    Kp -- proportional gain
    """
    K = cnt.TransferFunction(Kp, 1)
    return cnt.feedback(G, K)


def sim(tf, T, label):
    T, yout = cnt.step_response(tf, T=T)

    # 1 / yout[-1] normalizes to DC gain
    plt.plot(T, yout * 1 / yout[-1], label=label)
    plt.legend()


def main():
    dt = 0.0001
    T = np.arange(0, 6, dt)

    plt.xlabel("Time ($s$)")
    plt.ylabel("Position ($m$)")

    # Make plant
    G = cnt.TransferFunction(1, conv([1, 5], [1, 0]))

    sim(cnt.TransferFunction(1, 1), T, "Reference")
    Gcl = make_closed_loop_plant(G, 120)
    sim(Gcl, T, "Underdamped")
    Gcl = make_closed_loop_plant(G, 3)
    sim(Gcl, T, "Overdamped")
    Gcl = make_closed_loop_plant(G, 6.268)
    sim(Gcl, T, "Critically damped")

    if "--noninteractive" in sys.argv:
        latex.savefig("pid_responses")
    else:
        plt.show()


if __name__ == "__main__":
    main()
