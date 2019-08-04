#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
    import utils.latex as latex

import control as ct
from frccontrol import conv
import matplotlib.pyplot as plt
import numpy as np

plt.rc("text", usetex=True)


def sim(tf, T, label):
    T, yout = ct.step_response(tf, T=T)

    # 1 / yout[-1] normalizes to DC gain
    plt.plot(T, yout * 1 / yout[-1], label=label)
    plt.legend()


def main():
    dt = 0.0001
    T = np.arange(0, 6, dt)

    plt.xlabel("Time ($s$)")
    plt.ylabel("Position ($m$)")

    # Make plant
    G = ct.TransferFunction(1, conv([1, 5], [1, 0]))

    sim(ct.TransferFunction(1, 1), T, "Setpoint")

    K = ct.TransferFunction(120, 1)
    Gcl = ct.feedback(G, K)
    sim(Gcl, T, "Underdamped")

    K = ct.TransferFunction(3, 1)
    Gcl = ct.feedback(G, K)
    sim(Gcl, T, "Overdamped")

    K = ct.TransferFunction(6.268, 1)
    Gcl = ct.feedback(G, K)
    sim(Gcl, T, "Critically damped")

    if "--noninteractive" in sys.argv:
        latex.savefig("pid_responses")
    else:
        plt.show()


if __name__ == "__main__":
    main()
