#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
    import utils.latex as latex

import control as ct
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
    K = ct.TransferFunction(Kp, 1)
    return ct.feedback(G, K)


def sim(tf, T, label):
    T, yout = ct.step_response(tf, T=T)

    # 1 / yout[-1] normalizes to DC gain
    plt.plot(T, yout * 1 / yout[-1], label=label)
    plt.legend()


def main():
    dt = 0.0001
    T = np.arange(0, 0.25, dt)

    # Make plant
    J = 3.2284e-6  # kg-m^2
    b = 3.5077e-6  # N-m-s
    Ke = 0.0181  # V/rad/s
    Kt = 0.0181  # N-m/Amp
    K = Ke  # Ke = Kt
    R = 0.0902  # Ohms
    L = 230e-6  # H

    # Unstable plant
    # s((Js + b)(Ls + R) + K^2)
    # s(JLs^2 + JRs + bLs + bR + K^2)
    # JLs^3 + JRs^2 + bLs^2 + bRs + K^2s
    # JLs^3 + (JR + bL)s^2 + (bR + K^2)s
    G = ct.TransferFunction(K, [J * L, J * R + b * L, b * R + K ** 2, 0])
    ct.root_locus(G, grid=True)
    plt.xlabel("Real Axis (seconds$^{-1}$)")
    plt.ylabel("Imaginary Axis (seconds$^{-1}$)")
    if "--noninteractive" in sys.argv:
        latex.savefig("highfreq_unstable_rlocus")

    plt.figure(2)
    plt.xlabel("Time ($s$)")
    plt.ylabel("Position ($m$)")
    sim(ct.TransferFunction(1, 1), T, "Reference")
    Gcl = make_closed_loop_plant(G, 1)
    sim(Gcl, T, "Step response")
    if "--noninteractive" in sys.argv:
        latex.savefig("highfreq_unstable_step")
    else:
        plt.show()


if __name__ == "__main__":
    main()
