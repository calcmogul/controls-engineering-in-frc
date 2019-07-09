#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
    import utils.latex as latex

import control as cnt
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
    T = np.arange(0, 0.25, dt)

    # Make plant
    J = 3.2284e-6  # kg-m^2
    b = 3.5077e-6  # N-m-s
    Ke = 0.0181  # V/rad/s
    Kt = 0.0181  # N-m/Amp
    K = Ke  # Ke = Kt
    R = 0.0902  # Ohms
    L = 230e-6  # H

    # Stable plant (L = 0)
    # s((Js + b)R + K^2)
    # s(JRs + bR + K^2)
    # JRs^2 + bRs + K^2s
    # JRs^2 + (bR + K^2)s
    G = cnt.TransferFunction(K, [J * R, b * R + K ** 2, 0])
    cnt.root_locus(G, grid=True)
    plt.xlabel("Real Axis (seconds$^{-1}$)")
    plt.ylabel("Imaginary Axis (seconds$^{-1}$)")
    if "--noninteractive" in sys.argv:
        latex.savefig("highfreq_stable_rlocus")

    plt.figure(2)
    plt.xlabel("Time ($s$)")
    plt.ylabel("Position ($m$)")
    sim(cnt.TransferFunction(1, 1), T, "Reference")
    Gcl = make_closed_loop_plant(G, 1)
    sim(Gcl, T, "Step response")
    if "--noninteractive" in sys.argv:
        latex.savefig("highfreq_stable_step")
    else:
        plt.show()


if __name__ == "__main__":
    main()
