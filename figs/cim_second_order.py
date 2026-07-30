#!/usr/bin/env python

"""Plots time domain response of a second-order CIM motor model."""

import sys

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import StateSpace

from bookutil import latex

if "--noninteractive" in sys.argv:
    mpl.use("svg")


def main():
    """Entry point."""
    dt = 0.0001

    # Make plant
    J = 3.2284e-6  # kg-m²
    b = 3.5077e-6  # N-m-s
    Ke = 0.0181  # V/rad/s
    Kt = 0.0181  # N-m/A
    R = 0.0902  # Ω
    L = 230e-6  # H

    # From sum of forces:
    #
    #   Jα + bω = Kₜi                    (1)
    #   α + b/J ω = Kₜ/J i
    #   α = -b/J ω + Kₜ/J i
    #   α = -b/J ω + Kₜ/J i + 0 V
    #
    # From Kirchoff's voltage law:
    #
    #   L di/dt + Ri = V - Kₑω           (2)
    #   di/dt + R/L i = V/L - Kₑ/L ω
    #   di/dt = -R/L i + V/L - Kₑ/L ω
    #   di/dt = -R/L i + 1/L V - Kₑ/L ω
    #   di/dt = -Kₑ/L ω - R/L i + 1/L V
    A = np.array([[-b / J, Kt / J], [-Ke / L, -R / L]])
    B = np.array([[0], [1 / L]])
    C = np.array([[1, 0]])
    D = np.array([[0]])
    system = StateSpace(A, B, C, D).to_discrete(dt)

    plt.figure(2)
    plt.xlabel("Time (ms)")
    plt.ylabel("Angular velocity (rad/s)")

    r = np.array([[1], [0]])
    x = np.array([[0], [0]])
    xs = [x[0, 0]]
    ts = [0]
    t = 0
    while t < 0.05:
        u = np.linalg.pinv(system.B) @ (r - system.A @ r)
        x = system.A @ x + system.B @ u

        ts.append(t)
        xs.append(x[0, 0])

        t += dt

    plt.plot([t * 1e3 for t in ts], [r[0, 0] for t in ts], label="Reference")
    plt.legend()

    plt.plot([t * 1e3 for t in ts], xs, label="Step response")
    plt.legend()

    if "--noninteractive" in sys.argv:
        latex.savefig("cim_second_order_step")
    else:
        plt.show()


if __name__ == "__main__":
    main()
