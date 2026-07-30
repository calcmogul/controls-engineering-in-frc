#!/usr/bin/env python

"""Plots time domain response of a first-order CIM motor model."""

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

    # From sum of forces:
    #
    #   Jα + bω = Kₜi                        (1)
    #
    # From Kirchoff's voltage law:
    #
    #   Ri = V - Kₑω                         (2)
    #   i = V/R - Kₑ/R ω
    #
    # Plug (2) into (1).
    #
    #   Jα + bω = Kₜ(V/R - KₜKₑ/R ω)
    #   Jα + bω = KₜV/R - KₜKₑ/R ω
    #   Jα = KₜV/R - KₜKₑ/R ω - bω
    #   α = KₜV/(RJ) - KₜKₑ/(RJ) ω - b/J ω
    #   α = Kₜ/(RJ) V - (KₜKₑ/(RJ) + b/J) ω
    #   α = -(KₜKₑ/(RJ) + b/J)ω + Kt/(RJ) V
    A = np.array([[-(Kt * Ke / (R * J) + b / J)]])
    B = np.array([[Kt / (R * J)]])
    C = np.array([[1]])
    D = np.array([[0]])
    system = StateSpace(A, B, C, D).to_discrete(dt)

    plt.figure(2)
    plt.xlabel("Time (ms)")
    plt.ylabel("Angular velocity (rad/s)")

    r = np.array([[1]])
    x = np.array([[0]])
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
        latex.savefig("cim_first_order_step")
    else:
        plt.show()


if __name__ == "__main__":
    main()
