#!/usr/bin/env python3

"""Plots time domain response of a system that's unstable at high frequencies."""

import sys

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import StateSpace

from bookutil import latex

if "--noninteractive" in sys.argv:
    mpl.use("svg")
plt.rc("text", usetex=True)


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

    # State-space
    #
    # From sum of forces:
    #
    # J ddot{θ} + b dot{θ} = Kt i           (1)
    # ddot{θ} + b/J dot{θ} = Kt/J i
    # ddot{θ} = -b/J dot{θ} + Kt/J i
    # ddot{θ} = -b/J dot{θ} + Kt/J i + 0 V
    #
    # From Kirchoff's voltage law:
    #
    # L di/dt + R i = V - Ke dot{θ}         (2)
    # di/dt + R/L i = V/L - Ke/L dot{θ}
    # di/dt = -R/L i + V/L - Ke/L dot{θ}
    # di/dt = -R/L i + 1/L V - Ke/L dot{θ}
    # di/dt = -Ke/L dot{θ} - R/L i + 1/L V
    A = np.array([[0, 1, 0], [0, -b / J, Kt / J], [0, -Ke / L, -R / L]])
    B = np.array([[0], [0], [1 / L]])
    C = np.array([[1, 0, 0]])
    D = np.array([[0]])
    K = np.array([[1, 0, 0]])
    system = StateSpace(A - B @ K, B @ K, C - D @ K, D @ K).to_discrete(dt)

    plt.figure(2)
    plt.xlabel("Time ($s$)")
    plt.ylabel("Position ($m$)")

    x = np.array([[0], [0], [0]])
    u = np.array([[1], [0], [0]])
    Ts = [0]
    y = system.C @ x + system.D @ u
    ys = [y[0, 0]]
    t = 0
    while t < 0.25:
        x = system.A @ x + system.B @ u
        y = system.C @ x + system.D @ u

        Ts.append(t)
        ys.append(y[0, 0])

        t += dt

    plt.plot(Ts, [1 for t in Ts], label="Reference")
    plt.legend()

    plt.plot(Ts, ys, label="Step response")
    plt.legend()

    if "--noninteractive" in sys.argv:
        latex.savefig("highfreq_unstable_step")
    else:
        plt.show()


if __name__ == "__main__":
    main()
