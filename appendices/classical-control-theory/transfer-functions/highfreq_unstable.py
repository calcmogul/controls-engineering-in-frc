#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
    import bookutil.latex as latex

import control as ct
import matplotlib.pyplot as plt
import numpy as np

plt.rc("text", usetex=True)


def main():
    dt = 0.0001
    T = np.arange(0, 0.25, dt)

    # Make plant
    J = 3.2284e-6  # kg-m²
    b = 3.5077e-6  # N-m-s
    Ke = 0.0181  # V/rad/s
    Kt = 0.0181  # N-m/Amp
    R = 0.0902  # Ohms
    L = 230e-6  # H

    # Transfer function
    #
    # s((Js + b)(Ls + R) + Kt Ke)
    # s(JLs² + JRs + bLs + bR + Kt Ke)
    # JLs³ + JRs² + bLs² + bRs + Kt Ke s
    # JLs³ + (JR + bL)s² + (bR + Kt Ke)s
    G = ct.TransferFunction(Ke, [J * L, J * R + b * L, b * R + Kt * Ke, 0])

    ct.root_locus(G, grid=True)
    plt.xlabel("Real Axis (seconds$^{-1}$)")
    plt.ylabel("Imaginary Axis (seconds$^{-1}$)")
    if "--noninteractive" in sys.argv:
        latex.savefig("highfreq_unstable_rlocus")

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
    system = ct.StateSpace(A - B @ K, B @ K, C - D @ K, D @ K)

    plt.figure(2)
    plt.xlabel("Time ($s$)")
    plt.ylabel("Position ($m$)")

    plt.plot(T, [1 for t in T], label="Reference")
    plt.legend()

    _, yout = ct.step_response(system, T=T, input=0, output=0)
    plt.plot(T, yout, label="Step response")
    plt.legend()

    if "--noninteractive" in sys.argv:
        latex.savefig("highfreq_unstable_step")
    else:
        plt.show()


if __name__ == "__main__":
    main()
