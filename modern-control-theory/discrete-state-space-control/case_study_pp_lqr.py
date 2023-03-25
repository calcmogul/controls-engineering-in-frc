#!/usr/bin/env python3

"""Case study of pole placement vs LQR."""

import sys

from cycler import cycler
import frccontrol as fct
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import eig
from scipy.signal import StateSpace, place_poles

from bookutil import latex

if "--noninteractive" in sys.argv:
    mpl.use("svg")
plt.rc("text", usetex=True)


def main():
    """Entry point."""
    J = 7.7500e-05
    b = 8.9100e-05
    Kt = 0.0184
    Ke = 0.0211
    R = 0.0916
    L = 5.9000e-05

    # fmt: off
    A = np.array([[-b / J, Kt / J],
                  [-Ke / L, -R / L]])
    B = np.array([[0],
                  [1 / L]])
    C = np.array([[1, 0]])
    D = np.array([[0]])
    # fmt: on

    dt = 0.001
    tmax = 0.025

    sysd = StateSpace(A, B, C, D).to_discrete(dt)

    K_pp1 = place_poles(sysd.A, sysd.B, [0.1, 0.5]).gain_matrix
    K_pp2 = place_poles(sysd.A, sysd.B, [0.1, 0.4]).gain_matrix
    K_lqr = fct.LinearQuadraticRegulator(A, B, [20, float("inf")], [12], dt).K

    poles = eig(sysd.A - sysd.B @ K_pp1)[0]
    poles_pp1 = f"{np.round(poles[0], 3)} and {np.round(poles[1], 3)}"

    poles = eig(sysd.A - sysd.B @ K_pp2)[0]
    poles_pp2 = f"{np.round(poles[0], 3)} and {np.round(poles[1], 3)}"

    poles = eig(sysd.A - sysd.B @ K_lqr)[0]
    poles_lqr = f"{np.round(poles[0], 3)} and {np.round(poles[1], 3)}"

    t = np.arange(0, tmax, dt)
    r = np.array([[2000 * 0.1047], [0]])
    r_rec = np.zeros((2, 1, len(t)))

    # Pole placement 1
    x_pp1 = np.array([[0], [0]])
    x_pp1_rec = np.zeros((2, 1, len(t)))
    u_pp1_rec = np.zeros((1, 1, len(t)))

    # Pole placement 2
    x_pp2 = np.array([[0], [0]])
    x_pp2_rec = np.zeros((2, 1, len(t)))
    u_pp2_rec = np.zeros((1, 1, len(t)))

    # LQR
    x_lqr = np.array([[0], [0]])
    x_lqr_rec = np.zeros((2, 1, len(t)))
    u_lqr_rec = np.zeros((1, 1, len(t)))

    u_min = np.array([[-12.0]])
    u_max = np.array([[12.0]])

    for k in range(len(t)):
        # Pole placement 1
        u_pp1 = K_pp1 @ (r - x_pp1)

        # Pole placement 2
        u_pp2 = K_pp2 @ (r - x_pp2)

        # LQR
        u_lqr = K_lqr @ (r - x_lqr)

        u_pp1 = np.clip(u_pp1, u_min, u_max)
        x_pp1 = sysd.A @ x_pp1 + sysd.B @ u_pp1
        u_pp2 = np.clip(u_pp2, u_min, u_max)
        x_pp2 = sysd.A @ x_pp2 + sysd.B @ u_pp2
        u_lqr = np.clip(u_lqr, u_min, u_max)
        x_lqr = sysd.A @ x_lqr + sysd.B @ u_lqr

        r_rec[:, :, k] = r
        x_pp1_rec[:, :, k] = x_pp1
        u_pp1_rec[:, :, k] = u_pp1
        x_pp2_rec[:, :, k] = x_pp2
        u_pp2_rec[:, :, k] = u_pp2
        x_lqr_rec[:, :, k] = x_lqr
        u_lqr_rec[:, :, k] = u_lqr

    plt.figure(1)

    plt.subplot(3, 1, 1)
    plt.plot(t, r_rec[0, 0, :], label="Reference")
    plt.ylabel(r"$\omega$ (rad/s)")
    plt.plot(t, x_pp1_rec[0, 0, :], label=f"Pole placement at {poles_pp1}")
    plt.plot(t, x_pp2_rec[0, 0, :], label=f"Pole placement at {poles_pp2}")
    plt.plot(t, x_lqr_rec[0, 0, :], label=f"LQR at {poles_lqr}")
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(t, r_rec[1, 0, :], label="Reference")
    plt.ylabel("Current (A)")
    plt.plot(t, x_pp1_rec[1, 0, :], label=f"Pole placement at {poles_pp1}")
    plt.plot(t, x_pp2_rec[1, 0, :], label=f"Pole placement at {poles_pp2}")
    plt.plot(t, x_lqr_rec[1, 0, :], label=f"LQR at {poles_lqr}")
    plt.legend()

    plt.subplot(3, 1, 3)
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    plt.gca().set_prop_cycle(cycler("color", colors[1:]))
    plt.plot(t, u_pp1_rec[0, 0, :], label=f"Pole placement at {poles_pp1}")
    plt.plot(t, u_pp2_rec[0, 0, :], label=f"Pole placement at {poles_pp2}")
    plt.plot(t, u_lqr_rec[0, 0, :], label=f"LQR at {poles_lqr}")
    plt.legend()
    plt.ylabel("Input (V)")
    plt.xlabel("Time (s)")

    if "--noninteractive" in sys.argv:
        latex.savefig("case_study_pp_lqr")
    else:
        plt.show()


if __name__ == "__main__":
    main()
