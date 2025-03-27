#!/usr/bin/env python3

"""Feedforward case study."""

import sys

from cycler import cycler
import frccontrol as fct
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import StateSpace

from bookutil import latex

if "--noninteractive" in sys.argv:
    mpl.use("svg")


def main():
    """Entry point."""
    J = 7.7500e-05
    b = 8.9100e-05
    Kt = 0.0184
    Ke = 0.0211
    R = 0.0916
    L = 5.9000e-05

    A = np.array([[-b / J, Kt / J], [-Ke / L, -R / L]])
    B = np.array([[0], [1 / L]])
    C = np.array([[1, 0]])
    D = np.array([[0]])

    dt = 0.001
    tmax = 0.025

    sysd = StateSpace(A, B, C, D).to_discrete(dt)

    K = fct.LinearQuadraticRegulator(A, B, [20, float("inf")], [12], dt).K

    # Plant inversions
    Kff_ts = np.linalg.pinv(sysd.B)

    t = np.arange(0, tmax, dt)
    r = np.array([[2000 * 0.1047], [0]])
    r_rec = np.zeros((2, 1, len(t)))

    # No feedforward
    x = np.array([[0], [0]])
    x_rec = np.zeros((2, 1, len(t)))
    u_rec = np.zeros((1, 1, len(t)))

    # Plant inversion
    x_ts = np.array([[0], [0]])
    x_ts_rec = np.zeros((2, 1, len(t)))
    u_ts_rec = np.zeros((1, 1, len(t)))

    u_min = np.array([[-12.0]])
    u_max = np.array([[12.0]])

    for k in range(len(t)):
        r_rec[:, :, k] = r

        # Without feedforward
        u = K @ (r - x)
        u = np.clip(u, u_min, u_max)
        x = sysd.A @ x + sysd.B @ u
        x_rec[:, :, k] = x
        u_rec[:, :, k] = u

        # Plant inversion
        u_ts = K @ (r - x_ts) + Kff_ts @ (r - sysd.A @ r)
        u_ts = np.clip(u_ts, u_min, u_max)
        x_ts = sysd.A @ x_ts + sysd.B @ u_ts
        x_ts_rec[:, :, k] = x_ts
        u_ts_rec[:, :, k] = u_ts

    plt.figure(1)

    plt.subplot(3, 1, 1)
    plt.plot(t, r_rec[0, 0, :], label="Reference")
    plt.ylabel("Angular velocity (rad/s)")
    plt.plot(t, x_rec[0, 0, :], label="No feedforward")
    plt.plot(t, x_ts_rec[0, 0, :], label="Plant inversion")
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(t, r_rec[1, 0, :], label="Reference")
    plt.ylabel("Current (A)")
    plt.plot(t, x_rec[1, 0, :], label="No feedforward")
    plt.plot(t, x_ts_rec[1, 0, :], label="Plant inversion")
    plt.legend()

    plt.subplot(3, 1, 3)
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    plt.gca().set_prop_cycle(cycler("color", colors[1:]))
    plt.plot(t, u_rec[0, 0, :], label="No feedforward")
    plt.plot(t, u_ts_rec[0, 0, :], label="Plant inversion")
    plt.legend()
    plt.ylabel("Input (V)")
    plt.xlabel("Time (s)")

    if "--noninteractive" in sys.argv:
        latex.savefig("case_study_ff")
    else:
        plt.show()


if __name__ == "__main__":
    main()
