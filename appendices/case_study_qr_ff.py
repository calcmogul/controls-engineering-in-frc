#!/usr/bin/env python3

"""Case study for QR-weighted plant inversion feedforward."""

import sys

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

    dt = 0.0001
    tmax = 0.025

    sysd = StateSpace(A, B, C, D).to_discrete(dt)

    Q = fct.make_cost_matrix([20, 40])
    R = fct.make_cost_matrix([12])
    K = fct.LinearQuadraticRegulator(A, B, [20, 40], [12], dt).K

    # Plant inversions
    Kff_ts1 = np.linalg.pinv(sysd.B)
    Kff_ts2 = np.linalg.solve(sysd.B.T @ Q @ sysd.B + R, sysd.B.T @ Q)

    t = np.arange(0, tmax, dt)
    r = np.array([[2000 * 0.1047], [0]])
    r_rec = np.zeros((2, 1, len(t)))

    # Plant inversion
    x_ts1 = np.array([[0], [0]])
    x_ts1_rec = np.zeros((2, 1, len(t)))
    u_ts1_rec = np.zeros((1, 1, len(t)))

    # Plant inversion (Q and R cost)
    x_ts2 = np.array([[0], [0]])
    x_ts2_rec = np.zeros((2, 1, len(t)))
    u_ts2_rec = np.zeros((1, 1, len(t)))

    u_min = np.array([[-12.0]])
    u_max = np.array([[12.0]])

    for k in range(len(t)):
        r_rec[:, :, k] = r

        # Plant inversion
        u_ts1 = K @ (r - x_ts1) + Kff_ts1 @ (r - sysd.A @ r)
        u_ts1 = np.clip(u_ts1, u_min, u_max)
        x_ts1 = sysd.A @ x_ts1 + sysd.B @ u_ts1
        x_ts1_rec[:, :, k] = x_ts1
        u_ts1_rec[:, :, k] = u_ts1

        # Plant inversion (Q and R cost)
        u_ts2 = K @ (r - x_ts2) + Kff_ts2 @ (r - sysd.A @ r)
        u_ts2 = np.clip(u_ts2, u_min, u_max)
        x_ts2 = sysd.A @ x_ts2 + sysd.B @ u_ts2
        x_ts2_rec[:, :, k] = x_ts2
        u_ts2_rec[:, :, k] = u_ts2

    plt.figure(1)

    plt.subplot(3, 1, 1)
    plt.plot(t, r_rec[0, 0, :], label="Reference")
    plt.ylabel("Angular velocity (rad/s)")
    plt.plot(t, x_ts1_rec[0, 0, :], label="Plant inversion")
    plt.plot(t, x_ts2_rec[0, 0, :], label="Plant inversion (Q and R cost)")
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(t, r_rec[1, 0, :], label="Reference")
    plt.ylabel("Current (A)")
    plt.plot(t, x_ts1_rec[1, 0, :], label="Plant inversion")
    plt.plot(t, x_ts2_rec[1, 0, :], label="Plant inversion (Q and R cost)")
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(t, u_ts1_rec[0, 0, :], label="Plant inversion")
    plt.plot(t, u_ts2_rec[0, 0, :], label="Plant inversion (Q and R cost)")
    plt.legend()
    plt.ylabel("Input (V)")
    plt.xlabel("Time (s)")

    if "--noninteractive" in sys.argv:
        latex.savefig("case_study_qr_ff")
    else:
        plt.show()


if __name__ == "__main__":
    main()
