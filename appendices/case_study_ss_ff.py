#!/usr/bin/env python3

"""Case study for steady-state feedforward."""

import sys

import frccontrol as fct
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

    sysc = StateSpace(A, B, C, D)

    dt = 0.0001
    tmax = 0.025

    sysd = sysc.to_discrete(dt)

    # fmt: off
    Q = np.diag(1.0 / np.square([20, 40]))
    R = np.diag(1.0 / np.square([12]))
    # fmt: on
    K = fct.lqr(sysd, Q, R)

    # Steady-state feedforward
    tmp1 = np.block([[sysd.A - np.eye(sysd.A.shape[0]), sysd.B], [sysd.C, sysd.D]])
    tmp2 = np.block([[np.zeros((sysd.A.shape[0], 1))], [np.ones((sysd.C.shape[0], 1))]])
    NxNu = np.linalg.solve(tmp1, tmp2)
    Nx = NxNu[0 : sysd.A.shape[0], 0:1]
    Nu = NxNu[sysd.C.shape[0] + 1 :, 0:1]
    Kff_ss = Nu @ np.linalg.pinv(Nx)

    # Plant inversions
    Kff_ts1 = np.linalg.solve(sysd.B.T @ Q @ sysd.B + R, sysd.B.T @ Q)
    Kff_ts2 = np.linalg.pinv(sysd.B)

    t = np.arange(0, tmax, dt)
    r = np.array([[2000 * 0.1047], [0]])
    r_rec = np.zeros((2, 1, len(t)))

    # No feedforward
    x = np.array([[0], [0]])
    x_rec = np.zeros((2, 1, len(t)))
    u_rec = np.zeros((1, 1, len(t)))

    # Steady-state feedforward
    x_ss = np.array([[0], [0]])
    x_ss_rec = np.zeros((2, 1, len(t)))
    u_ss_rec = np.zeros((1, 1, len(t)))

    # Plant inversion (Q and R cost)
    x_ts1 = np.array([[0], [0]])
    x_ts1_rec = np.zeros((2, 1, len(t)))
    u_ts1_rec = np.zeros((1, 1, len(t)))

    # Plant inversion (Q cost only)
    x_ts2 = np.array([[0], [0]])
    x_ts2_rec = np.zeros((2, 1, len(t)))
    u_ts2_rec = np.zeros((1, 1, len(t)))

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

        # With steady-state feedforward
        u_ss = K @ (r - x_ss) + Kff_ss @ r
        u_ss = np.clip(u_ss, u_min, u_max)
        x_ss = sysd.A @ x_ss + sysd.B @ u_ss
        x_ss_rec[:, :, k] = x_ss
        u_ss_rec[:, :, k] = u_ss

        # Plant inversion (Q and R cost)
        u_ts1 = K @ (r - x_ts1) + Kff_ts1 @ (r - sysd.A @ r)
        u_ts1 = np.clip(u_ts1, u_min, u_max)
        x_ts1 = sysd.A @ x_ts1 + sysd.B @ u_ts1
        x_ts1_rec[:, :, k] = x_ts1
        u_ts1_rec[:, :, k] = u_ts1

        # Plant inversion
        u_ts2 = K @ (r - x_ts2) + Kff_ts2 @ (r - sysd.A @ r)
        u_ts2 = np.clip(u_ts2, u_min, u_max)
        x_ts2 = sysd.A @ x_ts2 + sysd.B @ u_ts2
        x_ts2_rec[:, :, k] = x_ts2
        u_ts2_rec[:, :, k] = u_ts2

    plt.figure(1)

    plt.subplot(3, 1, 1)
    plt.plot(t, r_rec[0, 0, :], label="Reference")
    plt.ylabel(r"$\omega$ (rad/s)")
    plt.plot(t, x_rec[0, 0, :], label="No FF")
    plt.plot(t, x_ss_rec[0, 0, :], label="Steady-state FF")
    plt.plot(t, x_ts1_rec[0, 0, :], label="Plant inversion (Q and R cost)")
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(t, r_rec[1, 0, :], label="Reference")
    plt.ylabel("Current (A)")
    plt.plot(t, x_rec[1, 0, :], label="No FF")
    plt.plot(t, x_ss_rec[1, 0, :], label="Steady-state FF")
    plt.plot(t, x_ts1_rec[1, 0, :], label="Plant inversion (Q and R cost)")
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(t, u_rec[0, 0, :], label="No FF")
    plt.plot(t, u_ss_rec[0, 0, :], label="Steady-state FF")
    plt.plot(t, u_ts1_rec[0, 0, :], label="Plant inversion (Q and R cost)")
    plt.legend()
    plt.ylabel("Control effort (V)")
    plt.xlabel("Time (s)")

    if "--noninteractive" in sys.argv:
        latex.savefig("case_study_ss_ff1")

    plt.figure(2)

    plt.subplot(3, 1, 1)
    plt.plot(t, r_rec[0, 0, :], label="Reference")
    plt.ylabel(r"$\omega$ (rad/s)")
    plt.plot(t, x_ts1_rec[0, 0, :], label="Plant inversion (Q and R cost)")
    plt.plot(t, x_ts2_rec[0, 0, :], label="Plant inversion (Q cost only)")
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(t, r_rec[1, 0, :], label="Reference")
    plt.ylabel("Current (A)")
    plt.plot(t, x_ts1_rec[1, 0, :], label="Plant inversion (Q and R cost)")
    plt.plot(t, x_ts2_rec[1, 0, :], label="Plant inversion (Q cost only)")
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(t, u_ts1_rec[0, 0, :], label="Plant inversion (Q and R cost)")
    plt.plot(t, u_ts2_rec[0, 0, :], label="Plant inversion (Q cost only)")
    plt.legend()
    plt.ylabel("Control effort (V)")
    plt.xlabel("Time (s)")

    if "--noninteractive" in sys.argv:
        latex.savefig("case_study_ss_ff2")
    else:
        plt.show()


if __name__ == "__main__":
    main()
