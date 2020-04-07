#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
    import bookutil.latex as latex

import control as ct
import frccontrol as fct
import matplotlib.pyplot as plt
import numpy as np
from numpy import concatenate

plt.rc("text", usetex=True)


def main():
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

    sysc = ct.StateSpace(A, B, C, D)

    dt = 0.0001
    tmax = 0.025

    sysd = sysc.sample(dt)

    # fmt: off
    Q = np.array([[1 / 20**2, 0],
                  [0, 1 / 40**2]])
    R = np.array([[1 / 12**2]])
    # fmt: on
    K_pp1 = ct.place(sysd.A, sysd.B, [0.1, 0.9])
    K_pp2 = ct.place(sysd.A, sysd.B, [0.1, 0.8])
    K_lqr = fct.lqr(sysd, Q, R)

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

    u_min = np.asarray(-12)
    u_max = np.asarray(12)

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
    plt.ylabel("$\omega$ (rad/s)")
    plt.plot(t, x_pp1_rec[0, 0, :], label="Pole placement at $(0.1, 0)$ and $(0.9, 0)$")
    plt.plot(t, x_pp2_rec[0, 0, :], label="Pole placement at $(0.1, 0)$ and $(0.8, 0)$")
    plt.plot(t, x_lqr_rec[0, 0, :], label="LQR")
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(t, r_rec[1, 0, :], label="Reference")
    plt.ylabel("Current (A)")
    plt.plot(t, x_pp1_rec[1, 0, :], label="Pole placement at $(0.1, 0)$ and $(0.9, 0)$")
    plt.plot(t, x_pp2_rec[1, 0, :], label="Pole placement at $(0.1, 0)$ and $(0.8, 0)$")
    plt.plot(t, x_lqr_rec[1, 0, :], label="LQR")
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(t, u_pp1_rec[0, 0, :], label="Pole placement at $(0.1, 0)$ and $(0.9, 0)$")
    plt.plot(t, u_pp2_rec[0, 0, :], label="Pole placement at $(0.1, 0)$ and $(0.8, 0)$")
    plt.plot(t, u_lqr_rec[0, 0, :], label="LQR")
    plt.legend()
    plt.ylabel("Control effort (V)")
    plt.xlabel("Time (s)")

    if "--noninteractive" in sys.argv:
        latex.savefig("case_study_pp_lqr")
    else:
        plt.show()


if __name__ == "__main__":
    main()
