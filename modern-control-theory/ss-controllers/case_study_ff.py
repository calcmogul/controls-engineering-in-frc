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
    K = fct.lqr(sysd, Q, R)

    # Plant inversions
    Kff_ts1 = np.linalg.pinv(sysd.B)
    Kff_ts2 = np.linalg.inv(sysd.B.T * Q * sysd.B + R) * (sysd.B.T * Q)

    t = np.arange(0, tmax, dt)
    r = np.array([[2000 * 0.1047], [0]])
    r_rec = np.zeros((2, 1, len(t)))

    # No feedforward
    x = np.array([[0], [0]])
    x_rec = np.zeros((2, 1, len(t)))
    u_rec = np.zeros((1, 1, len(t)))

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

        # Without feedforward
        u = K @ (r - x)
        u = np.clip(u, u_min, u_max)
        x = sysd.A @ x + sysd.B @ u
        x_rec[:, :, k] = x
        u_rec[:, :, k] = u

        # Plant inversion
        u_ts1 = K @ (r - x_ts1) + Kff_ts1 @ (r - sysd.A @ r)
        u_ts1 = np.clip(u_ts1, u_min, u_max)
        x_ts1 = sysd.A @ x_ts1 + sysd.B @ u_ts1
        x_ts1_rec[:, :, k] = x_ts1
        u_ts1_rec[:, :, k] = u_ts1

    plt.figure(1)

    plt.subplot(3, 1, 1)
    plt.plot(t, r_rec[0, 0, :], label="Reference")
    plt.ylabel("$\omega$ (rad/s)")
    plt.plot(t, x_rec[0, 0, :], label="No feedforward")
    plt.plot(t, x_ts1_rec[0, 0, :], label="Plant inversion")
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(t, r_rec[1, 0, :], label="Reference")
    plt.ylabel("Current (A)")
    plt.plot(t, x_rec[1, 0, :], label="No feedforward")
    plt.plot(t, x_ts1_rec[1, 0, :], label="Plant inversion")
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(t, u_rec[0, 0, :], label="No feedforward")
    plt.plot(t, u_ts1_rec[0, 0, :], label="Plant inversion")
    plt.legend()
    plt.ylabel("Control effort (V)")
    plt.xlabel("Time (s)")

    if "--noninteractive" in sys.argv:
        latex.savefig("case_study_ff")
    else:
        plt.show()


if __name__ == "__main__":
    main()
