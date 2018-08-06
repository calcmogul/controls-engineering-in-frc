#!/usr/bin/env python3

import matplotlib as mpl
mpl.use("svg")
import control as cnt
import frccontrol as frccnt
import matplotlib.pyplot as plt
import numpy as np
from numpy import concatenate

import latexutils

plt.rc("text", usetex=True)


def main():
    J = 7.7500e-05
    b = 8.9100e-05
    Kt = 0.0184
    Ke = 0.0211
    R = 0.0916
    L = 5.9000e-05

    # yapf: disable
    A = np.matrix([[-b / J, Kt / J],
                   [-Ke / L, -R / L]])
    B = np.matrix([[0],
                   [1 / L]])
    C = np.matrix([[1, 0]])
    D = np.matrix([[0]])
    # yapf: enable

    sys = cnt.StateSpace(A, B, C, D)

    dt = 0.0001
    tmax = 0.025

    sysd = sys.sample(dt)

    # yapf: disable
    Q = np.matrix([[1 / 20**2, 0],
                   [0, 1 / 40**2]])
    R = np.matrix([[1 / 12**2]])
    # yapf: enable
    K = frccnt.dlqr(sysd, Q, R)

    # Steady-state feedforward
    tmp1 = concatenate(
        (concatenate((sysd.A - np.eye(sysd.A.shape[0]), sysd.B), axis=1),
         concatenate((sysd.C, sysd.D), axis=1)),
        axis=0)
    tmp2 = concatenate(
        (np.zeros((sysd.A.shape[0], 1)), np.ones((sysd.C.shape[0], 1))), axis=0)
    NxNu = np.linalg.pinv(tmp1) * tmp2
    Nx = NxNu[0:sysd.A.shape[0], 0]
    Nu = NxNu[sysd.C.shape[0] + 1:, 0]
    Kff1 = Nu * np.linalg.pinv(Nx)

    # Two-state feedforwards
    Kff2 = np.linalg.inv(sysd.B.T * Q * sysd.B + R) * (sysd.B.T * Q)
    Kff3 = np.linalg.inv(sysd.B.T * Q * sysd.B) * (sysd.B.T * Q)

    t = np.arange(0, tmax, dt)
    r = np.matrix([[2000 * 0.1047], [0]])
    r_rec = np.zeros((2, 1, len(t)))

    # No feedforward
    x0 = np.matrix([[0], [0]])
    x0_rec = np.zeros((2, 1, len(t)))
    u0_rec = np.zeros((1, 1, len(t)))

    # Steady-state feedforward
    x1 = np.matrix([[0], [0]])
    x1_rec = np.zeros((2, 1, len(t)))
    u1_rec = np.zeros((1, 1, len(t)))

    # Two-state feedforward
    x2 = np.matrix([[0], [0]])
    x2_rec = np.zeros((2, 1, len(t)))
    u2_rec = np.zeros((1, 1, len(t)))

    # Two-state feedforward (no R cost)
    x3 = np.matrix([[0], [0]])
    x3_rec = np.zeros((2, 1, len(t)))
    u3_rec = np.zeros((1, 1, len(t)))

    u_min = np.asmatrix(-12)
    u_max = np.asmatrix(12)

    for k in range(len(t)):
        # Without feedforward
        u0 = K * (r - x0)

        # With steady-state feedforward
        u1 = K * (r - x1) + Kff1 * r

        # With two-state feedforwards
        u2 = K * (r - x2) + Kff2 * (r - sysd.A * r)
        u3 = K * (r - x3) + Kff3 * (r - sysd.A * r)

        u0 = np.clip(u0, u_min, u_max)
        x0 = sysd.A * x0 + sysd.B * u0
        u1 = np.clip(u1, u_min, u_max)
        x1 = sysd.A * x1 + sysd.B * u1
        u2 = np.clip(u2, u_min, u_max)
        x2 = sysd.A * x2 + sysd.B * u2
        u3 = np.clip(u3, u_min, u_max)
        x3 = sysd.A * x3 + sysd.B * u3

        r_rec[:, :, k] = r
        x0_rec[:, :, k] = x0
        u0_rec[:, :, k] = u0
        x1_rec[:, :, k] = x1
        u1_rec[:, :, k] = u1
        x2_rec[:, :, k] = x2
        u2_rec[:, :, k] = u2
        x3_rec[:, :, k] = x3
        u3_rec[:, :, k] = u3

    plt.figure(1)

    plt.subplot(3, 1, 1)
    plt.plot(t, r_rec[0, 0, :], label="Reference")
    plt.ylabel("$\omega$ (rad/s)")
    plt.plot(t, x0_rec[0, 0, :], label="No FF")
    plt.plot(t, x1_rec[0, 0, :], label="Steady-state FF")
    plt.plot(t, x2_rec[0, 0, :], label="Two-state FF")
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(t, r_rec[1, 0, :], label="Reference")
    plt.ylabel("Current (A)")
    plt.plot(t, x0_rec[1, 0, :], label="No FF")
    plt.plot(t, x1_rec[1, 0, :], label="Steady-state FF")
    plt.plot(t, x2_rec[1, 0, :], label="Two-state FF")
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(t, u0_rec[0, 0, :], label="No FF")
    plt.plot(t, u1_rec[0, 0, :], label="Steady-state FF")
    plt.plot(t, u2_rec[0, 0, :], label="Two-state FF")
    plt.legend()
    plt.ylabel("Control effort (V)")
    plt.xlabel("Time (s)")

    latexutils.savefig("case_study_ff1")

    plt.figure(2)

    plt.subplot(3, 1, 1)
    plt.plot(t, r_rec[0, 0, :], label="Reference")
    plt.ylabel("$\omega$ (rad/s)")
    plt.plot(t, x2_rec[0, 0, :], label="Two-state FF")
    plt.plot(t, x3_rec[0, 0, :], label="Two-state FF (no R cost)")
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(t, r_rec[1, 0, :], label="Reference")
    plt.ylabel("Current (A)")
    plt.plot(t, x2_rec[1, 0, :], label="Two-state FF")
    plt.plot(t, x3_rec[1, 0, :], label="Two-state FF (no R cost)")
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(t, u2_rec[0, 0, :], label="Two-state FF")
    plt.plot(t, u3_rec[0, 0, :], label="Two-state FF (no R cost)")
    plt.legend()
    plt.ylabel("Control effort (V)")
    plt.xlabel("Time (s)")

    latexutils.savefig("case_study_ff2")


if __name__ == "__main__":
    main()
