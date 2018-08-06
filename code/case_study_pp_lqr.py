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
    Kpp1 = cnt.place(sysd.A, sysd.B, [0.1, 0.9])
    Kpp2 = cnt.place(sysd.A, sysd.B, [0.1, 0.8])
    Klqr = frccnt.dlqr(sysd, Q, R)

    t = np.arange(0, tmax, dt)
    r = np.matrix([[2000 * 0.1047], [0]])
    r_rec = np.zeros((2, 1, len(t)))

    # Pole placement 1
    xpp1 = np.matrix([[0], [0]])
    xpp1_rec = np.zeros((2, 1, len(t)))
    upp1_rec = np.zeros((1, 1, len(t)))

    # Pole placement 2
    xpp2 = np.matrix([[0], [0]])
    xpp2_rec = np.zeros((2, 1, len(t)))
    upp2_rec = np.zeros((1, 1, len(t)))

    # LQR
    xlqr = np.matrix([[0], [0]])
    xlqr_rec = np.zeros((2, 1, len(t)))
    ulqr_rec = np.zeros((1, 1, len(t)))

    u_min = np.asmatrix(-12)
    u_max = np.asmatrix(12)

    for k in range(len(t)):
        # Pole placement 1
        upp1 = Kpp1 * (r - xpp1)

        # Pole placement 2
        upp2 = Kpp2 * (r - xpp2)

        # LQR
        ulqr = Klqr * (r - xlqr)

        upp1 = np.clip(upp1, u_min, u_max)
        xpp1 = sysd.A * xpp1 + sysd.B * upp1
        upp2 = np.clip(upp2, u_min, u_max)
        xpp2 = sysd.A * xpp2 + sysd.B * upp2
        ulqr = np.clip(ulqr, u_min, u_max)
        xlqr = sysd.A * xlqr + sysd.B * ulqr

        r_rec[:, :, k] = r
        xpp1_rec[:, :, k] = xpp1
        upp1_rec[:, :, k] = upp1
        xpp2_rec[:, :, k] = xpp2
        upp2_rec[:, :, k] = upp2
        xlqr_rec[:, :, k] = xlqr
        ulqr_rec[:, :, k] = ulqr

    plt.figure(1)

    plt.subplot(3, 1, 1)
    plt.plot(t, r_rec[0, 0, :], label="Reference")
    plt.ylabel("$\omega$ (rad/s)")
    plt.plot(
        t,
        xpp1_rec[0, 0, :],
        label="Pole placement at $(0.1, 0)$ and $(0.9, 0)$")
    plt.plot(
        t,
        xpp2_rec[0, 0, :],
        label="Pole placement at $(0.1, 0)$ and $(0.8, 0)$")
    plt.plot(t, xlqr_rec[0, 0, :], label="LQR")
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(t, r_rec[1, 0, :], label="Reference")
    plt.ylabel("Current (A)")
    plt.plot(
        t,
        xpp1_rec[1, 0, :],
        label="Pole placement at $(0.1, 0)$ and $(0.9, 0)$")
    plt.plot(
        t,
        xpp2_rec[1, 0, :],
        label="Pole placement at $(0.1, 0)$ and $(0.8, 0)$")
    plt.plot(t, xlqr_rec[1, 0, :], label="LQR")
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(
        t,
        upp1_rec[0, 0, :],
        label="Pole placement at $(0.1, 0)$ and $(0.9, 0)$")
    plt.plot(
        t,
        upp2_rec[0, 0, :],
        label="Pole placement at $(0.1, 0)$ and $(0.8, 0)$")
    plt.plot(t, ulqr_rec[0, 0, :], label="LQR")
    plt.legend()
    plt.ylabel("Control effort (V)")
    plt.xlabel("Time (s)")

    latexutils.savefig("case_study_pp_lqr")


if __name__ == "__main__":
    main()
