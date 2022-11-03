#!/usr/bin/env python3

"""Plots the FFT of musical notes."""

import math
import sys

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import scipy.fftpack

from bookutil import latex

if "--noninteractive" in sys.argv:
    mpl.use("svg")
plt.rc("text", usetex=True)


def main():
    """Entry point."""
    T = 0.000001
    xlim = [0, 0.05]
    ylim = [-3, 3]
    x = np.arange(xlim[0], xlim[1], T)
    plt.xlim(xlim)
    plt.ylim(ylim)

    f_f = 349.23
    f_a = 440
    f_c = 261.63

    yf = np.sin(f_f * 2 * math.pi * x)
    ya = np.sin(f_a * 2 * math.pi * x)
    yc = np.sin(f_c * 2 * math.pi * x)
    ysum = yf + ya + yc

    num_plots = 4

    fig = plt.figure(1)
    plt.axis("off")

    ax = fig.add_subplot(num_plots, 1, 1)
    ax.set_ylim(ylim)
    ax.set_ylabel("Fmaj4")
    ax.plot(x, ysum)
    ax.xaxis.set_visible(False)
    ax.yaxis.set_visible(True)

    ax = fig.add_subplot(num_plots, 1, 2)
    ax.set_ylim(ylim)
    ax.set_ylabel("$F_4$")
    ax.plot(x, yf)
    ax.xaxis.set_visible(False)
    ax.yaxis.set_visible(True)

    ax = fig.add_subplot(num_plots, 1, 3)
    ax.set_ylim(ylim)
    ax.set_ylabel("$A_4$")
    ax.plot(x, ya)
    ax.xaxis.set_visible(False)
    ax.yaxis.set_visible(True)

    ax = fig.add_subplot(num_plots, 1, 4)
    ax.set_ylim(ylim)
    ax.set_ylabel("$C_4$")
    ax.plot(x, yc)
    ax.set_xlabel("Time (s)")
    ax.yaxis.set_visible(True)

    if "--noninteractive" in sys.argv:
        latex.savefig("fourier_chord")

    N = 1000000  # Number of samples
    T = 1.0 / 2000.0  # Sample spacing
    x = np.arange(0.0, N * T, T)
    yf = np.sin(f_f * 2 * math.pi * x)
    ya = np.sin(f_a * 2 * math.pi * x)
    yc = np.sin(f_c * 2 * math.pi * x)
    ysum = yf + ya + yc
    yf = scipy.fftpack.fft(ysum)
    xf = np.linspace(0.0, 1.0 / (2.0 * T), N // 2)

    plt.figure(2)
    plt.plot(xf, 2.0 / N * np.abs(yf[: N // 2]))
    plt.xlabel("Frequency (Hz)")
    plt.xlim([0, 700])

    if "--noninteractive" in sys.argv:
        latex.savefig("fourier_chord_fft")
    else:
        plt.show()


if __name__ == "__main__":
    main()
