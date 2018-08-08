#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
    import latexutils

import math
import matplotlib.pyplot as plt
import numpy as np
import scipy.fftpack

plt.rc("text", usetex=True)


def main():
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

    plt.subplot(num_plots, 1, 1)
    plt.ylim(ylim)
    plt.ylabel("Fmaj4")
    plt.plot(x, ysum)
    plt.gca().axes.get_xaxis().set_ticks([])

    plt.subplot(num_plots, 1, 2)
    plt.ylim(ylim)
    plt.ylabel("$F_4$")
    plt.plot(x, yf)
    plt.gca().axes.get_xaxis().set_ticks([])

    plt.subplot(num_plots, 1, 3)
    plt.ylim(ylim)
    plt.ylabel("$A_4$")
    plt.plot(x, ya)
    plt.gca().axes.get_xaxis().set_ticks([])

    plt.subplot(num_plots, 1, 4)
    plt.ylim(ylim)
    plt.ylabel("$C_4$")
    plt.plot(x, yc)

    plt.xlabel("$t$")

    if "--noninteractive" in sys.argv:
        latexutils.savefig("fourier_chord")

    plt.figure(2)
    N = 1000000  # Number of samples
    T = 1.0 / 2000.0  # Sample spacing
    x = np.arange(0.0, N * T, T)
    yf = np.sin(f_f * 2 * math.pi * x)
    ya = np.sin(f_a * 2 * math.pi * x)
    yc = np.sin(f_c * 2 * math.pi * x)
    ysum = yf + ya + yc
    yf = scipy.fftpack.fft(ysum)
    xf = np.linspace(0.0, 1.0 / (2.0 * T), N // 2)

    plt.plot(xf, 2.0 / N * np.abs(yf[: N // 2]))
    plt.xlabel("Frequency (Hz)")
    plt.xlim([0, 700])

    if "--noninteractive" in sys.argv:
        latexutils.savefig("fourier_chord_fft")
    else:
        plt.show()


if __name__ == "__main__":
    main()
