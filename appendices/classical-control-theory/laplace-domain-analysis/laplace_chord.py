#!/usr/bin/env python3

"""Plots the Laplace transform of musical notes."""

import math
import sys

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib import cm
import numpy as np

from bookutil import latex

if "--noninteractive" in sys.argv:
    mpl.use("svg")


def sin_tf(freq, s):
    """Returns a transfer function that oscillates at the given frequency."""
    return freq / ((s - freq * 1j) * (s + freq * 1j))


def clamp(val, low, high):
    """Clamps a value within a range."""
    return max(low, min(val, high))


def main():
    """Entry point."""
    f_f = 349.23
    f_a = 440
    f_c = 261.63

    T = 0.000001
    xlim = [0, 0.05]
    ylim = [-3, 3]
    x = np.arange(xlim[0], xlim[1], T)
    plt.xlim(xlim)
    plt.ylim(ylim)

    yf = np.sin(f_f * 2 * math.pi * x)
    ya = np.sin(f_a * 2 * math.pi * x)
    yc = np.sin(f_c * 2 * math.pi * x)
    ysum = yf + ya + yc
    ysum_attenuating = ysum * np.exp(-25 * x)

    num_plots = 2

    fig = plt.figure(1)
    plt.axis("off")

    ax = fig.add_subplot(num_plots, 1, 1)
    ax.set_ylim(ylim)
    ax.set_ylabel("Fmaj4 (σ = 0)")
    ax.plot(x, ysum)
    ax.xaxis.set_visible(True)
    ax.yaxis.set_visible(True)

    ax = fig.add_subplot(num_plots, 1, 2)
    ax.set_ylim(ylim)
    ax.set_ylabel("Attenuating Fmaj4 (σ = -25)")
    ax.plot(x, ysum_attenuating)
    ax.set_xlabel("Time (s)")
    ax.yaxis.set_visible(True)

    if "--noninteractive" in sys.argv:
        latex.savefig("laplace_chord_attenuating")

    x, y = np.mgrid[-150.0:150.0:500j, 200.0:500.0:500j]

    # Need an (N, 2) array of (x, y) pairs.
    xy = np.column_stack([x.flat, y.flat])

    z = np.zeros(xy.shape[0])
    for i, pair in enumerate(xy):
        s = pair[0] + pair[1] * 1j
        h = sin_tf(f_f, s) * sin_tf(f_a, s) * sin_tf(f_c, s)
        z[i] = clamp(math.sqrt(h.real**2 + h.imag**2), -30, 30)
    z = z.reshape(x.shape)

    fig = plt.figure(2)
    ax = fig.add_subplot(111, projection="3d")
    ax.plot_surface(x, y, z, cmap=cm.coolwarm)
    ax.set_xlabel("Re(σ)")
    ax.set_ylabel("Im(jω)")
    ax.set_zlabel("H(s)")
    ax.set_zticks([])

    if "--noninteractive" in sys.argv:
        latex.savefig("laplace_chord_3d")
    else:
        plt.show()


if __name__ == "__main__":
    main()
