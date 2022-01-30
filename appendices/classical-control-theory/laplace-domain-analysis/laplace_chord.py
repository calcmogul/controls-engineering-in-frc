#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
    import bookutil.latex as latex

import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import math
import numpy as np

plt.rc("text", usetex=True)


def sin_tf(freq, s):
    return freq / ((s - freq * 1j) * (s + freq * 1j))


def clamp(val, low, high):
    return max(low, min(val, high))


def main():
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

    plt.subplot(num_plots, 1, 1)
    plt.ylim(ylim)
    plt.ylabel("Fmaj4 ($\sigma = 0$)")
    plt.plot(x, ysum)
    plt.gca().axes.get_xaxis().set_ticks([])

    plt.subplot(num_plots, 1, 2)
    plt.ylim(ylim)
    plt.ylabel("Attenuating Fmaj4 ($\sigma = -25$)")
    plt.plot(x, ysum_attenuating)
    plt.gca().axes.get_xaxis().set_ticks([])

    plt.xlabel("$t$")

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
    ax.set_xlabel("$Re(\sigma)$")
    ax.set_ylabel("$Im(j\omega)$")
    ax.set_zlabel("$H(s)$")
    ax.set_zticks([])

    if "--noninteractive" in sys.argv:
        latex.savefig("laplace_chord_3d")
    else:
        plt.show()


if __name__ == "__main__":
    main()
