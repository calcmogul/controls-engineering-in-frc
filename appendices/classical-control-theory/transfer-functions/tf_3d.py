#!/usr/bin/env python3

"""Plots the magnitude of a transfer function in 3D."""

import math
import sys

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib import cm
import numpy as np

from bookutil import latex

if "--noninteractive" in sys.argv:
    mpl.use("svg")


def clamp(val, low, high):
    """Clamps a value within a range."""
    return max(low, min(val, high))


def func(x, y):
    """
    Function to plot.

    Parameter ``x``:
        x coordinate.
    Parameter ``y``:
        y coordinate.
    Returns:
        z coordinate.
    """
    s = x + y * 1j
    h = (s - 9 + 9j) * (s - 9 - 9j) / (s * (s + 10))
    return clamp(math.sqrt(h.real**2 + h.imag**2), -30, 30)


def main():
    """Entry point."""
    x, y = np.mgrid[-20.0:20.0:250j, -20.0:20.0:250j]

    # Need an (N, 2) array of (x, y) pairs.
    xy = np.column_stack([x.flat, y.flat])

    z = np.zeros(xy.shape[0])
    for i, pair in enumerate(xy):
        z[i] = func(pair[0], pair[1])
    z = z.reshape(x.shape)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.plot_surface(x, y, z, cmap=cm.coolwarm)
    ax.set_xlabel("Re(σ)")
    ax.set_ylabel("Im(jω)")
    ax.set_zlabel("H(s)")
    ax.set_zticks([])

    if "--noninteractive" in sys.argv:
        latex.savefig("tf_3d")
    else:
        plt.show()


if __name__ == "__main__":
    main()
