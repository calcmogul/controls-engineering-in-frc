#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
    import latexutils

import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import math
import numpy as np

plt.rc("text", usetex=True)


def clamp(val, low, high):
    return max(low, min(val, high))


def main():
    x, y = np.mgrid[-20.0:20.0:250j, -20.0:20.0:250j]

    # Need an (N, 2) array of (x, y) pairs.
    xy = np.column_stack([x.flat, y.flat])

    z = np.zeros(xy.shape[0])
    for i, pair in enumerate(xy):
        s = pair[0] + pair[1] * 1j
        h = (s - 9 + 9j) * (s - 9 - 9j) / (s * (s + 10))
        z[i] = clamp(math.sqrt(h.real ** 2 + h.imag ** 2), -30, 30)
    z = z.reshape(x.shape)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.plot_surface(x, y, z, cmap=cm.coolwarm)
    ax.set_xlabel("$Re(\sigma)$")
    ax.set_ylabel("$Im(j\omega)$")
    ax.set_zlabel("$H(s)$")
    ax.set_zticks([])

    if "--noninteractive" in sys.argv:
        latexutils.savefig("tf_3d")
    else:
        plt.show()


if __name__ == "__main__":
    main()
