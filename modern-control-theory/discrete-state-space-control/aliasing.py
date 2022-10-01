#!/usr/bin/env python3

"""Demonstrates aliasing of two sine waves."""

import math
import sys

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

from bookutil import latex

if "--noninteractive" in sys.argv:
    mpl.use("svg")
plt.rc("text", usetex=True)


def main():
    """Entry point."""
    xlim = [0, 4]
    x = np.arange(xlim[0], xlim[1], 0.001)
    plt.xlim(xlim)

    y1 = np.sin(2 * math.pi * x * 1.5)
    y2 = np.sin(-2 * math.pi * x * 0.5)
    plt.plot(x, y1, label="Original signal at $1.5$ Hz")

    # Draw intersection points
    idx = np.argwhere(np.diff(np.sign(y1 - y2)) != 0)
    plt.plot(x[idx], y1[idx], "ko", label="Samples at $2$ Hz")

    plt.plot(x, y2, color="k", linestyle="--", label="Aliased signal at $0.5$ Hz")

    plt.xlabel("$t$")
    plt.ylabel("$f(t)$")
    plt.legend()

    if "--noninteractive" in sys.argv:
        latex.savefig("aliasing")
    else:
        plt.show()


if __name__ == "__main__":
    main()
