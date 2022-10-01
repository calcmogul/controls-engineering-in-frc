#!/usr/bin/env python3

"""Plots Taylor series approximations of exp(x)."""

import math
import sys

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

from bookutil import latex

# Set the default color cycle
mpl.rcParams["axes.prop_cycle"] = mpl.cycler(
    color=["k", "violet", "blueviolet", "b", "g", "darkorange", "crimson"]
)

if "--noninteractive" in sys.argv:
    mpl.use("svg")
plt.rc("text", usetex=True)


def taylor_exp(x, n):
    """Evaluates the nth-order Taylor series expansion of e^x around x = 0 at
    the given x.

    Keyword arguments:
    x -- The value at which to evaluate the Taylor series expansion.
    n -- The order of the Taylor series expansion.
    """
    val = 0
    for i in range(n + 1):
        val += x**i / math.factorial(i)
    return val


def main():
    """Entry point."""
    xlim = [-5, 5]
    xs = np.arange(xlim[0], xlim[1], 0.001)
    plt.xlim(xlim)
    plt.ylim([-2, 20])
    plt.plot(xs, [math.exp(x) for x in xs], label="$e^t$")
    for i in range(5, -1, -1):
        plt.plot(
            xs,
            [taylor_exp(x, i) for x in xs],
            label=f"Taylor series of $e^t$ ($n = {i}$)",
        )
    plt.xlabel("$t$")
    plt.ylabel("$f(t)$")
    plt.legend()
    if "--noninteractive" in sys.argv:
        latex.savefig("taylor_series")
    else:
        plt.show()


if __name__ == "__main__":
    main()
