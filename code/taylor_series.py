#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

import matplotlib as mpl

if "--noninteractive" in sys.argv:
    mpl.use("svg")
    import latexutils

import math
import matplotlib.pyplot as plt
import numpy as np

# Set the default color cycle
mpl.rcParams["axes.prop_cycle"] = mpl.cycler(
    color=["k", "violet", "blueviolet", "b", "g", "darkorange", "crimson"]
)

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
        val += x ** i / math.factorial(i)
    return val


def main():
    xlim = [-5, 5]
    xs = np.arange(xlim[0], xlim[1], 0.001)
    plt.xlim(xlim)
    plt.ylim([-2, 20])
    plt.plot(xs, [math.exp(x) for x in xs], label="$e^t$")
    for i in range(5, -1, -1):
        plt.plot(
            xs,
            [taylor_exp(x, i) for x in xs],
            label="Taylor series of $e^t$ ($n = " + str(i) + "$)",
        )
    plt.xlabel("$t$")
    plt.ylabel("$f(t)$")
    plt.legend()
    if "--noninteractive" in sys.argv:
        latexutils.savefig("taylor_series")
    else:
        plt.show()


if __name__ == "__main__":
    main()
