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

plt.rc("text", usetex=True)


def taylor_exp(x, n):
    """Returns value at x for Taylor series expansion of order n."""
    val = 0
    for i in range(n + 1):
        val += x**i / math.factorial(i)
    return val


def main():
    xlim = [-5, 5]
    xs = np.arange(xlim[0], xlim[1], 0.001)
    plt.xlim(xlim)
    plt.ylim([-2, 20])
    plt.plot(xs, [math.exp(x) for x in xs], label="$e^t$")
    for i in range(6):
        plt.plot(
            xs, [taylor_exp(x, i) for x in xs],
            label="Taylor series of $e^t$ ($n = " + str(i) + "$)")
    plt.xlabel("$t$")
    plt.ylabel("$f(t)$")
    plt.legend()
    if "--noninteractive" in sys.argv:
        latexutils.savefig("taylor_series")
    else:
        plt.show()


if __name__ == "__main__":
    main()
