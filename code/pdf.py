#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
    import utils.latex as latex

import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import norm

plt.rc("text", usetex=True)


def main():
    themecolor = "#386ba6"

    x1 = np.arange(-7.5, 9.5, 0.001)
    plt.plot(x1, norm.pdf(x1, loc=1, scale=2))

    x2 = np.arange(2.0, 2.5, 0.001)
    plt.fill_between(
        x2, [0] * len(x2), norm.pdf(x2, loc=1, scale=2), facecolor=themecolor, alpha=0.5
    )
    plt.plot([2.0, 2.0], [0.0, norm.pdf([2.0], loc=1, scale=2)], color=themecolor)
    plt.plot([2.5, 2.5], [0.0, norm.pdf([2.5], loc=1, scale=2)], color=themecolor)

    # Left arrow
    plt.annotate(
        "$x_1$",
        xy=(2.0, 0.025),  # Start coord of arrow
        xycoords="data",
        xytext=(2.0 - 1.0, 0.025),  # End coord of arrow
        textcoords="data",
        arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=0"),
        ha="center",
        va="center",
    )

    plt.annotate(
        "$dx$", xy=(2.25, -0.005), xycoords="data", ha="center", va="center",
    )

    plt.xlabel("$x$")
    plt.ylabel("$p(x)$")
    if "--noninteractive" in sys.argv:
        latex.savefig("pdf")
    else:
        plt.show()


if __name__ == "__main__":
    main()
