#!/usr/bin/env python3

import matplotlib as mpl
mpl.use("svg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import norm

import latexutils

plt.rc("text", usetex=True)


def main():
    x = np.arange(-7.5, 9.5, 0.001)
    plt.plot(x, norm.pdf(x, loc=1, scale=2))
    plt.axvline(x=2.0, label="$x_1$", color="k", linestyle="--")
    plt.axvline(x=2.5, label="$x_1 + dx_1$", color="k", linestyle="-.")
    plt.xlabel("$x$")
    plt.ylabel("$p(x)$")
    plt.legend()
    latexutils.savefig("pdf")


if __name__ == "__main__":
    main()
