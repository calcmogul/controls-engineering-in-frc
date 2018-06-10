#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import norm

plt.rc("text", usetex=True)


def main():
    # Plot between -9 and 11 with 0.001 steps
    x_axis = np.arange(-7.5, 9.5, 0.001)
    # u = 0, omega = 2
    plt.plot(x_axis, norm.pdf(x_axis, 1, 2))
    plt.axvline(x=2.0, label="$x_1$", color="k", linestyle="--")
    plt.axvline(x=2.5, label="$x_1 + dx_1$", color="k", linestyle="-.")
    plt.xlabel("x")
    plt.ylabel("p(x)")
    plt.legend()
    plt.savefig("pdf.svg")


if __name__ == "__main__":
    main()
