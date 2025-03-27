#!/usr/bin/env python3

"""Plots a joint probability density function in 3D."""

import sys

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import multivariate_normal

from bookutil import latex

if "--noninteractive" in sys.argv:
    mpl.use("svg")


def main():
    """Entry point."""
    x, y = np.mgrid[-1.0:1.0:30j, 0.0:2.0:30j]

    # Need an (N, 2) array of (x, y) pairs.
    xy = np.column_stack([x.flat, y.flat])

    mu = np.array([0.0, 1.0])
    sigma = np.array([0.5, 0.5])
    covariance = np.diag(sigma**2)
    z = multivariate_normal.pdf(xy, mean=mu, cov=covariance).reshape(x.shape)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.plot_surface(x, y, z)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("p(x, y)")

    if "--noninteractive" in sys.argv:
        latex.savefig("joint_pdf")
    else:
        plt.show()


if __name__ == "__main__":
    main()
