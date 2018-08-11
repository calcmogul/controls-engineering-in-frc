#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
    import latexutils

import control as cnt
from frccontrol import conv
import matplotlib.pyplot as plt

plt.rc("text", usetex=True)


def main():
    #        (s - 9 + 9i)(s - 9 - 9i)
    # G(s) = ------------------------
    #               s(s + 10)
    G = cnt.tf(conv([1, -9 + 9j], [1, -9 - 9j]), conv([1, 0], [1, 10]))
    cnt.root_locus(G, grid=True)

    # Show plot
    plt.title("Root Locus")
    plt.xlabel("Real Axis (seconds$^{-1}$)")
    plt.ylabel("Imaginary Axis (seconds$^{-1}$)")
    plt.gca().set_aspect("equal")
    if "--noninteractive" in sys.argv:
        latexutils.savefig("rlocus_zeroes")
    else:
        plt.show()


if __name__ == "__main__":
    main()
