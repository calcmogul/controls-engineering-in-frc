#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
    import bookutil.latex as latex

import control as ct
import matplotlib.pyplot as plt

plt.rc("text", usetex=True)


def main():
    #          1
    # G(s) = -----
    #        s - 1
    G = ct.tf([1], [1, -1])
    ct.root_locus(G, grid=True)

    plt.title("Root Locus")
    plt.xlabel("Real Axis (seconds$^{-1}$)")
    plt.ylabel("Imaginary Axis (seconds$^{-1}$)")
    if "--noninteractive" in sys.argv:
        latex.savefig("rlocus_infty")
    else:
        plt.show()


if __name__ == "__main__":
    main()
