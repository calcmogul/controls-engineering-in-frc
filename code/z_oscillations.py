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
import numpy as np

plt.rc("text", usetex=True)


def sim(tf, T, label):
    T, yout = cnt.step_response(tf, T=T)

    plt.plot(T, np.squeeze(yout, 0), label=label)
    plt.legend()


def main():
    dt = 0.1
    T = np.arange(0, 6, dt)

    plt.figure(1)
    plt.xticks(np.arange(min(T), max(T) + 1, 1.0))
    plt.xlabel("Time (s)")
    plt.ylabel("Step response")
    tf = cnt.TransferFunction(1, [1, -0.6], dt)
    sim(tf, T, "Single pole in RHP")
    tf = cnt.TransferFunction(1, [1, 0.6], dt)
    sim(tf, T, "Single pole in LHP")
    if "--noninteractive" in sys.argv:
        latexutils.savefig("z_oscillations_1p")

    plt.figure(2)
    plt.xlabel("Time (s)")
    plt.ylabel("Step response")
    tf = cnt.TransferFunction(1, conv([1, 0.6 + 0.6j], [1, 0.6 - 0.6j]), dt)
    sim(tf, T, "Complex conjugate poles in LHP")
    tf = cnt.TransferFunction(1, conv([1, -0.6 + 0.6j], [1, -0.6 - 0.6j]), dt)
    sim(tf, T, "Complex conjugate poles in RHP")
    if "--noninteractive" in sys.argv:
        latexutils.savefig("z_oscillations_2p")
    else:
        plt.show()


if __name__ == "__main__":
    main()
