#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
    import bookutil.latex as latex

import control as ct
from frccontrol import conv
import matplotlib.pyplot as plt
import numpy as np

plt.rc("text", usetex=True)


def sim(tf, T, label):
    T, yout = ct.step_response(tf, T=T)

    plt.plot(T, np.squeeze(yout), label=label)
    plt.legend()


def main():
    dt = 0.1
    T = np.arange(0, 6, dt)

    plt.figure(1)
    plt.xticks(np.arange(min(T), max(T) + 1, 1.0))
    plt.xlabel("Time (s)")
    plt.ylabel("Step response")
    tf = ct.TransferFunction(1, [1, -0.6], dt)
    sim(tf, T, "Single pole in RHP")
    tf = ct.TransferFunction(1, [1, 0.6], dt)
    sim(tf, T, "Single pole in LHP")
    if "--noninteractive" in sys.argv:
        latex.savefig("z_oscillations_1p")

    plt.figure(2)
    plt.xlabel("Time (s)")
    plt.ylabel("Step response")
    den = [np.real(x) for x in conv([1, 0.6 + 0.6j], [1, 0.6 - 0.6j])]
    tf = ct.TransferFunction(1, den, dt)
    sim(tf, T, "Complex conjugate poles in LHP")
    den = [np.real(x) for x in conv([1, -0.6 + 0.6j], [1, -0.6 - 0.6j])]
    tf = ct.TransferFunction(1, den, dt)
    sim(tf, T, "Complex conjugate poles in RHP")
    if "--noninteractive" in sys.argv:
        latex.savefig("z_oscillations_2p")
    else:
        plt.show()


if __name__ == "__main__":
    main()
