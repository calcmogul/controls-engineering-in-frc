#!/usr/bin/env python3

"""Simulates oscillations of different types of discrete transfer functions."""

import sys

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import TransferFunction, dstep

from bookutil import latex

if "--noninteractive" in sys.argv:
    mpl.use("svg")


def sim(tf, t, label):
    """Simulate the given transfer function for t seconds."""
    T, yout = dstep(tf, t=t)

    plt.plot(T, np.squeeze(yout), label=label)
    plt.legend()


def main():
    """Entry point."""
    dt = 0.1
    T = np.arange(0, 6, dt)

    plt.figure(1)
    plt.xticks(np.arange(min(T), max(T) + 1, 1.0))
    plt.xlabel("Time (s)")
    plt.ylabel("Step response")
    tf = TransferFunction(1, [1, -0.6], dt=dt)
    sim(tf, T, "Single pole in RHP")
    tf = TransferFunction(1, [1, 0.6], dt=dt)
    sim(tf, T, "Single pole in LHP")
    if "--noninteractive" in sys.argv:
        latex.savefig("z_oscillations_1p")

    plt.figure(2)
    plt.xlabel("Time (s)")
    plt.ylabel("Step response")
    den = [np.real(x) for x in np.convolve([1, 0.6 + 0.6j], [1, 0.6 - 0.6j])]
    tf = TransferFunction(1, den, dt=dt)
    sim(tf, T, "Complex conjugate poles in LHP")
    den = [np.real(x) for x in np.convolve([1, -0.6 + 0.6j], [1, -0.6 - 0.6j])]
    tf = TransferFunction(1, den, dt=dt)
    sim(tf, T, "Complex conjugate poles in RHP")
    if "--noninteractive" in sys.argv:
        latex.savefig("z_oscillations_2p")
    else:
        plt.show()


if __name__ == "__main__":
    main()
