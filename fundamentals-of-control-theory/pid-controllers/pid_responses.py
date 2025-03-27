#!/usr/bin/env python3

"""Plots PID controller underdamped, overdamped, and critically damped responses."""

import sys

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import TransferFunction, step

from bookutil import latex

if "--noninteractive" in sys.argv:
    mpl.use("svg")


def sim(tf, T, label):
    """Simulates the given transfer function for T seconds."""
    T, yout = step(tf, T=T)

    # 1 / yout[-1] normalizes to DC gain
    plt.plot(T, yout * 1 / yout[-1], label=label)
    plt.legend()


def feedback(tf1, tf2, sign=-1):
    """Feedback interconnection between two LTI objects."""
    return TransferFunction(
        np.polymul(tf1.num, tf2.den),
        np.polyadd(np.polymul(tf2.den, tf1.den), -sign * np.polymul(tf2.num, tf1.num)),
    )


def main():
    """Entry point."""
    dt = 0.0001
    T = np.arange(0, 6, dt)

    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")

    # Make plant
    G = TransferFunction(1, np.convolve([1, 5], [1, 0]))

    sim(TransferFunction(1, 1), T, "Setpoint")

    K = TransferFunction(120, 1)
    Gcl = feedback(G, K)
    sim(Gcl, T, "Underdamped")

    K = TransferFunction(3, 1)
    Gcl = feedback(G, K)
    sim(Gcl, T, "Overdamped")

    K = TransferFunction(6.268, 1)
    Gcl = feedback(G, K)
    sim(Gcl, T, "Critically damped")

    if "--noninteractive" in sys.argv:
        latex.savefig("pid_responses")
    else:
        plt.show()


if __name__ == "__main__":
    main()
