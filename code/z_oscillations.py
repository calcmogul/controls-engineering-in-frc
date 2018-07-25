#!/usr/bin/env python3

import matplotlib as mpl
mpl.use("svg")
import control as cnt
from frccontrol import conv
import matplotlib.pyplot as plt
import numpy as np


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
    plt.savefig("z_oscillations_1p.svg")

    plt.figure(2)
    plt.xlabel("Time (s)")
    plt.ylabel("Step response")
    tf = cnt.TransferFunction(1, conv([1, 0.6 + 0.6j], [1, 0.6 - 0.6j]), dt)
    sim(tf, T, "Complex conjugate poles in LHP")
    tf = cnt.TransferFunction(1, conv([1, -0.6 + 0.6j], [1, -0.6 - 0.6j]), dt)
    sim(tf, T, "Complex conjugate poles in RHP")
    plt.savefig("z_oscillations_2p.svg")


if __name__ == "__main__":
    main()
