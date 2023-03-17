#!/usr/bin/env python3

"""Simulate an elevator with a zero-order hold."""

import sys

import frccontrol as fct
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

from bookutil import latex
from bookutil.systems import Elevator

if "--noninteractive" in sys.argv:
    mpl.use("svg")
plt.rc("text", usetex=True)


def generate_zoh(data, dt, sample_period):
    """Generates zero-order hold of data set.

    Keyword arguments:
    data -- array of position data
    dt -- dt of original data samples
    sample_period -- desired time between samples in zero-order hold
    """
    y = []
    count = 0
    val = 0
    for datum in data:
        count += 1
        if count >= sample_period / dt:
            val = datum
            count = 0
        y.append(val)
    return y


def main():
    """Entry point."""
    dt = 0.005
    sample_period = 0.1
    elevator = Elevator(dt)

    # Set up graphing
    l0 = 0.1
    l1 = l0 + 5.0
    l2 = l1 + 0.1
    l3 = l2 + 1.0
    ts = np.arange(0, l3, dt)

    # Generate references for simulation
    refs = []
    for t in ts:
        if t < l0:
            r = np.array([[0.0], [0.0]])
        elif t < l1:
            r = np.array([[1.524], [0.0]])
        else:
            r = np.array([[0.0], [0.0]])
        refs.append(r)

    state_rec, _, _, _ = fct.generate_time_responses(elevator, refs)
    pos = state_rec[0, :]

    plt.figure(1)
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    plt.plot(ts, pos, label="Continuous")
    y = generate_zoh(pos, dt, sample_period)
    plt.plot(ts, y, label=f"Zero-order hold (T={sample_period} s)")
    plt.legend()
    if "--noninteractive" in sys.argv:
        latex.savefig("zoh")
    else:
        plt.show()


if __name__ == "__main__":
    main()
