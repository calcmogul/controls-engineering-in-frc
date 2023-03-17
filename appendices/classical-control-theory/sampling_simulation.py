#!/usr/bin/env python3

"""Simulates an elevator with different discretization methods."""

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


def generate_refs(T):
    """Generates refs for the given duration."""
    # Set up graphing
    l0 = 0.1
    l1 = l0 + 5.0
    l2 = l1 + 0.1
    l3 = l2 + 1.0
    ts = np.arange(0, l3, T)

    refs = []

    # Generate references for simulation
    for t in ts:
        if t < l0:
            r = np.array([[0.0], [0.0]])
        elif t < l1:
            r = np.array([[1.524], [0.0]])
        else:
            r = np.array([[0.0], [0.0]])
        refs.append(r)

    return ts, refs


def simulate(elevator, dt, method):
    """Simulate an elevator with a timestep of dt using the given discretization
    method.

    Keyword arguments:
    elevator -- the elevator to simulate
    dt -- the timestep duration
    method -- the discretization method ("zoh", "euler", "backward_diff", or
              "bilinear")
    """
    ts, refs = generate_refs(dt)
    elevator.sim = elevator.plant.to_discrete(dt, method)
    elevator.x = np.zeros((elevator.x.shape[0], 1))
    elevator.observer.x_hat = np.zeros((elevator.observer.x_hat.shape[0], 1))
    state_rec, _, _, _ = fct.generate_time_responses(elevator, refs)

    pos = state_rec[0, :]
    if method == "zoh":
        label = "Zero-order hold"
    elif method == "euler":
        label = "Forward Euler"
    elif method == "backward_diff":
        label = "Backward Euler"
    elif method == "bilinear":
        label = "Bilinear transform"
    label += f" (T={dt} s)"
    plt.plot(ts, pos, label=label)


def main():
    """Entry point."""
    elevator = Elevator(0.1)

    plt.figure(1)
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    simulate(elevator, 0.1, "zoh")
    simulate(elevator, 0.1, "euler")
    simulate(elevator, 0.1, "backward_diff")
    simulate(elevator, 0.1, "bilinear")
    plt.ylim([-2, 3])
    plt.legend()
    if "--noninteractive" in sys.argv:
        latex.savefig("sampling_simulation_010")

    plt.figure(2)
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    simulate(elevator, 0.05, "zoh")
    simulate(elevator, 0.05, "euler")
    simulate(elevator, 0.05, "backward_diff")
    simulate(elevator, 0.05, "bilinear")
    plt.ylim([-2, 3])
    plt.legend()
    if "--noninteractive" in sys.argv:
        latex.savefig("sampling_simulation_005")

    plt.figure(3)
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    simulate(elevator, 0.01, "zoh")
    simulate(elevator, 0.01, "euler")
    simulate(elevator, 0.01, "backward_diff")
    simulate(elevator, 0.01, "bilinear")
    plt.ylim([-0.25, 2])
    plt.legend()
    if "--noninteractive" in sys.argv:
        latex.savefig("sampling_simulation_004")
    else:
        plt.show()


if __name__ == "__main__":
    main()
