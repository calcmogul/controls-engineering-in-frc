#!/usr/bin/env python3

from frccontrol.examples.elevator import Elevator
import matplotlib.pyplot as plt
import numpy as np

plt.rc("text", usetex=True)


def generate_refs(dt):
    # Set up graphing
    l0 = 0.1
    l1 = l0 + 5.0
    l2 = l1 + 0.1
    l3 = l2 + 1.0
    t = np.linspace(0, l3, l3 / dt)

    refs = []

    # Generate references for simulation
    for i in range(len(t)):
        if t[i] < l0:
            r = np.matrix([[0.0], [0.0]])
        elif t[i] < l1:
            r = np.matrix([[1.524], [0.0]])
        else:
            r = np.matrix([[0.0], [0.0]])
        refs.append(r)

    return t, refs


def simulate(elevator, dt, method):
    t, refs = generate_refs(dt)
    elevator.sysd = elevator.sysc.sample(dt, method)
    elevator.x = np.zeros((elevator.x.shape[0], 1))
    elevator.x_hat = np.zeros((elevator.x_hat.shape[0], 1))
    state_rec, ref_rec, u_rec = elevator.generate_time_responses(t, refs)

    pos = elevator.extract_row(state_rec, 0)
    if method == "zoh":
        label = "Zero-order hold"
    elif method == "euler":
        label = "Forward Euler"
    label += " (T={}s)".format(dt)
    plt.plot(t, pos, label=label)


def main():
    elevator = Elevator(0.1)

    plt.figure(1)
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")

    simulate(elevator, 0.1, "zoh")
    simulate(elevator, 0.1, "euler")
    simulate(elevator, 0.01, "euler")

    plt.ylim([-0.25, 3])
    plt.legend()
    plt.savefig("sampling_simulation.svg")


if __name__ == "__main__":
    main()
