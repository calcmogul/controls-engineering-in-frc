#!/usr/bin/env python3

"""Simulates an elevator with different discretization methods."""

import sys

import frccontrol as fct
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

from bookutil import latex

if "--noninteractive" in sys.argv:
    mpl.use("svg")


class Elevator:
    """An frccontrol system representing an elevator."""

    def __init__(self, dt):
        """
        Elevator subsystem.

        Parameter ``dt``:
            Time between model/controller updates.
        """
        self.dt = dt

        # Number of motors
        num_motors = 2.0
        # Elevator carriage mass in kg
        m = 6.803886
        # Radius of pulley in meters
        r = 0.02762679089
        # Gear ratio
        G = 42.0 / 12.0 * 40.0 / 14.0
        self.plant = fct.elevator(fct.MOTOR_CIM, num_motors, m, r, G)

        # Sim variables
        self.sim = self.plant.to_discrete(self.dt)
        self.x = np.zeros((2, 1))
        self.u = np.zeros((1, 1))
        self.y = np.zeros((1, 1))

        # States: position (m), velocity (m/s)
        # Inputs: voltage (V)
        # Outputs: position (m)
        self.feedforward = fct.LinearPlantInversionFeedforward(
            self.plant.A, self.plant.B, self.dt
        )
        self.feedback = fct.LinearQuadraticRegulator(
            self.plant.A, self.plant.B, [0.02, 0.4], [12.0], self.dt
        )

        self.u_min = np.array([[-12.0]])
        self.u_max = np.array([[12.0]])

    def update(self, r, next_r):
        """
        Advance the model by one timestep.

        Parameter ``r``:
            The current reference.
        Parameter ``next_r``:
            The next reference.
        """
        # Update sim model
        self.x = self.sim.A @ self.x + self.sim.B @ self.u
        self.y = self.sim.C @ self.x + self.sim.D @ self.u

        self.u = np.clip(
            self.feedforward.calculate(next_r) + self.feedback.calculate(self.x, r),
            self.u_min,
            self.u_max,
        )


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
    """
    Simulate an elevator with a timestep of dt using the given discretization
    method.

    Parameter ``elevator``:
        The elevator to simulate.
    Parameter ``dt``:
        The timestep duration.
    Parameter ``method``:
        The discretization method ("zoh", "euler", "backward_diff", or
        "bilinear")
    """
    ts, refs = generate_refs(dt)
    elevator.sim = elevator.plant.to_discrete(dt, method)
    elevator.x = np.zeros((elevator.x.shape[0], 1))
    _, x_rec, _, _ = fct.generate_time_responses(elevator, refs)

    label = ""
    if method == "zoh":
        label += "Zero-order hold"
    elif method == "euler":
        label += "Forward Euler"
    elif method == "backward_diff":
        label += "Backward Euler"
    elif method == "bilinear":
        label += "Bilinear transform"
    label += f" (T={dt} s)"
    plt.plot(ts, x_rec[0, :], label=label)


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
