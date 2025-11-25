#!/usr/bin/env python3

"""Simulate an elevator with a zero-order hold."""

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


def generate_zoh(data, dt, sample_period):
    """
    Generates zero-order hold of data set.

    Parameter ``data``:
        Array of position data.
    Parameter ``dt``:
        dt of original data samples.
    Parameter ``sample_period``:
        Desired time between samples in zero-order hold.
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

    _, x_rec, _, _ = fct.generate_time_responses(elevator, refs)

    plt.figure(1)
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    plt.plot(ts, x_rec[0, :], label="Continuous")
    plt.plot(
        ts,
        generate_zoh(x_rec[0, :], dt, sample_period),
        label=f"Zero-order hold (T={sample_period} s)",
    )
    plt.legend()
    if "--noninteractive" in sys.argv:
        latex.savefig("zoh")
    else:
        plt.show()


if __name__ == "__main__":
    main()
