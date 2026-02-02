#!/usr/bin/env python3

"""Simulates elevator position control with a time delay."""

import sys

import frccontrol as fct
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

from bookutil import latex

if "--noninteractive" in sys.argv:
    mpl.use("svg")


class Elevator:
    """An frccontrol system representing an elevator with a time delay."""

    def __init__(self, dt, delay=0.0):
        """
        Elevator subsystem.

        Parameter ``dt``:
            Time between model/controller updates.

        Parameter ``delay``:
            Input delay in seconds.
        """
        self.dt = dt
        self.delay = delay

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

        # Prepare time delay
        self.ubuf = []
        for _ in range(int(self.delay / self.dt)):
            self.ubuf.append(np.zeros((1, 1)))

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
        self.ubuf.append(self.u)
        self.u = self.ubuf.pop(0)


class PlotMetadata:
    """
    Plot metadata.
    """

    def __init__(self, delay, compensate, gain_digits, plot_filename):
        """
        Constructs PlotMetadata.

        Parameter ``delay``:
            The input delay.

        Parameter ``compensate``:
            Whether to perform latency compensation.

        Parameter ``gain_digits``:
            Number of digits to include in feedback gain plot label.

        Parameter ``plot_filename``:
            Plot filename.
        """
        self.delay = delay
        self.compensate = compensate
        self.gain_digits = gain_digits
        self.plot_filename = plot_filename


def main():
    """Entry point."""

    dt = 0.005

    # Set up graphing
    l0 = 0.1
    l1 = l0 + 5.0
    l2 = l1 + 0.1
    ts = np.arange(0, l2 + 5.0, dt)

    # Generate references
    refs = []
    for t in ts:
        if t < l0:
            r = np.array([[0.0], [0.0]])
        elif t < l1:
            r = np.array([[1.524], [0.0]])
        else:
            r = np.array([[0.0], [0.0]])
        refs.append(r)

    # Run simulations
    for plot in [
        PlotMetadata(0.05, False, 2, "elevator_time_delay_no_comp"),
        PlotMetadata(0.05, True, 2, "elevator_time_delay_comp"),
    ]:
        elevator = Elevator(dt, plot.delay)
        if plot.compensate:
            elevator.feedback.latency_compensate(
                elevator.plant.A, elevator.plant.B, elevator.dt, plot.delay
            )

        r_rec, x_rec, u_rec, _ = fct.generate_time_responses(elevator, refs)

        plt.figure()

        # Plot position
        plt.subplot(3, 1, 1)
        plt.ylabel("Position (m)")
        plt.plot(ts, r_rec[0, :], label="Reference")
        plt.plot(
            ts,
            x_rec[0, :],
            label=f"State ($K_p = {round(elevator.feedback.K[0, 0], plot.gain_digits)}$)",
        )
        plt.legend()

        # Plot velocity
        plt.subplot(3, 1, 2)
        plt.ylabel("Velocity (m/s)")
        plt.plot(ts, r_rec[1, :], label="Reference")
        plt.plot(
            ts,
            x_rec[1, :],
            label=f"State ($K_d = {round(elevator.feedback.K[0, 1], plot.gain_digits)}$)",
        )
        plt.legend()

        # Plot voltage
        plt.subplot(3, 1, 3)
        plt.ylabel("Voltage (V)")
        plt.plot(ts, u_rec[0, :], label="Input")
        plt.legend()
        plt.xlabel("Time (s)")

        if "--noninteractive" in sys.argv:
            latex.savefig(plot.plot_filename)

    if "--noninteractive" not in sys.argv:
        plt.show()


if __name__ == "__main__":
    main()
