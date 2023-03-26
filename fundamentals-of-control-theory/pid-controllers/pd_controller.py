#!/usr/bin/env python3

"""Demonstrates PD controller."""

import sys

import frccontrol as fct
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

from bookutil import latex

if "--noninteractive" in sys.argv:
    mpl.use("svg")
plt.rc("text", usetex=True)


class Elevator:
    """An frccontrol system representing an elevator."""

    def __init__(self, dt):
        """Elevator subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
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
        self.plant = fct.models.elevator(fct.models.MOTOR_CIM, num_motors, m, r, G)

        # Sim variables
        self.sim = self.plant.to_discrete(self.dt)
        self.x = np.zeros((2, 1))
        self.u = np.zeros((1, 1))
        self.y = np.zeros((1, 1))

        # States: position (m), velocity (m/s)
        # Inputs: voltage (V)
        # Outputs: position (m)
        self.feedback = fct.LinearQuadraticRegulator(
            self.plant.A, self.plant.B, [0.02, 0.4], [12.0], self.dt
        )

        self.u_min = np.array([[-12.0]])
        self.u_max = np.array([[12.0]])

    # pylint: disable=unused-argument
    def update(self, r, next_r):
        """
        Advance the model by one timestep.

        Keyword arguments:
        r -- the current reference
        next_r -- the next reference
        """
        # Update sim model
        self.x = self.sim.A @ self.x + self.sim.B @ self.u
        self.y = self.sim.C @ self.x + self.sim.D @ self.u

        self.u = np.clip(
            self.feedback.calculate(self.x, r),
            self.u_min,
            self.u_max,
        )


def main():
    """Entry point."""
    dt = 0.005
    elevator = Elevator(dt)

    # Set up graphing
    l0 = 0.1
    l1 = l0 + 5.0
    l2 = l1 + 0.1
    ts = np.arange(0, l2 + 5.0, dt)

    ts, xprof, vprof, _ = fct.generate_trapezoid_profile(
        max_v=0.5, time_to_max_v=0.5, dt=dt, goal=3
    )

    # Generate references for simulation
    refs = [np.array([[xprof[i]], [vprof[i]]]) for i in range(len(ts))]

    # Run simulation
    x_rec, r_rec, u_rec, _ = fct.generate_time_responses(elevator, refs)

    plt.figure()

    # Plot position
    plt.subplot(3, 1, 1)
    plt.ylabel("Position (m)")
    plt.plot(
        ts, x_rec[0, :], label=f"Output ($K_p = {round(elevator.feedback.K[0, 0], 2)}$)"
    )
    plt.plot(ts, r_rec[0, :], label="Setpoint")
    plt.legend()

    # Plot velocity
    plt.subplot(3, 1, 2)
    plt.ylabel("Velocity (m/s)")
    plt.plot(
        ts, x_rec[1, :], label=f"Output ($K_d = {round(elevator.feedback.K[0, 1], 2)}$)"
    )
    plt.plot(ts, r_rec[1, :], label="Setpoint")
    plt.legend()

    # Plot voltage
    plt.subplot(3, 1, 3)
    plt.ylabel("Voltage (V)")
    plt.plot(ts, u_rec[0, :], label="Input")
    plt.legend()
    plt.xlabel("Time (s)")

    if "--noninteractive" in sys.argv:
        latex.savefig("pd_controller")
    else:
        plt.show()


if __name__ == "__main__":
    main()
