#!/usr/bin/env python3

"""Simulates drivetrain velocity control with a time delay."""

import sys

import frccontrol as fct
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import scipy as sp

from bookutil import latex

if "--noninteractive" in sys.argv:
    mpl.use("svg")
plt.rc("text", usetex=True)


class Drivetrain:
    """An frccontrol system representing a drivetrain with a time delay."""

    def __init__(self, dt, delay=0.0):
        """Drivetrain subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        delay -- input delay in seconds
        """
        self.dt = dt
        self.delay = delay

        Kv = 3.02
        Ka = 0.642

        A = np.array([[-Kv / Ka]])
        B = np.array([[1.0 / Ka]])
        C = np.array([[1]])
        D = np.array([[0]])

        self.plant = sp.signal.StateSpace(A, B, C, D)

        # Sim variables
        self.sim = self.plant.to_discrete(self.dt)
        self.x = np.zeros((1, 1))
        self.u = np.zeros((1, 1))
        self.y = np.zeros((1, 1))

        # States: angular velocity (rad/s)
        # Inputs: voltage (V)
        # Outputs: angular velocity (rad/s)
        self.observer = fct.KalmanFilter(self.plant, [1.0], [0.01], self.dt)
        self.feedforward = fct.LinearPlantInversionFeedforward(
            self.plant.A, self.plant.B, self.dt
        )
        self.feedback = fct.LinearQuadraticRegulator(
            self.plant.A, self.plant.B, [0.2], [7.0], self.dt
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

        Keyword arguments:
        r -- the current reference
        next_r -- the next reference
        """
        # Update sim model
        self.x = self.sim.A @ self.x + self.sim.B @ self.u
        self.y = self.sim.C @ self.x + self.sim.D @ self.u

        self.observer.predict(self.u, self.dt)
        self.observer.correct(self.u, self.y)
        self.u = np.clip(
            self.feedforward.calculate(next_r)
            + self.feedback.calculate(self.observer.x_hat, r),
            self.u_min,
            self.u_max,
        )
        self.ubuf.append(self.u)
        self.u = self.ubuf.pop(0)


def main():
    """Entry point."""

    dt = 0.001
    delay = 0.04

    # Set up graphing
    l0 = 0.1
    l1 = l0 + 5.0
    l2 = l1 + 0.1
    ts = np.arange(0, l2 + 5.0, dt)

    # Run simulation
    refs = []
    for t in ts:
        if t < l0:
            r = np.array([[0.0]])
        elif t < l1:
            r = np.array([[2.0]])
        else:
            r = np.array([[0.0]])
        refs.append(r)
    for i in range(2):
        drivetrain = Drivetrain(dt, delay)
        if i == 1:
            drivetrain.feedback.latency_compensate(
                drivetrain.plant.A, drivetrain.plant.B, drivetrain.dt, delay
            )

        x_rec, ref_rec, u_rec, _ = fct.generate_time_responses(drivetrain, refs)

        plt.figure()
        if i == 0:
            # Plot angular velocity
            plt.subplot(2, 1, 1)
            plt.ylabel("Velocity (m/s)")
            plt.plot(
                ts,
                x_rec[0, :],
                label=f"State ($K_p = {round(drivetrain.feedback.K[0, 0], 2)}$)",
            )
            plt.plot(ts, ref_rec[0, :], label="Reference")
            plt.legend()

            # Plot voltage
            plt.subplot(2, 1, 2)
            plt.ylabel("Voltage (V)")
            plt.plot(ts, u_rec[0, :], label="Control effort")
            plt.legend()
            plt.xlabel("Time (s)")

            if "--noninteractive" in sys.argv:
                latex.savefig("drivetrain_time_delay_no_comp")
        else:
            # Plot angular velocity
            plt.subplot(2, 1, 1)
            plt.ylabel("Velocity (m/s)")
            plt.plot(
                ts,
                x_rec[0, :],
                label=f"State ($K_p = {round(drivetrain.feedback.K[0, 0], 2)}$)",
            )
            plt.plot(ts, ref_rec[0, :], label="Reference")
            plt.legend()

            # Plot voltage
            plt.subplot(2, 1, 2)
            plt.ylabel("Voltage (V)")
            plt.plot(ts, u_rec[0, :], label="Control effort")
            plt.legend()
            plt.xlabel("Time (s)")

            if "--noninteractive" in sys.argv:
                latex.savefig("drivetrain_time_delay_comp")
            else:
                plt.show()


if __name__ == "__main__":
    main()
