#!/usr/bin/env python3

"""Simulates flywheel velocity control with a time delay."""

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


class Flywheel:
    """An frccontrol system representing a flywheel with a time delay."""

    def __init__(self, dt, delay=0.0):
        """Flywheel subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        delay -- input delay in seconds
        """
        self.dt = dt
        self.delay = delay

        Kv = 0.011
        Ka = 0.005515

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
        self.observer = fct.KalmanFilter(self.plant, [700.0], [50.0], self.dt)
        self.feedforward = fct.LinearPlantInversionFeedforward(
            self.plant.A, self.plant.B, self.dt
        )
        self.feedback = fct.LinearQuadraticRegulator(
            self.plant.A, self.plant.B, [80.0], [12.0], self.dt
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
    delay = 0.08

    # Set up graphing
    l0 = 0.1
    l1 = l0 + 5.0
    l2 = l1 + 0.1
    ts = np.arange(0, l2 + 5.0, dt)

    # Run simulations
    refs = []
    for t in ts:
        if t < l0:
            r = np.array([[0.0]])
        elif t < l1:
            r = np.array([[510.0]])
        else:
            r = np.array([[0.0]])
        refs.append(r)
    for i in range(2):
        flywheel = Flywheel(dt, delay)
        if i == 1:
            flywheel.feedback.latency_compensate(
                flywheel.plant.A, flywheel.plant.B, flywheel.dt, delay
            )

        x_rec, ref_rec, u_rec, _ = fct.generate_time_responses(flywheel, refs)

        plt.figure()
        if i == 0:
            # Plot angular velocity
            plt.subplot(2, 1, 1)
            plt.ylabel("Angular velocity (rad/s)")
            plt.plot(
                ts,
                x_rec[0, :],
                label=f"State ($K_p = {round(flywheel.feedback.K[0, 0], 2)}$)",
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
                latex.savefig("flywheel_time_delay_no_comp")
        else:
            # Plot angular velocity
            plt.subplot(2, 1, 1)
            plt.ylabel("Angular velocity (rad/s)")
            plt.plot(
                ts,
                x_rec[0, :],
                label=f"State ($K_p = {round(flywheel.feedback.K[0, 0], 8)}$)",
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
                latex.savefig("flywheel_time_delay_comp")
            else:
                plt.show()


if __name__ == "__main__":
    main()
