#!/usr/bin/env python3

"""Plots single-jointed arm following a motion profile."""

import sys

import frccontrol as fct
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

from bookutil import latex

if "--noninteractive" in sys.argv:
    mpl.use("svg")


class SingleJointedArm:
    """An frccontrol system representing a single-jointed arm."""

    def __init__(self, dt):
        """Single-jointed arm subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        """
        self.dt = dt

        # Number of motors
        num_motors = 1.0
        # Mass of arm in kg
        m = 2.2675
        # Length of arm in m
        l = 1.2192
        # Arm moment of inertia in kg-m^2
        J = 1 / 3 * m * l**2
        # Gear ratio
        G = 1.0 / 2.0
        self.plant = fct.models.single_jointed_arm(
            fct.models.MOTOR_CIM, num_motors, J, G
        )

        self.sim = self.plant.to_discrete(self.dt)
        self.x = np.zeros((2, 1))
        self.u = np.zeros((1, 1))
        self.y = np.zeros((1, 1))

        # States: angle (rad), angular velocity (rad/s)
        # Inputs: voltage (V)
        # Outputs: angle (rad)
        self.observer = fct.KalmanFilter(
            self.plant, [0.01745, 0.1745329], [0.01], self.dt
        )
        self.feedforward = fct.LinearPlantInversionFeedforward(
            self.plant.A, self.plant.B, self.dt
        )
        self.feedback = fct.LinearQuadraticRegulator(
            self.plant.A, self.plant.B, [0.01745, 0.08726], [12.0], self.dt
        )

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


def main():
    """Entry point."""

    dt = 0.005
    single_jointed_arm = SingleJointedArm(dt)

    ts, xprof, vprof, _ = fct.generate_trapezoid_profile(
        max_v=0.5, time_to_max_v=0.5, dt=dt, goal=1.04
    )

    # Run simulation
    refs = [np.array([[xprof[i]], [vprof[i]]]) for i in range(len(ts))]
    r_rec, x_rec, u_rec, _ = fct.generate_time_responses(single_jointed_arm, refs)

    fct.plot_time_responses(
        ["Angle (rad)", "Angular velocity (rad/s)"],
        ["Voltage (V)"],
        ts,
        r_rec,
        x_rec,
        u_rec,
    )
    if "--noninteractive" in sys.argv:
        latex.savefig("single_jointed_arm_response")
    else:
        plt.show()


if __name__ == "__main__":
    main()
