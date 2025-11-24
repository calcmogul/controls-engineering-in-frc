#!/usr/bin/env python3

"""Plots differential drive following a motion profile."""

import sys

import frccontrol as fct
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

from bookutil import latex

if "--noninteractive" in sys.argv:
    mpl.use("svg")


class DifferentialDrive:
    """An frccontrol system representing a differential drive."""

    def __init__(self, dt):
        """
        DifferentialDrive subsystem.

        Parameter ``dt``:
            Time between model/controller updates.
        """
        self.dt = dt

        self.in_low_gear = False

        # Number of motors per side
        num_motors = 2.0

        # High and low gear ratios of differential drive
        Glow = 60.0 / 11.0
        Ghigh = 60.0 / 11.0

        # Drivetrain mass in kg
        m = 52
        # Radius of wheels in meters
        r = 0.08255 / 2.0
        # Radius of robot in meters
        rb = 0.59055 / 2.0
        # Moment of inertia of the differential drive in kg-m^2
        J = 6.0

        # Gear ratios of left and right sides of differential drive respectively
        if self.in_low_gear:
            Gl = Glow
            Gr = Glow
        else:
            Gl = Ghigh
            Gr = Ghigh

        self.plant = fct.models.differential_drive(
            fct.models.MOTOR_CIM, num_motors, m, r, rb, J, Gl, Gr
        )

        # Sim variables
        self.sim = self.plant.to_discrete(self.dt)
        self.x = np.zeros((2, 1))
        self.u = np.zeros((2, 1))
        self.y = np.zeros((2, 1))

        # States: left velocity (m/s), right velocity (m/s)
        # Inputs: left voltage (V), right voltage (V)
        # Outputs: left velocity (m/s), right velocity (m/s)
        self.observer = fct.KalmanFilter(self.plant, [1.0, 1.0], [0.01, 0.01], self.dt)
        self.feedforward = fct.LinearPlantInversionFeedforward(
            self.plant.A, self.plant.B, self.dt
        )
        if self.in_low_gear:
            self.feedback = fct.LinearQuadraticRegulator(
                self.plant.A,
                self.plant.B,
                [1.0, 1.0],
                [12.0, 12.0],
                self.dt,
            )
        else:
            self.feedback = fct.LinearQuadraticRegulator(
                self.plant.A,
                self.plant.B,
                [0.95, 0.95],
                [12.0, 12.0],
                self.dt,
            )

        self.u_min = np.array([[-12.0], [-12.0]])
        self.u_max = np.array([[12.0], [12.0]])

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
    diff_drive = DifferentialDrive(dt)

    ts, _, vprof, _ = fct.generate_trapezoid_profile(
        max_v=3.5, time_to_max_v=1.0, dt=dt, goal=50.0
    )

    # Run simulation
    refs = [np.array([[vprof[i]], [vprof[i]]]) for i in range(len(ts))]
    r_rec, x_rec, u_rec, _ = fct.generate_time_responses(diff_drive, refs)

    fct.plot_time_responses(
        [
            "Left velocity (m/s)",
            "Right velocity (m/s)",
        ],
        ["Left voltage (V)", "Right voltage (V)"],
        ts,
        r_rec,
        x_rec,
        u_rec,
    )
    if "--noninteractive" in sys.argv:
        latex.savefig("differential_drive_response")
    else:
        plt.show()


if __name__ == "__main__":
    main()
