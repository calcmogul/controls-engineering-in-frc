#!/usr/bin/env python3

"""Plots for various discretization methods."""

import math
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

        Keyword arguments:
        r -- the current reference
        next_r -- the next reference
        """
        # Update sim model
        self.x = self.sim.A @ self.x + self.sim.B @ self.u
        self.y = self.sim.C @ self.x + self.sim.D @ self.u

        self.u = np.clip(
            self.feedforward.calculate(next_r) + self.feedback.calculate(self.x, r),
            self.u_min,
            self.u_max,
        )


def generate_forward_euler_vel(data, dt, sample_period):
    """Generates forward Euler approximation of data set.

    Keyword arguments:
    data -- array of velocity data
    dt -- dt of original data samples
    sample_period -- desired time between samples in approximation
    """
    y = []
    val = 0
    for i in range(len(data)):
        # t1 = int(math.floor(i * dt / sample_period) * sample_period / dt)
        t2 = int((math.floor(i * dt / sample_period) + 1) * sample_period / dt)
        if t2 < len(data):
            val = data[t2]
        y.append(val)
    return y


def generate_backward_euler_vel(data, dt, sample_period):
    """Generates backward Euler approximation of data set.

    Keyword arguments:
    data -- array of velocity data
    dt -- dt of original data samples
    sample_period -- desired time between samples in approximation
    """
    y = []
    val = 0
    for i in range(len(data)):
        t1 = int(math.floor(i * dt / sample_period) * sample_period / dt)
        # t2 = int((math.floor(i * dt / sample_period) + 1) * sample_period / dt)
        if t1 < len(data):
            val = data[t1]
        y.append(val)
    return y


def generate_bilinear_transform_vel(data, dt, sample_period):
    """Generates bilinear transform approximation of data set.

    Keyword arguments:
    data -- array of velocity data
    dt -- dt of original data samples
    sample_period -- desired time between samples in approximation
    """
    y = []
    val = 0
    for i in range(len(data)):
        t1 = int(math.floor(i * dt / sample_period) * sample_period / dt)
        t2 = int((math.floor(i * dt / sample_period) + 1) * sample_period / dt)
        if t2 < len(data):
            alpha = (i - t1) / (t2 - t1)
            val = (1 - alpha) * data[t1] + alpha * data[t2]
        y.append(val)
    return y


def generate_forward_euler_pos(data, dt, sample_period):
    """Generates forward Euler approximation of data set.

    Keyword arguments:
    data -- array of velocity data
    dt -- dt of original data samples
    sample_period -- desired time between samples in approximation
    """
    y = []
    val = 0
    for i in range(len(data)):
        # t1 = int(math.floor(i * dt / sample_period) * sample_period / dt)
        t2 = int((math.floor(i * dt / sample_period) + 1) * sample_period / dt)
        if t2 < len(data):
            val += data[t2] * dt
        y.append(val)
    return y


def generate_backward_euler_pos(data, dt, sample_period):
    """Generates backward Euler approximation of data set.

    Keyword arguments:
    data -- array of velocity data
    dt -- dt of original data samples
    sample_period -- desired time between samples in approximation
    """
    y = []
    val = 0
    for i in range(len(data)):
        t1 = int(math.floor(i * dt / sample_period) * sample_period / dt)
        # t2 = int((math.floor(i * dt / sample_period) + 1) * sample_period / dt)
        if t1 < len(data):
            val += data[t1] * dt
        y.append(val)
    return y


def generate_bilinear_transform_pos(data, dt, sample_period):
    """Generates bilinear transform approximation of data set.

    Keyword arguments:
    data -- array of velocity data
    dt -- dt of original data samples
    sample_period -- desired time between samples in approximation
    """
    y = []
    val = 0
    for i in range(len(data)):
        t1 = int(math.floor(i * dt / sample_period) * sample_period / dt)
        t2 = int((math.floor(i * dt / sample_period) + 1) * sample_period / dt)
        if t2 < len(data):
            alpha = (i - t1) / (t2 - t1)
            val += ((1 - alpha) * data[t1] + alpha * data[t2]) * dt
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

    x_rec = np.zeros((2, 0))

    # Run simulation
    for t in ts:
        if t < l0:
            r = np.array([[0.0], [0.0]])
        elif t < l1:
            r = np.array([[1.524], [0.0]])
        else:
            r = np.array([[0.0], [0.0]])

        if t + dt < l0:
            next_r = np.array([[0.0], [0.0]])
        elif t + dt < l1:
            next_r = np.array([[1.524], [0.0]])
        else:
            next_r = np.array([[0.0], [0.0]])

        elevator.update(r, next_r)

        # Log states for plotting
        x_rec = np.concatenate((x_rec, elevator.x), axis=1)

        r = next_r

    plt.figure(1)
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.plot(ts, x_rec[1, :], label="Continuous")
    plt.plot(
        ts,
        generate_forward_euler_vel(x_rec[1, :], dt, sample_period),
        label=f"Forward Euler (T={sample_period} s)",
    )
    plt.plot(
        ts,
        generate_backward_euler_vel(x_rec[1, :], dt, sample_period),
        label=f"Backward Euler (T={sample_period} s)",
    )
    plt.plot(
        ts,
        generate_bilinear_transform_vel(x_rec[1, :], dt, sample_period),
        label=f"Bilinear transform (T={sample_period} s)",
    )
    plt.legend()
    if "--noninteractive" in sys.argv:
        latex.savefig("discretization_methods_vel")

    plt.figure(2)
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    plt.plot(ts, x_rec[0, :], label="Continuous")
    plt.plot(
        ts,
        generate_forward_euler_pos(x_rec[1, :], dt, sample_period),
        label=f"Forward Euler (T={sample_period} s)",
    )
    plt.plot(
        ts,
        generate_backward_euler_pos(x_rec[1, :], dt, sample_period),
        label=f"Backward Euler (T={sample_period} s)",
    )
    plt.plot(
        ts,
        generate_bilinear_transform_pos(x_rec[1, :], dt, sample_period),
        label=f"Bilinear transform (T={sample_period} s)",
    )
    plt.legend()
    if "--noninteractive" in sys.argv:
        latex.savefig("discretization_methods_pos")
    else:
        plt.show()


if __name__ == "__main__":
    main()
