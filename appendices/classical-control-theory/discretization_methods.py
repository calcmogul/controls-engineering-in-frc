#!/usr/bin/env python3

"""Plots for various discretization methods."""

import math
import sys

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

from bookutil import latex
from bookutil.systems import Elevator

if "--noninteractive" in sys.argv:
    mpl.use("svg")
plt.rc("text", usetex=True)


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

    state_rec = np.zeros((2, 0))

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
        state_rec = np.concatenate((state_rec, elevator.observer.x_hat), axis=1)

        r = next_r

    pos = state_rec[0, :]
    vel = state_rec[1, :]

    plt.figure(1)
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.plot(ts, vel, label="Continuous")
    y = generate_forward_euler_vel(vel, dt, sample_period)
    plt.plot(ts, y, label=f"Forward Euler (T={sample_period} s)")
    y = generate_backward_euler_vel(vel, dt, sample_period)
    plt.plot(ts, y, label=f"Backward Euler (T={sample_period} s)")
    y = generate_bilinear_transform_vel(vel, dt, sample_period)
    plt.plot(ts, y, label=f"Bilinear transform (T={sample_period} s)")
    plt.legend()
    if "--noninteractive" in sys.argv:
        latex.savefig("discretization_methods_vel")

    plt.figure(2)
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    plt.plot(ts, pos, label="Continuous")
    y = generate_forward_euler_pos(vel, dt, sample_period)
    plt.plot(ts, y, label=f"Forward Euler (T={sample_period} s)")
    y = generate_backward_euler_pos(vel, dt, sample_period)
    plt.plot(ts, y, label=f"Backward Euler (T={sample_period} s)")
    y = generate_bilinear_transform_pos(vel, dt, sample_period)
    plt.plot(ts, y, label=f"Bilinear transform (T={sample_period} s)")
    plt.legend()
    if "--noninteractive" in sys.argv:
        latex.savefig("discretization_methods_pos")
    else:
        plt.show()


if __name__ == "__main__":
    main()
