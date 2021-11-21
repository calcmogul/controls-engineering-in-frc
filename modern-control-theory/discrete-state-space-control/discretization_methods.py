#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
    import bookutil.latex as latex

import matplotlib.pyplot as plt
import math
import numpy as np

from bookutil.systems import Elevator

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
        t1 = int(math.floor(i * dt / sample_period) * sample_period / dt)
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
        t2 = int((math.floor(i * dt / sample_period) + 1) * sample_period / dt)
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
        t1 = int(math.floor(i * dt / sample_period) * sample_period / dt)
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
        t2 = int((math.floor(i * dt / sample_period) + 1) * sample_period / dt)
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
    dt = 0.005
    sample_period = 0.1
    elevator = Elevator(dt)

    # Set up graphing
    l0 = 0.1
    l1 = l0 + 5.0
    l2 = l1 + 0.1
    l3 = l2 + 1.0
    t = np.arange(0, l3, dt)

    refs = []

    # Generate references for simulation
    for i in range(len(t)):
        if t[i] < l0:
            r = np.array([[0.0], [0.0]])
        elif t[i] < l1:
            r = np.array([[1.524], [0.0]])
        else:
            r = np.array([[0.0], [0.0]])
        refs.append(r)

    state_rec, ref_rec, u_rec, y_rec = elevator.generate_time_responses(t, refs)
    pos = elevator.extract_row(state_rec, 0)
    vel = elevator.extract_row(state_rec, 1)

    plt.figure(1)
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.plot(t, vel, label="Continuous")
    y = generate_forward_euler_vel(vel, dt, sample_period)
    plt.plot(t, y, label="Forward Euler (T={}s)".format(sample_period))
    y = generate_backward_euler_vel(vel, dt, sample_period)
    plt.plot(t, y, label="Backward Euler (T={}s)".format(sample_period))
    y = generate_bilinear_transform_vel(vel, dt, sample_period)
    plt.plot(t, y, label="Bilinear transform (T={}s)".format(sample_period))
    plt.legend()
    if "--noninteractive" in sys.argv:
        latex.savefig("discretization_methods_vel")

    plt.figure(2)
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    plt.plot(t, pos, label="Continuous")
    y = generate_forward_euler_pos(vel, dt, sample_period)
    plt.plot(t, y, label="Forward Euler (T={}s)".format(sample_period))
    y = generate_backward_euler_pos(vel, dt, sample_period)
    plt.plot(t, y, label="Backward Euler (T={}s)".format(sample_period))
    y = generate_bilinear_transform_pos(vel, dt, sample_period)
    plt.plot(t, y, label="Bilinear transform (T={}s)".format(sample_period))
    plt.legend()
    if "--noninteractive" in sys.argv:
        latex.savefig("discretization_methods_pos")
    else:
        plt.show()


if __name__ == "__main__":
    main()
