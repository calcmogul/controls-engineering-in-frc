#!/usr/bin/env python3

"""Applies a Kalman filter to a robot's position."""

import sys

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

from bookutil import latex

if "--noninteractive" in sys.argv:
    mpl.use("svg")
plt.rc("text", usetex=True)


def main():
    """Entry point."""
    # y₁: measurement of distance from robot to corner
    # y₂: measurement of distance from robot to wall
    y = np.genfromtxt("kalman_robot.csv", delimiter=",")
    measurements = y.shape[1]

    A = np.array([[1, 1, 0], [0, 0, 0], [0, 0, 1]])
    C = np.array([[1, 0, 0], [-1, 0, 1]])

    dt = 1
    P = np.array(
        [[10, 10 / dt, 10], [10 / dt, 20 / dt**2, 10 / dt], [10, 10 / dt, 20]]
    )
    Q = np.diag(np.square([0, 0.1, 0]))
    R = np.array([[10, 0], [0, 10]])

    # Kalman filter storage
    x_hat_rec = [np.zeros((3, 1)) for _ in range(measurements)]
    P_rec = [np.zeros((3, 3)) for _ in range(measurements)]

    # x₁: robot position measured from corner
    # x₂: robot velocity with positive direction toward wall
    # x₃: wall position measured from corner
    x_hat = np.array([[y[0, 0]], [(y[0, 1] - y[0, 0]) / dt], [y[0, 0] + y[1, 0]]])

    x_hat_rec[0] = x_hat
    P_rec[0] = P

    for k in range(1, measurements):
        # Predict
        x_hat = A @ x_hat + np.array([[0, 0.8, 0]]).T
        P = A @ P @ A.T + Q

        # Update
        K = P @ C.T @ np.linalg.inv(C @ P @ C.T + R)
        x_hat += K @ (y[:, k : k + 1] - C @ x_hat)
        P = (np.eye(3) - K @ C) @ P

        x_hat_rec[k] = x_hat
        P_rec[k] = P

    t = np.linspace(0, measurements, measurements)

    # State estimates and measurements
    plt.figure(1)
    plt.xlabel("Time (s)")
    plt.plot(
        t, [x_hat[0, 0] for x_hat in x_hat_rec], label="Robot position estimate (cm)"
    )
    plt.plot(
        t, [x_hat[1, 0] for x_hat in x_hat_rec], label="Robot velocity estimate (cm/s)"
    )
    plt.plot(
        t, [x_hat[2, 0] for x_hat in x_hat_rec], label="Wall position estimate (cm)"
    )
    plt.plot(t, y[0, :], label="Robot to corner measurement (cm)")
    plt.plot(t, y[1, :], label="Robot to wall measurement (cm)")
    plt.legend()
    if "--noninteractive" in sys.argv:
        latex.savefig("kalman_filter_all")

    # Robot position estimate and variance
    plt.figure(2)
    plt.xlabel("Time (s)")
    plt.plot(
        t, [x_hat[0, 0] for x_hat in x_hat_rec], label="Robot position estimate (cm)"
    )
    plt.plot(t, [P[0, 0] for P in P_rec], label="Robot position variance ($cm^2$)")
    plt.legend()
    if "--noninteractive" in sys.argv:
        latex.savefig("kalman_filter_robot_pos")

    # Wall position estimate and variance
    plt.figure(3)
    plt.xlabel("Time (s)")
    plt.plot(
        t, [x_hat[2, 0] for x_hat in x_hat_rec], label="Wall position estimate (cm)"
    )
    plt.plot(t, [P[2, 0] for P in P_rec], label="Wall position variance ($cm^2$)")
    plt.legend()
    if "--noninteractive" in sys.argv:
        latex.savefig("kalman_filter_wall_pos")
    else:
        plt.show()


if __name__ == "__main__":
    main()
