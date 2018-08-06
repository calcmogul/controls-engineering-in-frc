#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
    import latexutils

import matplotlib.pyplot as plt
import numpy as np

plt.rc("text", usetex=True)


def main():
    # x_1: robot position measured from corner
    # x_2: robot velocity with positive direction toward wall
    # x_3: wall position measured from corner
    xhat = np.matrix([[0], [0], [0]])

    # y_1: measurement of distance from robot to corner
    # y_2: measurement of distance from robot to wall
    y = []
    import csv

    with open("kalman_robot.csv", newline="") as data:
        reader = csv.reader(data)
        for i, row in enumerate(reader):
            yrow = np.asmatrix([float(x) for x in row])
            if y == []:
                y = yrow
            else:
                y = np.concatenate((y, yrow))

    # fmt: off
    phi = np.matrix([[1, 1, 0],
                     [0, 0, 0],
                     [0, 0, 1]])
    gamma = np.matrix([[0],
                       [0.1],
                       [0]])

    Q = np.matrix([[1]])
    R = np.matrix([[10, 0],
                   [0, 10]])

    P = np.zeros((3, 3));
    K = np.zeros((3, 2));
    H = np.matrix([[1, 0, 0],
                   [-1, 0, 1]])
    # fmt: on

    num_points = y.shape[1]
    xhat_rec = np.zeros((3, 1, num_points))
    P_rec = np.zeros((3, 3, num_points))
    t = np.linspace(0, num_points, num_points)

    dt = 1

    for k in range(100):
        if k == 1:
            xhat[0] = y[0, k]
            xhat[1] = (y[0, 1] - y[0, 0]) / dt
            xhat[2] = y[0, k] + y[1, k]
            # fmt: off
            P = np.matrix([[10, 10 / dt, 10],
                           [10 / dt, 20 / dt**2, 10 / dt],
                           [10, 10 / dt, 20]])
            # fmt: on

            xhat_rec[:, :, k] = xhat
            Ptemp = np.matrix([P[0, 0], P[1, 1], P[2, 2]]).T
            P_rec[:, :, k] = Ptemp
        elif k > 1:
            # Predict
            xhat = phi * xhat + np.matrix([0, 0.8, 0]).T
            P = phi * P * phi.T + gamma * Q * gamma.T

            # Update
            K = P * H.T * np.linalg.inv(H * P * H.T + R)
            xhat = xhat + K * (y[:, k] - H * xhat)
            P = (np.eye(3, 3) - K * H) * P

            xhat_rec[:, :, k] = xhat
            Ptemp = np.matrix([P[0, 0], P[1, 1], P[2, 2]]).T
            P_rec[:, :, k] = Ptemp

    # State estimates and measurements
    plt.figure(1)
    plt.xlabel("Time (s)")
    plt.plot(t[1:], xhat_rec[0, 0, 1:], label="Robot position estimate (cm)")
    plt.plot(t[1:], xhat_rec[1, 0, 1:], label="Robot velocity estimate (cm/s)")
    plt.plot(t[1:], xhat_rec[2, 0, 1:], label="Wall position estimate (cm)")
    plt.plot(t[1:], y[0, 1:].T, label="Robot to corner measurement (cm)")
    plt.plot(t[1:], y[1, 1:].T, label="Robot to wall measurement (cm)")
    plt.legend()
    if "--noninteractive" in sys.argv:
        latexutils.savefig("kalman_filter_all")

    # Robot position estimate and variance
    plt.figure(2)
    plt.xlabel("Time (s)")
    plt.plot(t[1:], xhat_rec[0, 0, 1:], label="Robot position estimate (cm)")
    plt.plot(t[1:], P_rec[0, 0, 1:], label="Robot position variance ($cm^2$)")
    plt.legend()
    if "--noninteractive" in sys.argv:
        latexutils.savefig("kalman_filter_robot_pos")

    # Wall position estimate and variance
    plt.figure(3)
    plt.xlabel("Time (s)")
    plt.plot(t[1:], xhat_rec[2, 0, 1:], label="Wall position estimate (cm)")
    plt.plot(t[1:], P_rec[2, 0, 1:], label="Wall position variance ($cm^2$)")
    plt.legend()
    if "--noninteractive" in sys.argv:
        latexutils.savefig("kalman_filter_wall_pos")
    else:
        plt.show()


if __name__ == "__main__":
    main()
