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

    # yapf: disable
    phi = np.matrix([[1, 1, 0],
                     [0, 1, 0],
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
    # yapf: enable

    # Initialize matrix storage
    xhat_pre_rec = np.zeros((3, 1, y.shape[1]))
    xhat_post_rec = np.zeros((3, 1, y.shape[1]))
    P_pre_rec = np.zeros((3, 3, y.shape[1]))
    P_post_rec = np.zeros((3, 3, y.shape[1]))
    xhat_smooth_rec = np.zeros((3, 1, y.shape[1]))
    P_smooth_rec = np.zeros((3, 3, y.shape[1]))
    t = [0] * (y.shape[1])

    # Forward Kalman filter

    # Set up initial conditions for first measurement
    xhat[0] = y[0, 0]
    xhat[1] = 0.2
    xhat[2] = y[0, 0] + y[1, 0]
    # yapf: disable
    P = np.matrix([[10, 0, 10],
                   [0, 1, 0],
                   [10, 0, 20]])
    # yapf: enable

    xhat_pre_rec[:, :, 1] = xhat
    P_pre_rec[:, :, 1] = P
    xhat_post_rec[:, :, 1] = xhat
    P_post_rec[:, :, 1] = P
    t[1] = 1

    for k in range(2, 100):
        # Predict
        xhat = phi * xhat
        P = phi * P * phi.T + gamma * Q * gamma.T

        xhat_pre_rec[:, :, k] = xhat
        P_pre_rec[:, :, k] = P

        # Update
        K = P * H.T * np.linalg.inv(H * P * H.T + R)
        xhat = xhat + K * (y[:, k] - H * xhat)
        P = (np.eye(3, 3) - K * H) * P

        xhat_post_rec[:, :, k] = xhat
        P_post_rec[:, :, k] = P
        t[k] = k

    # Kalman smoother

    # Last estimate is already optional, so add it to the record
    xhat_smooth_rec[:, :, -1] = xhat_post_rec[:, :, -1]
    P_smooth_rec[:, :, -1] = P_post_rec[:, :, -1]

    for k in range(y.shape[1] - 2, 0, -1):
        A = P_post_rec[:, :, k] * phi.T * np.linalg.inv(P_pre_rec[:, :, k + 1])
        xhat = xhat_post_rec[:, :, k] + A * (
            xhat_smooth_rec[:, :, k + 1] - xhat_pre_rec[:, :, k + 1])
        P = P_post_rec[:, :, k] + A * (P - P_pre_rec[:, :, k + 1]) * A.T

        xhat_smooth_rec[:, :, k] = xhat
        P_smooth_rec[:, :, k] = P

    # Robot position
    plt.figure(1)
    plt.xlabel("Time (s)")
    plt.ylabel("Position (cm)")
    plt.plot(t[1:], xhat_post_rec[0, 0, 1:], label="Kalman filter")
    plt.plot(t[1:], xhat_smooth_rec[0, 0, 1:], label="Kalman smoother")
    plt.legend()
    if "--noninteractive" in sys.argv:
        latexutils.savefig("kalman_smoother_robot_pos")

    # Robot velocity
    plt.figure(2)
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (cm)")
    plt.plot(t[1:], xhat_post_rec[1, 0, 1:], label="Kalman filter")
    plt.plot(t[1:], xhat_smooth_rec[1, 0, 1:], label="Kalman smoother")
    plt.legend()
    if "--noninteractive" in sys.argv:
        latexutils.savefig("kalman_smoother_robot_vel")

    # Wall position
    plt.figure(3)
    plt.xlabel("Time (s)")
    plt.ylabel("Wall position (cm)")
    plt.plot(t[1:], xhat_post_rec[2, 0, 1:], label="Kalman filter")
    plt.plot(t[1:], xhat_smooth_rec[2, 0, 1:], label="Kalman smoother")
    plt.legend()
    if "--noninteractive" in sys.argv:
        latexutils.savefig("kalman_smoother_wall_pos")

    # Robot position variance
    plt.figure(4)
    plt.xlabel("Time (s)")
    plt.ylabel("Robot position variance ($cm^2$)")
    plt.plot(t[1:], P_post_rec[1, 1, 1:], label="Kalman filter")
    plt.plot(t[1:], P_smooth_rec[1, 1, 1:], label="Kalman smoother")
    plt.legend()
    if "--noninteractive" in sys.argv:
        latexutils.savefig("kalman_smoother_robot_pos_variance")
    else:
        plt.show()


if __name__ == "__main__":
    main()
