#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
    import utils.latex as latex

import matplotlib.pyplot as plt
import numpy as np

plt.rc("text", usetex=True)


def main():
    # x_1: robot position measured from corner
    # x_2: robot velocity with positive direction toward wall
    # x_3: wall position measured from corner
    xhat = np.array([[0], [0], [0]])

    # y_1: measurement of distance from robot to corner
    # y_2: measurement of distance from robot to wall
    y = np.genfromtxt("kalman_robot.csv", delimiter=",")

    # fmt: off
    phi = np.array([[1, 1, 0],
                    [0, 0, 0],
                    [0, 0, 1]])
    gamma = np.array([[0],
                      [0.1],
                      [0]])

    Q = np.array([[1]])
    R = np.array([[10, 0],
                   [0, 10]])

    P = np.zeros((3, 3))
    K = np.zeros((3, 2))
    C = np.array([[1, 0, 0],
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
            P = np.array([[10, 10 / dt, 10],
                          [10 / dt, 20 / dt**2, 10 / dt],
                          [10, 10 / dt, 20]])
            # fmt: on

            xhat_rec[:, :, k] = xhat
            P_rec[:, :, k] = np.array([P[0, 0], P[1, 1], P[2, 2]]).T
        elif k > 1:
            # Predict
            xhat = phi @ xhat + np.array([[0, 0.8, 0]]).T
            P = phi @ P @ phi.T + gamma @ Q @ gamma.T

            # Update
            K = P @ C.T @ np.linalg.inv(C @ P @ C.T + R)
            xhat += K @ (y[:, k : k + 1] - C @ xhat)
            P = (np.eye(3, 3) - K @ C) @ P

            xhat_rec[:, :, k] = xhat
            P_rec[:, :, k] = np.array([P[0, 0], P[1, 1], P[2, 2]]).T

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
        latex.savefig("kalman_filter_all")

    # Robot position estimate and variance
    plt.figure(2)
    plt.xlabel("Time (s)")
    plt.plot(t[1:], xhat_rec[0, 0, 1:], label="Robot position estimate (cm)")
    plt.plot(t[1:], P_rec[0, 0, 1:], label="Robot position variance ($cm^2$)")
    plt.legend()
    if "--noninteractive" in sys.argv:
        latex.savefig("kalman_filter_robot_pos")

    # Wall position estimate and variance
    plt.figure(3)
    plt.xlabel("Time (s)")
    plt.plot(t[1:], xhat_rec[2, 0, 1:], label="Wall position estimate (cm)")
    plt.plot(t[1:], P_rec[2, 0, 1:], label="Wall position variance ($cm^2$)")
    plt.legend()
    if "--noninteractive" in sys.argv:
        latex.savefig("kalman_filter_wall_pos")
    else:
        plt.show()


if __name__ == "__main__":
    main()
