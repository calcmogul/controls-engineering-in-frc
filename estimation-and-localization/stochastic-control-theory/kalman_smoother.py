#!/usr/bin/env python3

"""Applies a Kalman smoother to a robot's position."""

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

    A = np.array([[1, 1, 0], [0, 1, 0], [0, 0, 1]])
    C = np.array([[1, 0, 0], [-1, 0, 1]])

    Q = np.diag(np.square([0, 0.1, 0]))
    R = np.array([[10, 0], [0, 10]])

    # Kalman smoother storage
    x_hat_pre = [np.zeros((3, 1)) for _ in range(measurements)]
    x_hat_post = [np.zeros((3, 1)) for _ in range(measurements)]
    P_pre = [np.zeros((3, 3)) for _ in range(measurements)]
    P_post = [np.zeros((3, 3)) for _ in range(measurements)]
    x_hat_smooth = [np.zeros((3, 1)) for _ in range(measurements)]
    P_smooth = [np.zeros((3, 3)) for _ in range(measurements)]

    # x₁: robot position measured from corner
    # x₂: robot velocity with positive direction toward wall
    # x₃: wall position measured from corner
    x_hat_pre[0] = np.array([[y[0, 0]], [0.2], [y[0, 0] + y[1, 0]]])
    x_hat_post[0] = np.array([[y[0, 0]], [0.2], [y[0, 0] + y[1, 0]]])

    P_pre[0] = np.array([[10, 0, 10], [0, 1, 0], [10, 0, 20]])
    P_post[0] = np.array([[10, 0, 10], [0, 1, 0], [10, 0, 20]])

    # Filter
    for k in range(measurements - 1):
        # Predict
        x_hat_pre[k + 1] = A @ x_hat_post[k]
        P_pre[k + 1] = A @ P_post[k] @ A.T + Q

        # Update
        K = P_pre[k + 1] @ C.T @ np.linalg.inv(C @ P_pre[k + 1] @ C.T + R)
        x_hat_post[k + 1] = x_hat_pre[k + 1] + K @ (
            y[:, k + 1 : k + 2] - C @ x_hat_pre[k + 1]
        )
        P_post[k + 1] = (np.eye(3) - K @ C) @ P_pre[k + 1]

    # Last filtered estimate is already optimal smoothed estimate
    x_hat_smooth[-1] = x_hat_post[-1]
    P_smooth[-1] = P_post[-1]

    # Smoother
    for k in range(measurements - 2, -1, -1):
        K = P_post[k] @ A.T @ np.linalg.inv(P_pre[k + 1])
        x_hat_smooth[k] = x_hat_post[k] + K @ (x_hat_smooth[k + 1] - x_hat_pre[k + 1])
        P_smooth[k] = P_post[k] + K @ (P_smooth[k + 1] - P_pre[k + 1]) @ K.T

    t = np.linspace(0, measurements, measurements)

    # Robot position
    plt.figure(1)
    plt.xlabel("Time (s)")
    plt.ylabel("Position (cm)")
    plt.plot(t, [x_hat[0, 0] for x_hat in x_hat_post], label="Kalman filter")
    plt.plot(t, [x_hat[0, 0] for x_hat in x_hat_smooth], label="Kalman smoother")
    plt.legend()
    if "--noninteractive" in sys.argv:
        latex.savefig("kalman_smoother_robot_pos")

    # Robot velocity
    plt.figure(2)
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (cm/s)")
    plt.plot(t, [x_hat[1, 0] for x_hat in x_hat_post], label="Kalman filter")
    plt.plot(t, [x_hat[1, 0] for x_hat in x_hat_smooth], label="Kalman smoother")
    plt.legend()
    if "--noninteractive" in sys.argv:
        latex.savefig("kalman_smoother_robot_vel")

    # Wall position
    plt.figure(3)
    plt.xlabel("Time (s)")
    plt.ylabel("Wall position (cm)")
    plt.plot(t, [x_hat[2, 0] for x_hat in x_hat_post], label="Kalman filter")
    plt.plot(t, [x_hat[2, 0] for x_hat in x_hat_smooth], label="Kalman smoother")
    plt.legend()
    if "--noninteractive" in sys.argv:
        latex.savefig("kalman_smoother_wall_pos")

    # Robot position variance
    plt.figure(4)
    plt.xlabel("Time (s)")
    plt.ylabel("Robot position variance ($cm^2$)")
    plt.plot(t, [P[1, 1] for P in P_post], label="Kalman filter")
    plt.plot(t, [P[1, 1] for P in P_smooth], label="Kalman smoother")
    plt.legend()
    if "--noninteractive" in sys.argv:
        latex.savefig("kalman_smoother_robot_pos_variance")
    else:
        plt.show()


if __name__ == "__main__":
    main()
