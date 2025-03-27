#!/usr/bin/env python3

"""Applies a Kalman filter to a robot's position."""

import sys

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

from bookutil import latex

if "--noninteractive" in sys.argv:
    mpl.use("svg")


def main():
    """Entry point."""
    # y₁: measurement of distance from robot to corner
    # y₂: measurement of distance from robot to wall
    # fmt: off
    y = np.array([
        [43.794, 57.633, 52.501, 47.741, 42.675, 61.580, 64.782, 62.067, 64.680,
         61.434, 53.198, 66.292, 48.886, 64.984, 55.556, 44.826, 53.479, 60.092,
         39.295, 58.557, 63.173, 72.700, 51.723, 72.837, 70.389, 63.027, 71.464,
         91.281, 57.632, 58.057, 79.159, 69.740, 51.656, 80.298, 62.199, 73.933,
         68.028, 87.307, 76.717, 81.176, 67.610, 71.928, 93.884, 82.448, 83.704,
         89.545, 84.565, 70.185, 80.299, 90.925, 67.683, 92.527, 102.98, 87.527,
         97.250, 104.54, 92.676, 77.447, 98.222, 99.823, 106.24, 90.743, 100.63,
         92.184, 110.30, 91.163, 118.74, 110.76, 125.61, 109.46, 99.350, 111.59,
         106.71, 107.65, 118.38, 102.45, 131.54, 106.53, 122.69, 112.80, 118.29,
         126.16, 100.79, 115.22, 111.41, 108.93, 110.93, 121.57, 125.68, 104.17,
         120.86, 100.90, 128.10, 137.61, 135.10, 108.85, 103.51, 116.52, 125.40,
         127.54],
        [153.51, 137.85, 152.47, 132.88, 144.13, 136.21, 155.70, 143.57, 141.12,
         119.06, 131.40, 135.97, 151.06, 130.23, 130.15, 137.39, 144.58, 142.68,
         129.01, 122.38, 147.95, 133.05, 126.03, 124.58, 126.55, 124.63, 136.21,
         130.98, 130.12, 123.73, 147.10, 140.62, 124.71, 123.68, 149.11, 126.48,
         123.43, 143.27, 93.710, 120.66, 115.41, 118.81, 125.63, 131.87, 117.63,
         118.09, 104.58, 106.65, 115.46, 116.09, 121.28, 120.02, 116.99, 120.94,
         135.00, 106.72, 96.423, 117.97, 113.74, 95.268, 95.429, 110.81, 84.145,
         88.162, 84.426, 97.524, 102.06, 106.31, 95.149, 82.980, 108.91, 103.69,
         79.461, 109.36, 110.04, 102.23, 99.369, 94.954, 92.398, 72.765, 88.895,
         96.450, 95.906, 87.688, 99.813, 78.470, 71.965, 103.65, 80.510, 82.427,
         75.490, 90.630, 83.955, 65.385, 75.600, 78.440, 71.732, 79.109, 67.313,
         75.403]
    ])
    # fmt: on
    measurements = y.shape[1]

    A = np.array([[1, 1, 0], [0, 0, 0], [0, 0, 1]])
    C = np.array([[1, 0, 0], [-1, 0, 1]])

    dt = 1
    P = np.array([[10, 10 / dt, 10], [10 / dt, 20 / dt**2, 10 / dt], [10, 10 / dt, 20]])
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
    plt.plot(t, [P[0, 0] for P in P_rec], label="Robot position variance (cm²)")
    plt.legend()
    if "--noninteractive" in sys.argv:
        latex.savefig("kalman_filter_robot_pos")

    # Wall position estimate and variance
    plt.figure(3)
    plt.xlabel("Time (s)")
    plt.plot(
        t, [x_hat[2, 0] for x_hat in x_hat_rec], label="Wall position estimate (cm)"
    )
    plt.plot(t, [P[2, 0] for P in P_rec], label="Wall position variance (cm²)")
    plt.legend()
    if "--noninteractive" in sys.argv:
        latex.savefig("kalman_filter_wall_pos")
    else:
        plt.show()


if __name__ == "__main__":
    main()
