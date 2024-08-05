#!/usr/bin/env python3

"""Solves double integrator minimum-time trajectory optimization problem with Jormungandr."""

from pathlib import Path
import re
import sys

from jormungandr.optimization import OptimizationProblem
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

from bookutil import latex

if "--noninteractive" in sys.argv:
    mpl.use("svg")
plt.rc("text", usetex=True)


def main():
    """Entry point."""
    T = 3.5  # s
    dt = 0.005  # 5 ms
    N = int(T / dt)

    r = 2.0

    problem = OptimizationProblem()

    # 2x1 state vector with N + 1 timesteps (includes last state)
    X = problem.decision_variable(2, N + 1)

    # 1x1 input vector with N timesteps (input at last state doesn't matter)
    U = problem.decision_variable(1, N)

    # Kinematics constraint assuming constant acceleration between timesteps
    for k in range(N):
        p_k1 = X[0, k + 1]
        v_k1 = X[1, k + 1]
        p_k = X[0, k]
        v_k = X[1, k]
        a_k = U[0, k]

        problem.subject_to(p_k1 == p_k + v_k * dt + 0.5 * a_k * dt**2)
        problem.subject_to(v_k1 == v_k + a_k * dt)

    # Start and end at rest
    problem.subject_to(X[:, 0] == np.array([[0.0], [0.0]]))
    problem.subject_to(X[:, N] == np.array([[r], [0.0]]))

    # Limit velocity
    problem.subject_to(-1 <= X[1, :])
    problem.subject_to(X[1, :] <= 1)

    # Limit acceleration
    problem.subject_to(-1 <= U)
    problem.subject_to(U <= 1)

    # Cost function - minimize position error
    J = 0.0
    for k in range(N + 1):
        J += (r - X[0, k]) ** 2
    problem.minimize(J)

    problem.solve()

    ts = [k * dt for k in range(N + 1)]

    # Plot states
    labels = ["Position (m)", "Velocity (m/s)"]
    for i in range(X.shape[0]):
        plt.figure()
        ax = plt.gca()

        xs = X.value()[i, :]
        ax.plot(ts, xs, label=labels[i])
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(labels[i])

        unit_name = re.search(r"(\w+)(\(.*?\))?", labels[i]).group(1).lower()
        figname = f"{Path(__file__).stem}_{unit_name}"

        if "--noninteractive" in sys.argv:
            latex.savefig(figname)

    # Plot inputs
    labels = ["Acceleration (m/s²)"]
    for i in range(U.shape[0]):
        plt.figure()
        ax = plt.gca()

        us = np.concatenate((U.value()[i, :], [0.0]))
        ax.plot(ts, us, label=labels[i])
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(labels[i])

        unit_name = re.search(r"(\w+)(\(.*?\))?", labels[i]).group(1).lower()
        figname = f"{Path(__file__).stem}_{unit_name}"

        if "--noninteractive" in sys.argv:
            latex.savefig(figname)

    if "--noninteractive" not in sys.argv:
        plt.show()


if __name__ == "__main__":
    main()
