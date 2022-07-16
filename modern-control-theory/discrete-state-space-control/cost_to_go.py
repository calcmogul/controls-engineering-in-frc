#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
    import bookutil.latex as latex

import math
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np
from scipy.linalg import solve_discrete_are
from scipy.signal import StateSpace

plt.rc("text", usetex=True)


def main():
    dt = 0.005

    contA = np.array([[0, 1], [0, -164.209]])
    contB = np.array([[0], [21.457]])
    C = np.array([[1, 0]])
    D = np.array([[0]])
    system = StateSpace(contA, contB, C, D).to_discrete(dt)

    A = system.A
    B = system.B
    Q = np.diag(1.0 / np.square([0.02, 0.4]))
    R = np.diag(1.0 / np.square([12]))
    P = solve_discrete_are(A, B, Q, R)

    K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
    K_p = f"{np.round(K[0, 0], 3)}"
    K_d = f"{np.round(K[0, 1], 3)}"
    if "--noninteractive" in sys.argv:
        with open("cost-to-go-kp.tex", "w") as f:
            f.write(f"$K_p = {K_p}$\n")
        with open("cost-to-go-kd.tex", "w") as f:
            f.write(f"$K_d={K_d}$\n")
    else:
        print(f"K_p={K_p}, K_d={K_d}")

    xmin = -1
    xmax = 1
    ymin = -1
    ymax = 1
    stepsize = 0.02
    xsteps = int((xmax - xmin) / stepsize)
    ysteps = int((ymax - ymin) / stepsize)
    X, Y = np.meshgrid(np.linspace(xmin, xmax, xsteps), np.linspace(ymin, ymax, ysteps))

    Z = np.empty(X.shape)
    for i in range(Z.shape[0]):
        for j in range(Z.shape[1]):
            x = np.array([[X[i][j]], [Y[i][j]]])
            # Optimal cost-to-go for linear system is xᵀPx
            Z[i][j] = np.squeeze(x.T @ P @ x)

    ax = plt.axes(projection="3d")
    ax.set_xlabel("Position")
    ax.set_ylabel("Velocity")
    ax.set_zlabel("Cost-to-go")
    surf = ax.plot_surface(X, Y, Z, cmap="viridis")
    surf._facecolors2d = surf._facecolor3d
    surf._edgecolors2d = surf._edgecolor3d
    if "--noninteractive" in sys.argv:
        latex.savefig("cost_to_go")
    else:
        plt.show()


if __name__ == "__main__":
    main()
