#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
    import bookutil.latex as latex

import math
import matplotlib.pyplot as plt
import numpy as np

plt.rc("text", usetex=True)

T = 1.0
STEPS = 400


def z_to_rgb(z):
    """Returns RGB color for given complex number."""
    # arg(z) ∈ [-π, π]
    arg = math.atan2(z.imag, z.real)

    # We want the following angle to color domain mappings:
    #   [π/2, π] → [0, 0.5]
    #   [-π, -π/2] → [0.5, 1]
    if arg >= 0.0:
        # Add 1/255 so first slot in colormap isn't used
        hue = arg / math.pi - 0.5 + 1 / 255
    elif arg < 0.0:
        hue = arg / math.pi + 1.5
    return hue


def exp_map(x, y):
    z = complex(x, y)
    z = np.exp(z * T)
    return z.real, z.imag


def plot_phase_lines(ax, transform=None):
    """Plots phase lines.

    Keyword arguments:
    ax -- Axes object
    transform -- x_new, y_new = f(x, y)
    """
    xmin = -2.0 * math.pi
    xmax = 0
    ymin = -math.pi
    ymax = math.pi

    angles = (
        np.array(
            [
                17 / 64,
                18 / 64,
                20 / 64,
                24 / 64,
                32 / 64,
                40 / 64,
                44 / 64,
                46 / 64,
                47 / 64,
            ]
        )
        * math.tau
    )

    for angle in angles:
        steps = int(STEPS / abs(math.cos(angle)))
        line_x = np.linspace(0, xmin, steps)
        line_y = np.array([x * math.tan(angle) for x in line_x])

        # Remove points outside y range
        while line_y[-1] < ymin or line_y[-1] > ymax:
            line_x = np.delete(line_x, -1)
            line_y = np.delete(line_y, -1)

        if transform:
            for i in range(len(line_x)):
                line_x[i], line_y[i] = transform(line_x[i], line_y[i])

        ax.plot(line_x, line_y, color=[1, 1, 1])


def main():
    xmin = -2.0 * math.pi
    xmax = 0
    ymin = -math.pi
    ymax = math.pi

    # Remap first slot of colormap to white for out-of-range complex numbers
    cmap = plt.get_cmap("viridis")
    cmap.colors[0] = [1, 1, 1]

    # Generate continuous pole locations and give them colors
    X, Y = np.mgrid[xmin : xmax : STEPS * 1j, ymin : ymax : STEPS * 1j]
    Z = np.empty(X.shape)
    for i in range(X.shape[0]):
        for j in range(Y.shape[1]):
            z = complex(X[i][j], Y[i][j])
            Z[i][j] = z_to_rgb(z)

    plt.figure(1)
    ax = plt.gca()
    ax.set_xlabel("Re")
    ax.set_ylabel("Im")

    # Draw continuous poles
    ax.pcolormesh(
        X, Y, Z, cmap=cmap, vmin=0, vmax=1, shading="gouraud", rasterized=True
    )

    plot_phase_lines(ax)

    # Give axes same scale
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    ax.set_aspect(abs(x1 - x0) / abs(y1 - y0))

    if "--noninteractive" in sys.argv:
        latex.savefig("s_plane")

    xmin = -1.05
    xmax = 1.05
    ymin = -1.05
    ymax = 1.05

    # Map continuous pole locations to discrete
    X, Y = np.mgrid[xmin : xmax : STEPS * 1j, ymin : ymax : STEPS * 1j]
    Z = np.empty(X.shape)
    for i in range(X.shape[0]):
        for j in range(Y.shape[1]):
            z = complex(X[i][j], Y[i][j])
            # z = e^sT
            if abs(z) <= 1:
                Z[i][j] = z_to_rgb(np.log(z) / T)
            else:
                Z[i][j] = 0

    plt.figure(2)
    ax = plt.gca()
    ax.set_xlabel("Re")
    ax.set_ylabel("Im")

    # Draw discrete poles
    ax.pcolormesh(
        X, Y, Z, cmap=cmap, vmin=0, vmax=1, shading="gouraud", rasterized=True
    )

    plot_phase_lines(ax, exp_map)

    # Give axes same scale
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    ax.set_aspect(abs(x1 - x0) / abs(y1 - y0))

    if "--noninteractive" in sys.argv:
        latex.savefig("z_plane")
    else:
        plt.show()


if __name__ == "__main__":
    main()
