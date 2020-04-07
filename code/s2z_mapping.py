#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
    import bookutil.latex as latex

import matplotlib.patches as pts
import matplotlib.pyplot as plt

plt.rc("text", usetex=True)
GREEN = (0, 0.75, 0)
BLUE = (0, 0, 0.75)
YELLOW = (1, 0.75, 0.25)


def draw_s_plane():
    ax = plt.gca()

    plt.title("s-plane")

    # Draw axes
    ax.annotate(
        "",
        xy=(-1.5, 0),  # Start coord of arrow
        xytext=(1.5, 0),  # End coord of arrow
        arrowprops=dict(arrowstyle="<->"),
        zorder=-999,
    )
    ax.annotate(
        "",
        xy=(0, -1.5),  # Start coord of arrow
        xytext=(0, 1.5),  # End coord of arrow
        arrowprops=dict(arrowstyle="<->"),
        zorder=-999,
    )

    # Green rectangle
    rect = pts.Rectangle(xy=(-0.005, 0), width=0.01, height=1.4, color=GREEN)
    ax.add_artist(rect)

    # Green arrow
    plt.annotate(
        "",
        xy=(0, 0),  # Start coord of arrow
        xytext=(0, 1.5),  # End coord of arrow
        arrowprops=dict(arrowstyle="<-", linewidth=2, color=GREEN),
        zorder=999,
    )

    # Blue rectangle
    rect = pts.Rectangle(xy=(-0.005, 0), width=0.01, height=-1.4, color=BLUE)
    ax.add_artist(rect)

    # Blue arrow
    plt.annotate(
        "",
        xy=(0, 0),  # Start coord of arrow
        xytext=(0, -1.5),  # End coord of arrow
        arrowprops=dict(arrowstyle="<-", linewidth=2, color=BLUE),
        zorder=999,
    )

    # Yellow rectangle
    rect = pts.Rectangle(xy=(0, -0.005), width=-1.4, height=0.01, color=YELLOW)
    ax.add_artist(rect)

    # Yellow arrow
    plt.annotate(
        "",
        xy=(0, 0),  # Start coord of arrow
        xytext=(-1.5, 0),  # End coord of arrow
        arrowprops=dict(arrowstyle="<-", linewidth=2, color=YELLOW),
        zorder=999,
    )

    plt.xlim([-1.5, 1.5])
    plt.ylim([-1.5, 1.5])
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    ax.set_aspect(abs(x1 - x0) / abs(y1 - y0))


def draw_z_plane():
    ax = plt.gca()

    plt.title("z-plane")

    # Draw axes
    ax.annotate(
        "",
        xy=(-1.5, 0),  # Start coord of arrow
        xytext=(1.5, 0),  # End coord of arrow
        arrowprops=dict(arrowstyle="<->"),
        zorder=-999,
    )
    ax.annotate(
        "",
        xy=(0, -1.5),  # Start coord of arrow
        xytext=(0, 1.5),  # End coord of arrow
        arrowprops=dict(arrowstyle="<->"),
        zorder=-999,
    )

    # Green arc
    arc = pts.Arc(
        xy=(0, 0), width=2, height=2, theta1=0, theta2=175, color=GREEN, linewidth=2
    )
    ax.add_artist(arc)

    # Green arrow
    plt.annotate(
        "",
        xy=(-1, 0.1),  # Start coord of arrow
        xytext=(-1, 0),  # End coord of arrow
        arrowprops=dict(arrowstyle="<-", linewidth=2, color=GREEN),
        zorder=999,
    )

    # Blue arc
    arc = pts.Arc(
        xy=(0, 0), width=2, height=2, theta1=185, theta2=360, color=BLUE, linewidth=2
    )
    ax.add_artist(arc)

    # Blue arrow
    plt.annotate(
        "",
        xy=(-1, -0.1),  # Start coord of arrow
        xytext=(-1, 0),  # End coord of arrow
        arrowprops=dict(arrowstyle="<-", linewidth=2, color=BLUE),
        zorder=999,
    )

    # Yellow rectangle
    rect = pts.Rectangle(xy=(0.1, -0.01), width=0.9, height=0.02, color=YELLOW)
    ax.add_artist(rect)

    # Yellow arrow
    plt.annotate(
        "",
        xy=(0, 0),  # Start coord of arrow
        xytext=(1, 0),  # End coord of arrow
        arrowprops=dict(arrowstyle="->", linewidth=2, color=YELLOW),
        zorder=999,
    )

    plt.xlim([-1.5, 1.5])
    plt.ylim([-1.5, 1.5])
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    ax.set_aspect(abs(x1 - x0) / abs(y1 - y0))


def main():
    plt.figure(1)
    draw_s_plane()
    if "--noninteractive" in sys.argv:
        latex.savefig("s_plane")

    plt.figure(2)
    draw_z_plane()
    if "--noninteractive" in sys.argv:
        latex.savefig("z_plane")
    else:
        plt.show()


if __name__ == "__main__":
    main()
