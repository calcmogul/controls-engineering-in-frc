#!/usr/bin/env python3

"""Companion figures for section on configuration spaces."""

import math
import sys

import matplotlib as mpl
from matplotlib import colors
import matplotlib.pyplot as plt

from bookutil import latex

if "--noninteractive" in sys.argv:
    mpl.use("svg")
plt.rc("text", usetex=True)


def config_plot(xlim, ylim):
    """Configure the global plot with x and y labels and limits.

    Keyword arguments:
    xlim -- the x limits
    ylim -- the y limits
    """
    plt.xlabel("Arm angle (rad)")
    plt.ylabel("Elevator height (m)")
    plt.xlim(xlim)
    plt.ylim(ylim)


def make_box(bottom_left, top_right):
    """Creates a pyplot Polygon box.

    Keyword arguments:
    bottom_left -- the box's bottom-left coordinate
    top_right -- the box's top-right coordinate
    """
    return plt.Polygon(
        [
            (bottom_left[0], bottom_left[1]),
            (top_right[0], bottom_left[1]),
            (top_right[0], top_right[1]),
            (bottom_left[0], top_right[1]),
        ]
    )


def make_invalid_region(xlim, ylim):
    """Creates a pyplot Polygon "invalid region".

    Keyword arguments:
    xlim -- the x limits of the invalid region
    ylim -- the y limits of the invalid region
    """
    points = list(zip(xlim, ylim))
    invalid_states = make_box(points[0], points[1])
    invalid_states.set_color(colors.hsv_to_rgb((1, 0.8, 0.9)))
    invalid_states.set_edgecolor((1, 0.5, 0.5))
    invalid_states.set_hatch("x")
    invalid_states.set_label("Invalid states")
    return invalid_states


def make_valid_region(points):
    """Turns a list of x-y pairs into a valid region.

    Keyword arguments:
    points -- list of x-y pairs
    """
    valid_states = plt.Polygon(points)
    valid_states.set_color((1, 1, 1))
    valid_states.set_label("Valid states")
    return valid_states


def draw_point(ax, x, y, label, horizontalalignment="left", verticalalignment="top"):
    """Draw a point and corresponding label onto an Axis.

    Keyword arguments:
    ax -- the Axis object on which to draw
    x -- the point's x coordinate
    y -- the point's y coordinate
    horizontalalignment -- how the text should be horizontally aligned with
                           respect to the point (default: "left")
    verticalignment -- how the text should be vertically aligned with respect to
                       the point (default: "top")
    """
    ax.scatter(x, y, color="C0", s=10, zorder=2)
    ax.annotate(
        label,
        (x + 0.05, y),
        horizontalalignment=horizontalalignment,
        verticalalignment=verticalalignment,
    )


def main():
    """Entry point."""
    xlim = [-math.pi, math.pi]
    ylim = [-1, 6.5]

    # Figure 1
    plt.figure()
    ax = plt.gca()
    config_plot(xlim, ylim)

    # Draw invalid region
    ax.add_patch(make_invalid_region(xlim, ylim))

    # Draw valid region
    ax.add_patch(
        make_valid_region(
            [
                (-3 / 4 * math.pi, 0),
                (3 / 4 * math.pi, 0),
                (3 / 4 * math.pi, 5),
                (-3 / 4 * math.pi, 5),
            ]
        )
    )

    ax.legend()

    if "--noninteractive" in sys.argv:
        latex.savefig("configuration_spaces_fig1")

    # Figure 2
    plt.figure()
    ax = plt.gca()
    config_plot(xlim, ylim)

    # Draw invalid region
    ax.add_patch(make_invalid_region(xlim, ylim))

    # Draw valid region
    ax.add_patch(
        make_valid_region(
            [
                (-3 / 4 * math.pi, 0),
                (-math.pi / 4, 0),
                (-math.pi / 4, 2),
                (math.pi / 4, 2),
                (math.pi / 4, 0),
                (3 / 4 * math.pi, 0),
                (3 / 4 * math.pi, 5),
                (-3 / 4 * math.pi, 5),
            ]
        )
    )

    ax.legend()

    if "--noninteractive" in sys.argv:
        latex.savefig("configuration_spaces_fig2")

    # Figure 3
    plt.figure()
    ax = plt.gca()
    config_plot(xlim, ylim)

    # Draw invalid region
    ax.add_patch(make_invalid_region(xlim, ylim))

    # Draw valid region
    ax.add_patch(
        make_valid_region(
            [
                (-3 / 4 * math.pi, 0),
                (-math.pi / 4, 0),
                (-math.pi / 4, 2),
                (math.pi / 4, 2),
                (math.pi / 4, 0),
                (3 / 4 * math.pi, 0),
                (3 / 4 * math.pi, 5),
                (-3 / 4 * math.pi, 5),
            ]
        )
    )

    # Draw start and end points
    draw_point(ax, -1, 0.5, label="x")
    draw_point(ax, 1, 0.5, label="r")

    ax.legend()

    if "--noninteractive" in sys.argv:
        latex.savefig("configuration_spaces_fig3")

    # Figure 4
    plt.figure()
    ax = plt.gca()
    config_plot(xlim, ylim)

    # Draw invalid region
    ax.add_patch(make_invalid_region(xlim, ylim))

    # Draw valid region
    ax.add_patch(
        make_valid_region(
            [
                (-3 / 4 * math.pi, 0),
                (-math.pi / 4, 0),
                (-math.pi / 4, 2),
                (math.pi / 4, 2),
                (math.pi / 4, 0),
                (3 / 4 * math.pi, 0),
                (3 / 4 * math.pi, 5),
                (-3 / 4 * math.pi, 5),
            ]
        )
    )

    # Draw path between start and goal
    points = [(-1, 0.5), (-math.pi / 4, 2), (math.pi / 4, 2), (1, 0.5)]
    ax.plot([p[0] for p in points], [p[1] for p in points], color="C0")

    # Draw start and end points
    draw_point(ax, -1, 0.5, label="x")
    draw_point(ax, -math.pi / 4, 2, label="a", verticalalignment="bottom")
    draw_point(ax, math.pi / 4, 2, label="b", verticalalignment="bottom")
    draw_point(ax, 1, 0.5, label="r")

    ax.legend()

    if "--noninteractive" in sys.argv:
        latex.savefig("configuration_spaces_fig4")
    else:
        plt.show()


if __name__ == "__main__":
    main()
