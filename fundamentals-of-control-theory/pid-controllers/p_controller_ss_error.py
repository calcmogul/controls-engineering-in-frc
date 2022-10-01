#!/usr/bin/env python3

"""Demonstrates P controller with steady-state error."""

import math
import sys

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

from bookutil import latex
from bookutil.systems import Flywheel

if "--noninteractive" in sys.argv:
    mpl.use("svg")
plt.rc("text", usetex=True)


def main():
    """Entry point."""
    dt = 0.005
    flywheel = Flywheel(dt)

    # Set up graphing
    l0 = 0.1
    l1 = l0 + 5.0
    l2 = l1 + 0.1
    ts = np.arange(0, l2 + 5.0, dt)

    refs = []

    # Generate references for simulation
    for t in ts:
        if t < l0:
            r = np.array([[0]])
        elif t < l1:
            r = np.array([[9000 / 60 * 2 * math.pi]])
        else:
            r = np.array([[0]])
        refs.append(r)

    plt.figure(1)
    x_rec, ref_rec, _, _ = flywheel.generate_time_responses(refs)

    plt.ylabel(flywheel.state_labels[0])
    plt.plot(ts, flywheel.extract_row(x_rec, 0), label="Output")
    plt.plot(ts, flywheel.extract_row(ref_rec, 0), label="Setpoint")

    fill_end = int(3.0 / dt)
    plt.fill_between(
        ts[:fill_end],
        np.ravel(x_rec)[:fill_end],
        np.ravel(ref_rec)[:fill_end],
        color=(0.5, 0.5, 0.5, 0.5),
        label="Error area for integral term",
    )
    plt.legend()
    plt.xlabel("Time (s)")

    annotate_t = 3.25
    time = int(annotate_t / dt)
    bottom = np.ravel(x_rec[0, time])
    top = np.ravel(ref_rec[0, time])
    plt.annotate(
        "",
        xy=(annotate_t, bottom - 5),  # Start coord of arrow
        xycoords="data",
        xytext=(annotate_t, top + 5),  # End coord of arrow
        textcoords="data",
        arrowprops=dict(arrowstyle="<->", connectionstyle="arc3,rad=0"),
        ha="center",
        va="center",
    )
    plt.annotate(
        "steady-state error",
        xy=(annotate_t + 0.125, (top + bottom) / 2.0),  # Start coord of arrow
        xycoords="data",
        ha="left",
        va="center",
    )

    if "--noninteractive" in sys.argv:
        latex.savefig("p_controller_ss_error")
    else:
        plt.show()


if __name__ == "__main__":
    main()
