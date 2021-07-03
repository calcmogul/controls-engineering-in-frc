#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
    import bookutil.latex as latex

import frccontrol as fct
import math
import matplotlib.pyplot as plt
import numpy as np

from bookutil.systems import Flywheel

plt.rc("text", usetex=True)


def main():
    dt = 0.005
    flywheel = Flywheel(dt)

    # Set up graphing
    l0 = 0.1
    l1 = l0 + 5.0
    l2 = l1 + 0.1
    t = np.arange(0, l2 + 5.0, dt)

    refs = []

    # Generate references for simulation
    for i in range(len(t)):
        if t[i] < l0:
            r = np.array([[0]])
        elif t[i] < l1:
            r = np.array([[9000 / 60 * 2 * math.pi]])
        else:
            r = np.array([[0]])
        refs.append(r)

    plt.figure(1)
    x_rec, ref_rec, u_rec, y_rec = flywheel.generate_time_responses(t, refs)

    plt.ylabel(flywheel.state_labels[0])
    plt.plot(t, flywheel.extract_row(x_rec, 0), label="Output")
    plt.plot(t, flywheel.extract_row(ref_rec, 0), label="Setpoint")

    fill_end = int(3.0 / dt)
    plt.fill_between(
        t[:fill_end],
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
