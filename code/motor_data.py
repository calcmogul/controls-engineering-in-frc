#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys
if "--noninteractive" in sys.argv:
    import matplotlib as mpl
    mpl.use("svg")
    import latexutils

import matplotlib.pyplot as plt
import numpy as np
import os

plt.rc("text", usetex=True)


def main():
    i_stall = 134
    i_free = 0.7
    rpm_free = 18730
    p_max = 347
    t_stall = 0.71

    # 775pro data
    fig, ax_left = plt.subplots()
    ax_left.set_xlabel("Speed (RPM)")

    rpm = np.arange(0, rpm_free, 50)
    line1 = ax_left.plot(
        rpm, [i_stall - i_stall / rpm_free * x for x in rpm],
        "b",
        label="Current (A)")
    ax_left.annotate(
        "Stall current: " + str(i_stall) + " A",
        xy=(0, i_stall),
        xytext=(0, 160),
        arrowprops=dict(arrowstyle="->"),
    )
    line2 = ax_left.plot(
        rpm, [
            -p_max / (rpm_free / 2.0)**2 * (x - rpm_free / 2.0)**2 + p_max
            for x in rpm
        ],
        "g",
        label="Output power (W)")
    ax_left.annotate(
        "Peak power: " + str(p_max) + " W" + os.linesep + "(at " + str(
            rpm_free / 2.0) + " RPM)",
        xy=(rpm_free / 2.0, p_max),
        xytext=(7000, 365),
        arrowprops=dict(arrowstyle="->"),
    )
    ax_left.set_ylabel("Current (A), Power (W)")
    ax_left.set_ylim([0, 400])
    plt.legend(loc=3)

    ax_right = ax_left.twinx()
    line3 = ax_right.plot(
        rpm, [t_stall - t_stall / rpm_free * x for x in rpm],
        "y",
        label="Torque (N-m)")
    ax_right.annotate(
        "Stall torque: " + str(t_stall) + " N-m",
        xy=(0, t_stall),
        xytext=(0, 0.75),
        arrowprops=dict(arrowstyle="->"),
    )
    ax_right.annotate(
        "Free speed: " + str(rpm_free) + " RPM" + os.linesep + "Free current: "
        + str(i_free) + " A",
        xy=(rpm_free, 0),
        xytext=(11000, 0.3),
        arrowprops=dict(arrowstyle="->"),
    )
    ax_right.set_ylabel("Torque (N-m)")
    ax_right.set_ylim([0, 0.8])
    plt.legend(loc=1)

    if "--noninteractive" in sys.argv:
        latexutils.savefig("motor_data")
    else:
        plt.show()


if __name__ == "__main__":
    main()
