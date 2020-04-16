#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
    import bookutil.latex as latex

import frccontrol as fct
import matplotlib.pyplot as plt

plt.rc("text", usetex=True)


def main():
    t, x, v, a = fct.generate_trapezoid_profile(
        max_v=7.0, time_to_max_v=2.0, dt=0.05, goal=50.0
    )
    plt.figure()
    plt.subplot(3, 1, 1)
    plt.ylabel("Position (m)")
    plt.plot(t, x, label="Position")
    plt.subplot(3, 1, 2)
    plt.ylabel("Velocity (m/s)")
    plt.plot(t, v, label="Velocity")
    plt.subplot(3, 1, 3)
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration ($m/s^2$)")
    plt.plot(t, a, label="Acceleration")
    if "--noninteractive" in sys.argv:
        latex.savefig("trapezoid_profile")

    t, x, v, a = fct.generate_s_curve_profile(
        max_v=7.0, max_a=3.5, time_to_max_a=1.0, dt=0.05, goal=50.0
    )
    plt.figure()
    plt.subplot(3, 1, 1)
    plt.ylabel("Position (m)")
    plt.plot(t, x, label="Position")
    plt.subplot(3, 1, 2)
    plt.ylabel("Velocity (m/s)")
    plt.plot(t, v, label="Velocity")
    plt.subplot(3, 1, 3)
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration ($m/s^2$)")
    plt.plot(t, a, label="Acceleration")
    if "--noninteractive" in sys.argv:
        latex.savefig("s_curve_profile")
    else:
        plt.show()


if __name__ == "__main__":
    main()
