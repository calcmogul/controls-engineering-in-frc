#!/usr/bin/env python3

"""Simulates Ramsete controller on decoupled model with nonlinear trajectory."""

import math
import sys

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

from bookutil import latex
from bookutil.drivetrain import get_diff_vels, ramsete
from bookutil.pose2d import Pose2d
from bookutil.systems import DrivetrainDecoupledVelocity

if "--noninteractive" in sys.argv:
    mpl.use("svg")


def main():
    """Entry point."""
    dt = 0.02
    drivetrain = DrivetrainDecoupledVelocity(dt)

    t, xprof, yprof, thetaprof, vprof, omegaprof = np.genfromtxt(
        "ramsete_traj.csv", delimiter=",", skip_header=1, unpack=True
    )

    # Initial robot pose
    pose = Pose2d(xprof[0] + 0.5, yprof[0] + 0.5, math.pi)
    desired_pose = Pose2d()

    # Ramsete tuning constants
    b = 2
    zeta = 0.7

    vl = float("inf")
    vr = float("inf")

    x_rec = []
    y_rec = []
    theta_rec = []
    vl_rec = []
    vlref_rec = []
    vr_rec = []
    vrref_rec = []
    ul_rec = []
    ur_rec = []

    # Run Ramsete
    next_r = np.array([[0.0], [0.0]])
    i = 0
    while i < len(t) - 1:
        desired_pose.x = xprof[i]
        desired_pose.y = yprof[i]
        desired_pose.theta = thetaprof[i]

        # pose_desired, v_desired, omega_desired, pose, b, zeta
        vref, omegaref = ramsete(desired_pose, vprof[i], omegaprof[i], pose, b, zeta)
        vlref, vrref = get_diff_vels(vref, omegaref, drivetrain.rb * 2.0)
        r = next_r
        next_r = np.array([[vlref], [vrref]])
        drivetrain.update(r, next_r)
        vc = (drivetrain.x[0, 0] + drivetrain.x[1, 0]) / 2.0
        omega = (drivetrain.x[1, 0] - drivetrain.x[0, 0]) / (2.0 * drivetrain.rb)
        vl, vr = get_diff_vels(vc, omega, drivetrain.rb * 2.0)

        # Log data for plots
        vlref_rec.append(vlref)
        vrref_rec.append(vrref)
        x_rec.append(pose.x)
        y_rec.append(pose.y)
        theta_rec.append(pose.theta)
        vl_rec.append(vl)
        vr_rec.append(vr)
        ul_rec.append(drivetrain.u[0, 0])
        ur_rec.append(drivetrain.u[1, 0])

        # Update nonlinear observer
        pose.x += vc * math.cos(pose.theta) * dt
        pose.y += vc * math.sin(pose.theta) * dt
        pose.theta += omega * dt

        if i < len(t) - 1:
            i += 1

    plt.figure(1)
    plt.plot(x_rec, y_rec, label="Ramsete controller")
    plt.plot(xprof, yprof, label="Reference trajectory")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.legend()

    plt.gca().set_aspect(1.0)
    plt.gca().set_box_aspect(1.0)

    if "--noninteractive" in sys.argv:
        latex.savefig("ramsete_traj_xy")

    plt.figure(2)
    num_plots = 7
    plt.subplot(num_plots, 1, 1)
    plt.title("Time domain responses")
    plt.ylabel(
        "x position (m)",
        horizontalalignment="right",
        verticalalignment="center",
        rotation=45,
    )
    plt.plot(t[:-1], x_rec, label="Estimated state")
    plt.plot(t, xprof, label="Reference")
    plt.legend()
    plt.subplot(num_plots, 1, 2)
    plt.ylabel(
        "y position (m)",
        horizontalalignment="right",
        verticalalignment="center",
        rotation=45,
    )
    plt.plot(t[:-1], y_rec, label="Estimated state")
    plt.plot(t, yprof, label="Reference")
    plt.legend()
    plt.subplot(num_plots, 1, 3)
    plt.ylabel(
        "Theta (rad)",
        horizontalalignment="right",
        verticalalignment="center",
        rotation=45,
    )
    plt.plot(t[:-1], theta_rec, label="Estimated state")
    plt.plot(t, thetaprof, label="Reference")
    plt.legend()

    t = t[:-1]
    plt.subplot(num_plots, 1, 4)
    plt.ylabel(
        "Left velocity (m/s)",
        horizontalalignment="right",
        verticalalignment="center",
        rotation=45,
    )
    plt.plot(t, vl_rec, label="Estimated state")
    plt.plot(t, vlref_rec, label="Reference")
    plt.legend()
    plt.subplot(num_plots, 1, 5)
    plt.ylabel(
        "Right velocity (m/s)",
        horizontalalignment="right",
        verticalalignment="center",
        rotation=45,
    )
    plt.plot(t, vr_rec, label="Estimated state")
    plt.plot(t, vrref_rec, label="Reference")
    plt.legend()
    plt.subplot(num_plots, 1, 6)
    plt.ylabel(
        "Left voltage (V)",
        horizontalalignment="right",
        verticalalignment="center",
        rotation=45,
    )
    plt.plot(t, ul_rec, label="Control effort")
    plt.legend()
    plt.subplot(num_plots, 1, 7)
    plt.ylabel(
        "Right voltage (V)",
        horizontalalignment="right",
        verticalalignment="center",
        rotation=45,
    )
    plt.plot(t, ur_rec, label="Control effort")
    plt.legend()
    plt.xlabel("Time (s)")

    if "--noninteractive" in sys.argv:
        latex.savefig("ramsete_traj_response")
    else:
        plt.show()


if __name__ == "__main__":
    main()
