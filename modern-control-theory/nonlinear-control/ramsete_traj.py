#!/usr/bin/env python3

"""Simulates Ramsete controller on decoupled model with nonlinear trajectory."""

import math
import sys

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from wpimath.geometry import Pose2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

from bookutil import latex
from bookutil.drivetrain import get_diff_vels, ramsete
from bookutil.systems import DrivetrainDecoupledVelocity

if "--noninteractive" in sys.argv:
    mpl.use("svg")


def main():
    """Entry point."""
    dt = 0.02

    drivetrain = DrivetrainDecoupledVelocity(dt)

    trajectory = TrajectoryGenerator.generateTrajectory(
        [Pose2d(1.330117, 13, 0), Pose2d(10.17, 18, 0)],
        TrajectoryConfig(3.5, 3.5),
    )

    ts = np.arange(0, trajectory.totalTime(), dt)
    xprof = []
    yprof = []
    thetaprof = []
    vprof = []
    omegaprof = []
    for t in ts:
        sample = trajectory.sample(t)
        xprof.append(sample.pose.X())
        yprof.append(sample.pose.Y())
        thetaprof.append(sample.pose.rotation().radians())
        vprof.append(sample.velocity)
        omegaprof.append(sample.velocity * sample.curvature)

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
    vr_rec = []
    r_vl_rec = []
    r_vr_rec = []
    ul_rec = []
    ur_rec = []

    # Run Ramsete
    next_r = np.array([[0.0], [0.0]])
    for i in range(len(ts) - 1):
        desired_pose = Pose2d(xprof[i], yprof[i], thetaprof[i])

        # pose_desired, v_desired, omega_desired, pose, b, zeta
        vref, omegaref = ramsete(desired_pose, vprof[i], omegaprof[i], pose, b, zeta)
        r_vl, r_vr = get_diff_vels(vref, omegaref, drivetrain.rb * 2.0)
        r = next_r
        next_r = np.array([[r_vl], [r_vr]])

        drivetrain.update(r, next_r)

        vc = (drivetrain.x[0, 0] + drivetrain.x[1, 0]) / 2.0
        omega = (drivetrain.x[1, 0] - drivetrain.x[0, 0]) / (2.0 * drivetrain.rb)
        vl, vr = get_diff_vels(vc, omega, drivetrain.rb * 2.0)

        # Log data for plots
        x_rec.append(pose.X())
        y_rec.append(pose.Y())
        theta_rec.append(pose.rotation().radians())
        vl_rec.append(vl)
        vr_rec.append(vr)
        r_vl_rec.append(r_vl)
        r_vr_rec.append(r_vr)
        ul_rec.append(drivetrain.u[0, 0])
        ur_rec.append(drivetrain.u[1, 0])

        # Update nonlinear observer
        pose = Pose2d(
            pose.X() + vc * pose.rotation().cos() * dt,
            pose.Y() + vc * pose.rotation().sin() * dt,
            pose.rotation().radians() + omega * dt,
        )

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
    plt.plot(ts[:-1], x_rec, label="State")
    plt.plot(ts, xprof, label="Reference")
    plt.legend()
    plt.subplot(num_plots, 1, 2)
    plt.ylabel(
        "y position (m)",
        horizontalalignment="right",
        verticalalignment="center",
        rotation=45,
    )
    plt.plot(ts[:-1], y_rec, label="State")
    plt.plot(ts, yprof, label="Reference")
    plt.legend()
    plt.subplot(num_plots, 1, 3)
    plt.ylabel(
        "Theta (rad)",
        horizontalalignment="right",
        verticalalignment="center",
        rotation=45,
    )
    plt.plot(ts[:-1], theta_rec, label="State")
    plt.plot(ts, thetaprof, label="Reference")
    plt.legend()

    ts = ts[:-1]
    plt.subplot(num_plots, 1, 4)
    plt.ylabel(
        "Left velocity (m/s)",
        horizontalalignment="right",
        verticalalignment="center",
        rotation=45,
    )
    plt.plot(ts, vl_rec, label="State")
    plt.plot(ts, r_vl_rec, label="Reference")
    plt.legend()
    plt.subplot(num_plots, 1, 5)
    plt.ylabel(
        "Right velocity (m/s)",
        horizontalalignment="right",
        verticalalignment="center",
        rotation=45,
    )
    plt.plot(ts, vr_rec, label="State")
    plt.plot(ts, r_vr_rec, label="Reference")
    plt.legend()
    plt.subplot(num_plots, 1, 6)
    plt.ylabel(
        "Left voltage (V)",
        horizontalalignment="right",
        verticalalignment="center",
        rotation=45,
    )
    plt.plot(ts, ul_rec, label="Input")
    plt.legend()
    plt.subplot(num_plots, 1, 7)
    plt.ylabel(
        "Right voltage (V)",
        horizontalalignment="right",
        verticalalignment="center",
        rotation=45,
    )
    plt.plot(ts, ur_rec, label="Input")
    plt.legend()
    plt.xlabel("Time (s)")

    if "--noninteractive" in sys.argv:
        latex.savefig("ramsete_traj_response")
    else:
        plt.show()


if __name__ == "__main__":
    main()
