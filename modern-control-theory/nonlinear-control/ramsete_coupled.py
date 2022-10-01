#!/usr/bin/env python3

"""Simulates Ramsete controller on coupled model with v and omega as states."""

import math
import sys

import frccontrol as fct
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

from bookutil import latex
from bookutil.drivetrain import ramsete
from bookutil.pose2d import Pose2d
from bookutil.systems import DrivetrainCoupledVelocity

if "--noninteractive" in sys.argv:
    mpl.use("svg")


def main():
    """Entry point."""
    dt = 0.005
    drivetrain = DrivetrainCoupledVelocity(dt)
    print("ctrb cond =", np.linalg.cond(fct.ctrb(drivetrain.sysd.A, drivetrain.sysd.B)))

    ts, xprof, vprof, _ = fct.generate_s_curve_profile(
        max_v=4.0, max_a=3.5, time_to_max_a=1.0, dt=dt, goal=10.0
    )

    # Generate references for LQR
    refs = []
    for i, _ in enumerate(ts):
        r = np.array([[vprof[i]], [0]])
        refs.append(r)

    # Run LQR
    state_rec, ref_rec, u_rec, _ = drivetrain.generate_time_responses(refs)
    nstates = drivetrain.sysd.A.shape[0]
    ninputs = drivetrain.sysd.B.shape[1]
    subplot_max = nstates + ninputs
    for i in range(nstates):
        plt.subplot(subplot_max, 1, i + 1)
        plt.ylabel(
            drivetrain.state_labels[i],
            horizontalalignment="right",
            verticalalignment="center",
            rotation=45,
        )
        if i == 0:
            plt.title("Time domain responses")
        if i == 1:
            plt.ylim([-3, 3])
        plt.plot(ts, drivetrain.extract_row(state_rec, i), label="Estimated state")
        plt.plot(ts, drivetrain.extract_row(ref_rec, i), label="Reference")
        plt.legend()

    for i in range(ninputs):
        plt.subplot(subplot_max, 1, nstates + i + 1)
        plt.ylabel(
            drivetrain.u_labels[i],
            horizontalalignment="right",
            verticalalignment="center",
            rotation=45,
        )
        plt.plot(ts, drivetrain.extract_row(u_rec, i), label="Control effort")
        plt.legend()
    plt.xlabel("Time (s)")
    if "--noninteractive" in sys.argv:
        latex.savefig("ramsete_coupled_vel_lqr_profile")

    # Initial robot pose
    pose = Pose2d(2, 0, np.pi / 2.0)
    desired_pose = Pose2d()

    # Ramsete tuning constants
    b = 2
    zeta = 0.7

    vref = float("inf")
    omegaref = float("inf")

    x_rec = []
    y_rec = []
    vref_rec = []
    omegaref_rec = []
    v_rec = []
    omega_rec = []
    ul_rec = []
    ur_rec = []

    # Log initial data for plots
    vref_rec.append(0)
    omegaref_rec.append(0)
    x_rec.append(pose.x)
    y_rec.append(pose.y)
    ul_rec.append(drivetrain.u[0, 0])
    ur_rec.append(drivetrain.u[1, 0])
    v_rec.append(0)
    omega_rec.append(0)

    # Run Ramsete
    i = 0
    while i < len(ts) - 1:
        desired_pose.x = 0
        desired_pose.y = xprof[i]
        desired_pose.theta = np.pi / 2.0

        # pose_desired, v_desired, omega_desired, pose, b, zeta
        vref, omegaref = ramsete(desired_pose, vprof[i], 0, pose, b, zeta)
        next_r = np.array([[vref], [omegaref]])
        drivetrain.update(next_r)

        # Log data for plots
        vref_rec.append(vref)
        omegaref_rec.append(omegaref)
        x_rec.append(pose.x)
        y_rec.append(pose.y)
        ul_rec.append(drivetrain.u[0, 0])
        ur_rec.append(drivetrain.u[1, 0])
        v_rec.append(drivetrain.x[0, 0])
        omega_rec.append(drivetrain.x[1, 0])

        # Update nonlinear observer
        pose.x += drivetrain.x[0, 0] * math.cos(pose.theta) * dt
        pose.y += drivetrain.x[0, 0] * math.sin(pose.theta) * dt
        pose.theta += drivetrain.x[1, 0] * dt

        if i < len(ts) - 1:
            i += 1

    plt.figure(2)
    plt.plot([0] * len(ts), xprof, label="Reference trajectory")
    plt.plot(x_rec, y_rec, label="Ramsete controller")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.legend()

    plt.gca().set_aspect(1.0)
    plt.gca().set_box_aspect(1.0)

    if "--noninteractive" in sys.argv:
        latex.savefig("ramsete_coupled_response")

    plt.figure(3)
    num_plots = 4
    plt.subplot(num_plots, 1, 1)
    plt.title("Time domain responses")
    plt.ylabel(
        "Velocity (m/s)",
        horizontalalignment="right",
        verticalalignment="center",
        rotation=45,
    )
    plt.plot(ts, vref_rec, label="Reference")
    plt.plot(ts, v_rec, label="Estimated state")
    plt.legend()
    plt.subplot(num_plots, 1, 2)
    plt.ylabel(
        "Angular rate (rad/s)",
        horizontalalignment="right",
        verticalalignment="center",
        rotation=45,
    )
    plt.plot(ts, omegaref_rec, label="Reference")
    plt.plot(ts, omega_rec, label="Estimated state")
    plt.legend()
    plt.subplot(num_plots, 1, 3)
    plt.ylabel(
        "Left voltage (V)",
        horizontalalignment="right",
        verticalalignment="center",
        rotation=45,
    )
    plt.plot(ts, ul_rec, label="Control effort")
    plt.legend()
    plt.subplot(num_plots, 1, 4)
    plt.ylabel(
        "Right voltage (V)",
        horizontalalignment="right",
        verticalalignment="center",
        rotation=45,
    )
    plt.plot(ts, ur_rec, label="Control effort")
    plt.legend()
    plt.xlabel("Time (s)")

    if "--noninteractive" in sys.argv:
        latex.savefig("ramsete_coupled_vel_lqr_response")
    else:
        plt.show()


if __name__ == "__main__":
    main()
