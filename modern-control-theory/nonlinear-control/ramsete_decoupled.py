#!/usr/bin/env python3

"""
Simulates Ramsete controller on decoupled model with right and left wheel
velocities as states.
"""

import math
import sys

import frccontrol as fct
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from wpimath.geometry import Pose2d

from bookutil import latex
from bookutil.drivetrain import get_diff_vels, ramsete
from bookutil.systems import DrivetrainDecoupledVelocity

if "--noninteractive" in sys.argv:
    mpl.use("svg")


def main():
    """Entry point."""
    dt = 0.005
    drivetrain = DrivetrainDecoupledVelocity(dt)

    cond = round(np.linalg.cond(fct.ctrb(drivetrain.sim.A, drivetrain.sim.B)), 3)
    if "--noninteractive" in sys.argv:
        with open("ramsete-decoupled-ctrb-cond.tex", "w", encoding="utf-8") as f:
            f.write(f"{cond}\n")
    else:
        print(f"ctrb cond = {cond}")

    ts, xprof, vprof, _ = fct.generate_s_curve_profile(
        max_v=4.0, max_a=3.5, time_to_max_a=1.0, dt=dt, goal=10.0
    )

    # Generate references for LQR
    refs = []
    for i, _ in enumerate(ts):
        r = np.array([[vprof[i]], [vprof[i]]])
        refs.append(r)

    # Run LQR
    state_rec, r_rec, u_rec, _ = fct.generate_time_responses(drivetrain, refs)
    state_labels = ["Left velocity (m/s)", "Right velocity (m/s)"]
    input_labels = ["Left voltage (V)", "Right voltage (V)"]
    fct.plot_time_responses(state_labels, input_labels, ts, state_rec, r_rec, u_rec)
    if "--noninteractive" in sys.argv:
        latex.savefig("ramsete_decoupled_vel_lqr_profile")

    # Initial robot pose
    pose = Pose2d(2, 0, math.pi / 2.0)
    desired_pose = Pose2d()

    # Ramsete tuning constants
    b = 2
    zeta = 0.7

    x_rec = [pose.X()]
    y_rec = [pose.Y()]
    r_v_rec = [0]
    r_omega_rec = [0]
    v_rec = [0]
    omega_rec = [0]
    ul_rec = [drivetrain.u[0, 0]]
    ur_rec = [drivetrain.u[1, 0]]

    # Run Ramsete
    next_r = np.array([[0.0], [0.0]])
    for i in range(len(ts) - 1):
        desired_pose = Pose2d(0, xprof[i], math.pi / 2.0)

        # pose_desired, v_desired, omega_desired, pose, b, zeta
        vref, omegaref = ramsete(desired_pose, vprof[i], 0, pose, b, zeta)
        vl, vr = get_diff_vels(vref, omegaref, drivetrain.rb * 2.0)
        r = next_r
        next_r = np.array([[vl], [vr]])
        drivetrain.update(r, next_r)
        vc = (drivetrain.x[0, 0] + drivetrain.x[1, 0]) / 2.0
        omega = (drivetrain.x[1, 0] - drivetrain.x[0, 0]) / (2.0 * drivetrain.rb)

        # Log data for plots
        r_v_rec.append(vref)
        r_omega_rec.append(omegaref)
        x_rec.append(pose.x)
        y_rec.append(pose.y)
        ul_rec.append(drivetrain.u[0, 0])
        ur_rec.append(drivetrain.u[1, 0])
        v_rec.append(vc)
        omega_rec.append(omega)

        # Update nonlinear observer
        pose = Pose2d(
            pose.X() + vc * pose.rotation().cos() * dt,
            pose.Y() + vc * pose.rotation().sin() * dt,
            pose.rotation().radians() + omega * dt,
        )

    plt.figure(2)
    plt.plot([0] * len(ts), xprof, label="Reference trajectory")
    plt.plot(x_rec, y_rec, label="Ramsete controller")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.legend()

    plt.gca().set_aspect(1.0)
    plt.gca().set_box_aspect(1.0)

    if "--noninteractive" in sys.argv:
        latex.savefig("ramsete_decoupled_response")

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
    plt.plot(ts, r_v_rec, label="Reference")
    plt.plot(ts, v_rec, label="State")
    plt.legend()
    plt.subplot(num_plots, 1, 2)
    plt.ylabel(
        "Angular rate (rad/s)",
        horizontalalignment="right",
        verticalalignment="center",
        rotation=45,
    )
    plt.plot(ts, r_omega_rec, label="Reference")
    plt.plot(ts, omega_rec, label="State")
    plt.legend()
    plt.subplot(num_plots, 1, 3)
    plt.ylabel(
        "Left voltage (V)",
        horizontalalignment="right",
        verticalalignment="center",
        rotation=45,
    )
    plt.plot(ts, ul_rec, label="Input")
    plt.legend()
    plt.subplot(num_plots, 1, 4)
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
        latex.savefig("ramsete_decoupled_vel_lqr_response")
    else:
        plt.show()


if __name__ == "__main__":
    main()
