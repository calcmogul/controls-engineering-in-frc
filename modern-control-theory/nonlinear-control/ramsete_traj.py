#!/usr/bin/env python3

"""Simulates Ramsete controller on decoupled model with nonlinear trajectory."""

import math
import sys

import frccontrol as fct
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import StateSpace
from wpimath.geometry import Pose2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

from bookutil import latex

if "--noninteractive" in sys.argv:
    mpl.use("svg")


def ramsete(pose_desired, v_desired, omega_desired, pose, b, zeta):
    """Ramsete is a nonlinear time-varying feedback controller for unicycle
    models that drives the model to a desired pose along a two-dimensional
    trajectory.

    The reference pose, linear velocity, and angular velocity should come from
    a drivetrain trajectory.

    Keyword arguments:
    pose_desired -- the desired pose
    v_desired -- the desired linear velocity
    omega_desired -- the desired angular velocity
    pose -- the current pose
    b -- tuning parameter (b > 0) for which larger values make convergence more
         aggressive like a proportional term
    zeta -- tuning parameter (0 < zeta < 1) for which larger values provide more
            damping in response

    Returns:
    linear velocity and angular velocity commands
    """
    e = pose_desired.relativeTo(pose)

    k = 2 * zeta * math.sqrt(omega_desired**2 + b * v_desired**2)
    v = v_desired * e.rotation().cos() + k * e.x
    omega = (
        omega_desired
        + k * e.rotation().radians()
        + b * v_desired * np.sinc(e.rotation().radians()) * e.y
    )

    return v, omega


def drivetrain_decoupled(motor, num_motors, m, r, rb, J, Gl, Gr):
    """Returns the state-space model for a drivetrain.

    States: [[left velocity], [right velocity]]
    Inputs: [[left voltage], [right voltage]]
    Outputs: [[left velocity], [right velocity]]

    Keyword arguments:
    motor -- instance of DcBrushedMotor
    num_motors -- number of motors driving the mechanism
    m -- mass of robot in kg
    r -- radius of wheels in meters
    rb -- radius of robot in meters
    J -- moment of inertia of the drivetrain in kg-m²
    Gl -- gear ratio of left side of drivetrain
    Gr -- gear ratio of right side of drivetrain

    Returns:
    StateSpace instance containing continuous model
    """
    motor = fct.models.gearbox(motor, num_motors)

    C1 = -(Gl**2) * motor.Kt / (motor.Kv * motor.R * r**2)
    C2 = Gl * motor.Kt / (motor.R * r)
    C3 = -(Gr**2) * motor.Kt / (motor.Kv * motor.R * r**2)
    C4 = Gr * motor.Kt / (motor.R * r)
    # fmt: off
    A = np.array([[(1 / m + rb**2 / J) * C1, (1 / m - rb**2 / J) * C3],
                  [(1 / m - rb**2 / J) * C1, (1 / m + rb**2 / J) * C3]])
    B = np.array([[(1 / m + rb**2 / J) * C2, (1 / m - rb**2 / J) * C4],
                  [(1 / m - rb**2 / J) * C2, (1 / m + rb**2 / J) * C4]])
    C = np.array([[1, 0],
                  [0, 1]])
    D = np.array([[0, 0],
                  [0, 0]])
    # fmt: on

    return StateSpace(A, B, C, D)


class Drivetrain:
    """An frccontrol system for a decoupled drivetrain."""

    def __init__(self, dt):
        """Drivetrain subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        """
        self.dt = dt

        # Number of motors per side
        num_motors = 2.0

        # Gear ratio of drivetrain
        G = 60.0 / 11.0

        # Drivetrain mass in kg
        m = 52
        # Radius of wheels in meters
        r = 0.08255 / 2.0
        # Radius of robot in meters
        self.rb = 0.59055 / 2.0

        # Moment of inertia of the drivetrain in kg-m²
        J = 6.0

        self.plant = drivetrain_decoupled(
            fct.models.MOTOR_CIM, num_motors, m, r, self.rb, J, G, G
        )

        # Sim variables
        self.sim = self.plant.to_discrete(self.dt)
        self.x = np.zeros((2, 1))
        self.u = np.zeros((2, 1))
        self.y = np.zeros((2, 1))

        # States: left velocity (m/s), right velocity (m/s)
        # Inputs: left voltage (V), right voltage (V)
        # Outputs: left velocity (m/s), right velocity (m/s)
        self.observer = fct.KalmanFilter(self.plant, [1.0, 1.0], [0.01, 0.01], self.dt)
        self.feedforward = fct.LinearPlantInversionFeedforward(
            self.plant.A, self.plant.B, self.dt
        )
        self.feedback = fct.LinearQuadraticRegulator(
            self.plant.A, self.plant.B, [0.95, 0.95], [12.0, 12.0], self.dt
        )

        self.u_min = np.array([[-12.0], [-12.0]])
        self.u_max = np.array([[12.0], [12.0]])

    def update(self, r, next_r):
        """
        Advance the model by one timestep.

        Keyword arguments:
        r -- the current reference
        next_r -- the next reference
        """
        # Update sim model
        self.x = self.sim.A @ self.x + self.sim.B @ self.u
        self.y = self.sim.C @ self.x + self.sim.D @ self.u

        self.observer.predict(self.u, self.dt)
        self.observer.correct(self.u, self.y)
        self.u = np.clip(
            self.feedforward.calculate(next_r)
            + self.feedback.calculate(self.observer.x_hat, r),
            self.u_min,
            self.u_max,
        )


def main():
    """Entry point."""
    dt = 0.02

    drivetrain = Drivetrain(dt)

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
        r_vl = vref - omegaref * drivetrain.rb
        r_vr = vref + omegaref * drivetrain.rb
        r = next_r
        next_r = np.array([[r_vl], [r_vr]])

        drivetrain.update(r, next_r)

        vc = (drivetrain.x[0, 0] + drivetrain.x[1, 0]) / 2.0
        omega = (drivetrain.x[1, 0] - drivetrain.x[0, 0]) / (2.0 * drivetrain.rb)
        vl = vc - omega * drivetrain.rb
        vr = vc + omega * drivetrain.rb

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
