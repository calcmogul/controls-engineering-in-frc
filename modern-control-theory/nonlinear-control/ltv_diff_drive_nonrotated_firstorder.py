#!/usr/bin/env python3

"""
Simulates LTV differential drive controller with linear ODE integration in
field coordinate frame.
"""

import math
import sys

import frccontrol as fct
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import StateSpace
from wpimath.geometry import Pose2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

from bookutil import latex, plotutil

if "--noninteractive" in sys.argv:
    mpl.use("svg")


def linearized_differential_drive(motor, num_motors, m, r, rb, J, Gl, Gr, states):
    """Returns the linearized state-space model for a differential drive.

    States: [[x], [y], [theta], [left velocity], [right velocity]]
    Inputs: [[left voltage], [right voltage]]
    Outputs: [[theta], [left velocity], [right velocity]]

    Keyword arguments:
    motor -- instance of DcBrushedMotor
    num_motors -- number of motors driving the mechanism
    m -- mass of robot in kg
    r -- radius of wheels in meters
    rb -- radius of robot in meters
    J -- moment of inertia of the differential drive in kg-m²
    Gl -- gear ratio of left side of differential drive
    Gr -- gear ratio of right side of differential drive
    states -- state vector around which to linearize model

    Returns:
    StateSpace instance containing continuous model
    """
    motor = fct.models.gearbox(motor, num_motors)

    C1 = -(Gl**2) * motor.Kt / (motor.Kv * motor.R * r**2)
    C2 = Gl * motor.Kt / (motor.R * r)
    C3 = -(Gr**2) * motor.Kt / (motor.Kv * motor.R * r**2)
    C4 = Gr * motor.Kt / (motor.R * r)
    theta = states[2, 0]
    vl = states[3, 0]
    vr = states[4, 0]
    v = (vr + vl) / 2.0
    if abs(v) < 1e-9:
        v = 1e-9
    c = math.cos(theta)
    s = math.sin(theta)
    A = np.array(
        [
            [0, 0, -v * s, 0.5 * c, 0.5 * c],
            [0, 0, v * c, 0.5 * s, 0.5 * s],
            [0, 0, 0, -0.5 / rb, 0.5 / rb],
            [0, 0, 0, (1 / m + rb**2 / J) * C1, (1 / m - rb**2 / J) * C3],
            [0, 0, 0, (1 / m - rb**2 / J) * C1, (1 / m + rb**2 / J) * C3],
        ]
    )
    B = np.array(
        [
            [0, 0],
            [0, 0],
            [0, 0],
            [(1 / m + rb**2 / J) * C2, (1 / m - rb**2 / J) * C4],
            [(1 / m - rb**2 / J) * C2, (1 / m + rb**2 / J) * C4],
        ]
    )
    C = np.array([[0, 0, 1, 0, 0], [0, 0, 0, 1, 0], [0, 0, 0, 0, 1]])
    D = np.zeros((3, 2))

    return StateSpace(A, B, C, D)


class Drivetrain:
    """An frccontrol system for a differential drive."""

    def __init__(self, dt):
        """Differential drive subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        """
        self.dt = dt

        # Radius of robot in meters
        self.rb = 0.59055 / 2.0

        self.plant = self.linearize(np.zeros((5, 1)))

        # States: x position (m), y position (m), heading (rad),
        #         left velocity (m/s), right velocity (m/s)
        # Inputs: left voltage (V), right voltage (V)
        # Outputs: heading (rad), left velocity (m/s), right velocity (m/s)
        self.observer = fct.ExtendedKalmanFilter(
            5,
            2,
            self.f,
            self.h,
            [0.5, 0.5, 10.0, 1.0, 1.0],
            [0.0001, 0.01, 0.01],
            self.dt,
        )

        # Sim variables
        self.x = np.zeros((5, 1))
        self.u = np.zeros((2, 1))
        self.y = np.zeros((3, 1))

        self.u_min = np.array([[-12.0], [-12.0]])
        self.u_max = np.array([[12.0], [12.0]])

    def linearize(self, states):
        """
        Return differential drive model linearized around the given state.

        Keyword arguments:
        states -- state around which to linearize.
        """
        # Number of motors per side
        num_motors = 3.0

        # Gear ratio of differential drive
        G = 60.0 / 11.0

        # Drivetrain mass in kg
        m = 52
        # Radius of wheels in meters
        r = 0.08255 / 2.0
        # Moment of inertia of the differential drive in kg-m²
        J = 6.0

        return linearized_differential_drive(
            fct.models.MOTOR_CIM,
            num_motors,
            m,
            r,
            self.rb,
            J,
            G,
            G,
            states,
        )

    def f(self, x, u):
        """
        Nonlinear differential drive dynamics.

        Keyword arguments:
        x -- state vector
        u -- input vector

        Returns:
        dx/dt -- state derivative
        """
        return (
            np.array(
                [
                    [(x[3, 0] + x[4, 0]) / 2.0 * math.cos(x[2, 0])],
                    [(x[3, 0] + x[4, 0]) / 2.0 * math.sin(x[2, 0])],
                    [(x[4, 0] - x[3, 0]) / (2.0 * self.rb)],
                    [self.plant.A[3, 3] * x[3, 0] + self.plant.A[3, 4] * x[4, 0]],
                    [self.plant.A[4, 3] * x[3, 0] + self.plant.A[4, 4] * x[4, 0]],
                ]
            )
            + self.plant.B @ u
        )

    def h(self, x, u):
        """
        Nonlinear differential drive dynamics.

        Keyword arguments:
        x -- state vector
        u -- input vector

        Returns:
        dx/dt -- state derivative
        """
        return self.plant.C @ x + self.plant.D @ u

    def update(self, r, next_r):
        """
        Advance the model by one timestep.

        Keyword arguments:
        r -- the current reference
        next_r -- the next reference
        """
        # Update sim model
        contA = fct.numerical_jacobian_x(5, 5, self.f, self.x, self.u)
        contB = fct.numerical_jacobian_u(5, 2, self.f, self.x, self.u)
        discA, discB = fct.discretize_ab(contA, contB, self.dt)
        self.x = discA @ self.x + discB @ self.u
        self.y = self.plant.C @ self.x + self.plant.D @ self.u

        # Use linearized prediction instead of RK4 of EKF.predict()
        discA, discQ = fct.discretize_aq(contA, self.observer.contQ, self.dt)
        self.observer.x_hat = discA @ self.observer.x_hat + discB @ self.u
        self.observer.P = discA @ self.observer.P @ discA.T + discQ

        self.observer.correct(self.u, self.y)

        # Feedforward
        rdot = (next_r - r) / self.dt
        u_ff = np.linalg.pinv(self.plant.B) @ (rdot - self.f(r, np.zeros((2, 1))))

        self.plant = self.linearize(self.observer.x_hat)

        K = fct.LinearQuadraticRegulator(
            self.plant.A,
            self.plant.B,
            [0.05, 0.125, 10.0, 0.95, 0.95],
            [12.0, 12.0],
            self.dt,
        ).K

        e = r - self.observer.x_hat
        u_fb = K @ e

        self.u = u_ff + u_fb

        u_cap = np.max(np.abs(self.u))
        if u_cap > 12.0:
            self.u = self.u / u_cap * 12.0


def main():
    """Entry point."""
    dt = 0.02

    # Radius of robot in meters
    rb = 0.59055 / 2.0

    trajectory = TrajectoryGenerator.generateTrajectory(
        [Pose2d(1.330117, 13, 0), Pose2d(10.17, 18, 0)],
        TrajectoryConfig(3.5, 3.5),
    )

    refs = []
    t_rec = np.arange(0, trajectory.totalTime(), dt)
    for t in t_rec:
        sample = trajectory.sample(t)
        vl = sample.velocity - sample.velocity * sample.curvature * rb
        vr = sample.velocity + sample.velocity * sample.curvature * rb
        refs.append(
            np.array(
                [
                    [sample.pose.X()],
                    [sample.pose.Y()],
                    [sample.pose.rotation().radians()],
                    [vl],
                    [vr],
                ]
            )
        )

    x = np.array(
        [[refs[0][0, 0] + 0.5], [refs[0][1, 0] + 0.5], [math.pi / 2], [0], [0]]
    )
    drivetrain = Drivetrain(dt)
    drivetrain.x = x
    drivetrain.observer.x_hat = x

    # Run simulation
    x_rec, r_rec, u_rec, _ = fct.generate_time_responses(drivetrain, refs)

    fig = plt.figure()
    if "--noninteractive" in sys.argv:
        plotutil.plot_xy(
            fig,
            x_rec[0, :],
            x_rec[1, :],
            r_rec[0, :],
            r_rec[1, :],
        )
        latex.savefig("ltv_diff_drive_nonrotated_firstorder_xy")
    else:
        anim = plotutil.animate_xy(  # pragma pylint: disable=unused-variable
            fig,
            x_rec[0, :],
            x_rec[1, :],
            r_rec[0, :],
            r_rec[1, :],
            dt,
        )

    fct.plot_time_responses(
        [
            "x position (m)",
            "y position (m)",
            "Heading (rad)",
            "Left velocity (m/s)",
            "Right velocity (m/s)",
        ],
        ["Left voltage (V)", "Right voltage (V)"],
        t_rec,
        x_rec,
        r_rec,
        u_rec,
    )

    if "--noninteractive" in sys.argv:
        latex.savefig("ltv_diff_drive_nonrotated_firstorder_response")
    else:
        plt.show()


if __name__ == "__main__":
    main()
