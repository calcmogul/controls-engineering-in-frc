#!/usr/bin/env python3

# Runs linearized differential drive simulation

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
    import utils.latex as latex

import control as ct
import frccontrol as fct
import math
import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


def differential_drive(motor, num_motors, m, r, rb, J, Gl, Gr, states):
    """Returns the state-space model for a differential drive.

    States: [[x], [y], [theta], [left velocity], [right velocity]]
    Inputs: [[left voltage], [right voltage]]
    Outputs: [[theta], [left velocity], [right velocity]]

    Keyword arguments:
    motor -- instance of DcBrushedMotor
    num_motors -- number of motors driving the mechanism
    m -- mass of robot in kg
    r -- radius of wheels in meters
    rb -- radius of robot in meters
    J -- moment of inertia of the differential drive in kg-m^2
    Gl -- gear ratio of left side of differential drive
    Gr -- gear ratio of right side of differential drive
    states -- state vector around which to linearize model

    Returns:
    StateSpace instance containing continuous model
    """
    motor = fct.models.gearbox(motor, num_motors)

    C1 = -Gl ** 2 * motor.Kt / (motor.Kv * motor.R * r ** 2)
    C2 = Gl * motor.Kt / (motor.R * r)
    C3 = -Gr ** 2 * motor.Kt / (motor.Kv * motor.R * r ** 2)
    C4 = Gr * motor.Kt / (motor.R * r)
    x = states[0, 0]
    y = states[1, 0]
    theta = states[2, 0]
    vl = states[3, 0]
    vr = states[4, 0]
    v = (vr + vl) / 2.0
    if abs(v) < 5e-8:
        vl = 5e-8
        vr = 5e-8
        v = 5e-8
    # fmt: off
    A = np.array([[0, 0, -v * math.sin(theta), 0.5 * math.cos(theta), 0.5 * math.cos(theta)],
                  [0, 0, v * math.cos(theta), 0.5 * math.sin(theta), 0.5 * math.sin(theta)],
                  [0, 0, 0, -0.5 / rb, 0.5 / rb],
                  [0, 0, 0, (1 / m + rb**2 / J) * C1, (1 / m - rb**2 / J) * C3],
                  [0, 0, 0, (1 / m - rb**2 / J) * C1, (1 / m + rb**2 / J) * C3]])
    B = np.array([[0, 0],
                  [0, 0],
                  [0, 0],
                  [(1 / m + rb**2 / J) * C2, (1 / m - rb**2 / J) * C4],
                  [(1 / m - rb**2 / J) * C2, (1 / m + rb**2 / J) * C4]])
    C = np.array([[0, 0, 1, 0, 0],
                  [0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 1]])
    D = np.array([[0, 0],
                  [0, 0],
                  [0, 0]])
    # fmt: on

    return ct.StateSpace(A, B, C, D, remove_useless=False)


class DifferentialDrive(fct.System):
    def __init__(self, dt, states):
        """Differential drive subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        states -- state vector around which to linearize model
        """
        state_labels = [
            ("x position", "m"),
            ("y position", "m"),
            ("Heading", "rad"),
            ("Left velocity", "m/s"),
            ("Right velocity", "m/s"),
        ]
        u_labels = [("Left voltage", "V"), ("Right voltage", "V")]
        self.set_plot_labels(state_labels, u_labels)

        u_min = np.array([[-12.0], [-12.0]])
        u_max = np.array([[12.0], [12.0]])

        f = (
            lambda x, u: np.array(
                [
                    [(x[3, 0] + x[4, 0]) / 2.0 * math.cos(x[2, 0])],
                    [(x[3, 0] + x[4, 0]) / 2.0 * math.sin(x[2, 0])],
                    [(x[4, 0] - x[3, 0]) / (2.0 * self.rb)],
                    [self.sysc.A[3, 3] * x[3, 0] + self.sysc.A[3, 4] * x[4, 0]],
                    [self.sysc.A[4, 3] * x[3, 0] + self.sysc.A[4, 4] * x[4, 0]],
                ]
            )
            + self.sysc.B @ u
        )
        fct.System.__init__(
            self, u_min, u_max, dt, states, np.zeros((2, 1)), nonlinear_func=f
        )

    def create_model(self, states, inputs):
        """Relinearize model around given state.

        Keyword arguments:
        states -- state vector around which to linearize model
        inputs -- input vector around which to linearize model

        Returns:
        StateSpace instance containing continuous state-space model
        """
        self.in_low_gear = False

        # Number of motors per side
        num_motors = 3.0

        # High and low gear ratios of differential drive
        Glow = 60.0 / 11.0
        Ghigh = 60.0 / 11.0

        # Differential drive mass in kg
        m = 52
        # Radius of wheels in meters
        r = 0.08255 / 2.0
        # Radius of robot in meters
        self.rb = 0.59055 / 2.0
        # Moment of inertia of the differential drive in kg-m^2
        J = 6.0

        # Gear ratios of left and right sides of differential drive respectively
        if self.in_low_gear:
            Gl = Glow
            Gr = Glow
        else:
            Gl = Ghigh
            Gr = Ghigh

        return differential_drive(
            fct.models.MOTOR_CIM,
            num_motors,
            m,
            r,
            self.rb,
            J,
            Gl,
            Gr,
            np.asarray(states),
        )

    def design_controller_observer(self):
        if self.in_low_gear:
            q_vel = 1.0
        else:
            q_vel = 0.95

        q_x = 0.0625
        q_y = 0.125
        q_heading = 10.0

        q = [q_x, q_y, q_heading, q_vel, q_vel]
        r = [12.0, 12.0]
        self.design_lqr(q, r)

        self.design_two_state_feedforward()

        q_pos = 0.5
        q_vel = 1.0
        r_gyro = 0.0001
        r_vel = 0.01
        self.design_kalman_filter(
            [q_pos, q_pos, q_heading, q_vel, q_vel], [r_gyro, r_vel, r_vel]
        )


def main():
    t = []
    refs = []

    dt = 0.00505

    # Radius of robot in meters
    rb = 0.59055 / 2.0

    state_labels = ["$x$", "$y$", "$\\theta$", "$v_l$", "$v_r$"]
    input_labels = ["Left voltage", "Right voltage"]

    v_range = [-1.1, 1.1]
    theta_range = [-np.pi, np.pi]
    v_dstate = 0.1
    theta_dstate = 0.1

    x, y = np.mgrid[
        v_range[0] : v_range[1] : v_dstate,
        theta_range[0] : theta_range[1] : theta_dstate,
    ]

    K_rec = np.zeros((2, 5, x.shape[0], y.shape[1]))
    for i in range(x.shape[0]):
        v = x[i, 0]
        for j in range(y.shape[1]):
            theta = y[0, j]
            x_linear = np.array([[0], [0], [theta], [v], [v]])
            diff_drive = DifferentialDrive(dt, x_linear)
            K_rec[:, :, i, j] = diff_drive.K

    x_label = "v (m/s)"
    y_label = "$\\theta$ (rad)"

    for i in range(len(state_labels)):
        fig = plt.figure(i + 1)
        ax = fig.add_subplot(111, projection="3d")
        z = K_rec[0, i, :, :].reshape(x.shape)
        label1 = f"{input_labels[0]}"
        ax.plot_surface(x, y, z, label=label1)
        z = K_rec[1, i, :, :].reshape(x.shape)
        label2 = f"{input_labels[1]}"
        ax.plot_surface(x, y, z, label=label2)
        ax.set_xlabel(x_label)
        ax.set_ylabel(y_label)
        ax.set_zlabel(f"Gain from {state_labels[i]} error to input")

        colors = ["blue", "orange"]
        scatter1_proxy = mpl.lines.Line2D(
            [0], [0], linestyle="none", c=colors[0], marker="o"
        )
        scatter2_proxy = mpl.lines.Line2D(
            [0], [0], linestyle="none", c=colors[1], marker="o"
        )
        ax.legend([scatter1_proxy, scatter2_proxy], [label1, label2], numpoints=1)
    plt.show()


if __name__ == "__main__":
    main()
