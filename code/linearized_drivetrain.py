#!/usr/bin/env python3

# Runs linearized drivetrain simulation

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
    import latexutils

import control as cnt
import frccontrol as frccnt
import math
import matplotlib.pyplot as plt
import numpy as np


def drivetrain(motor, num_motors, m, r, rb, J, Gl, Gr, states):
    """Returns the state-space model for a drivetrain.

    States: [[x], [y], [theta], [left velocity], [right velocity]]
    Inputs: [[left voltage], [right voltage]]
    Outputs: [[theta], [left velocity], [right velocity]]

    Keyword arguments:
    motor -- instance of DcBrushedMotor
    num_motors -- number of motors driving the mechanism
    m -- mass of robot in kg
    r -- radius of wheels in meters
    rb -- radius of robot in meters
    J -- moment of inertia of the drivetrain in kg-m^2
    Gl -- gear ratio of left side of drivetrain
    Gr -- gear ratio of right side of drivetrain
    states -- state vector around which to linearize model

    Returns:
    StateSpace instance containing continuous model
    """
    motor = frccnt.models.gearbox(motor, num_motors)

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
    if abs(v) < 1e-9:
        vl = 1e-9
        vr = 1e-9
        v = 1e-9
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

    return cnt.ss(A, B, C, D)


def get_diff_vels(v, omega, d):
    """Returns left and right wheel velocities given a central velocity and
    turning rate.
    Keyword arguments:
    v -- center velocity
    omega -- center turning rate
    d -- trackwidth
    """
    return v - omega * d / 2.0, v + omega * d / 2.0


class Drivetrain(frccnt.System):
    def __init__(self, dt, states):
        """Drivetrain subsystem.

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

        frccnt.System.__init__(self, states, u_min, u_max, dt, nonlinear=True)

    def create_model(self, states):
        """Relinearize model around given state.

        Keyword arguments:
        states -- state vector around which to linearize model

        Returns:
        StateSpace instance containing continuous state-space model
        """
        self.in_low_gear = False

        # Number of motors per side
        num_motors = 3.0

        # High and low gear ratios of drivetrain
        Glow = 60.0 / 11.0
        Ghigh = 60.0 / 11.0

        # Drivetrain mass in kg
        m = 52
        # Radius of wheels in meters
        r = 0.08255 / 2.0
        # Radius of robot in meters
        self.rb = 0.59055 / 2.0
        # Moment of inertia of the drivetrain in kg-m^2
        J = 6.0

        # Gear ratios of left and right sides of drivetrain respectively
        if self.in_low_gear:
            Gl = Glow
            Gr = Glow
        else:
            Gl = Ghigh
            Gr = Ghigh

        return drivetrain(
            frccnt.models.MOTOR_CIM,
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

        q_pos = 0.5
        q_heading = 10

        q = [q_pos, q_pos, q_heading, q_vel, q_vel]
        r = [12.0, 12.0]
        self.design_lqr(q, r)

        qff_pos = 1.0
        qff_heading = 1.0
        qff_vel = 1.0
        self.design_two_state_feedforward(
            [qff_pos, qff_pos, qff_heading, qff_vel, qff_vel],
            [float("inf"), float("inf")],
        )

        q_vel = 1.0
        r_gyro = 0.0001
        r_vel = 0.01
        self.design_kalman_filter(
            [q_pos, q_pos, q_heading, q_vel, q_vel], [r_gyro, r_vel, r_vel]
        )


def main():
    t = []
    refs = []

    # Radius of robot in meters
    rb = 0.59055 / 2.0

    with open("linearized_drivetrain.csv", "r") as trajectory_file:
        import csv

        current_t = 0

        reader = csv.reader(trajectory_file, delimiter=",")
        trajectory_file.readline()
        for row in reader:
            t.append(float(row[0]))
            vl, vr = get_diff_vels(float(row[4]), float(row[5]), rb * 2.0)

            ref = np.array(
                [[float(row[1])], [float(row[2])], [float(row[3])], [vl], [vr]]
            )
            refs.append(ref)

    dt = 0.00505
    x = np.array([[refs[0][0, 0]], [refs[0][1, 0]], [0], [0], [0]])
    drivetrain = Drivetrain(dt, x)

    state_rec, ref_rec, u_rec = drivetrain.generate_time_responses(t, refs)

    plt.figure(1)
    plt.plot(ref_rec[0, :], ref_rec[1, :], label="Reference trajectory")
    x_rec = np.squeeze(np.asarray(state_rec[0, :]))
    y_rec = np.squeeze(np.asarray(state_rec[1, :]))
    plt.plot(x_rec, y_rec, label="Linearized controller")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.legend()

    # Equalize aspect ratio
    xlim = plt.xlim()
    width = abs(xlim[0]) + abs(xlim[1])
    ylim = plt.ylim()
    height = abs(ylim[0]) + abs(ylim[1])
    if width > height:
        plt.ylim([-width / 2, width / 2])
    else:
        plt.xlim([-height / 2, height / 2])

    if "--noninteractive" in sys.argv:
        latexutils.savefig("linearized_drivetrain_xy")

    plt.figure(2)
    drivetrain.plot_time_responses(t, state_rec, ref_rec, u_rec)

    if "--noninteractive" in sys.argv:
        latexutils.savefig("linearized_drivetrain_response")
    else:
        plt.show()


if __name__ == "__main__":
    main()
