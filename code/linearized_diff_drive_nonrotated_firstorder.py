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
import matplotlib.pyplot as plt
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

    return ct.StateSpace(A, B, C, D, remove_useless=False)


def get_diff_vels(v, omega, d):
    """Returns left and right wheel velocities given a central velocity and
    turning rate.
    Keyword arguments:
    v -- center velocity
    omega -- center turning rate
    d -- trackwidth
    """
    return v - omega * d / 2.0, v + omega * d / 2.0


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

    def relinearize(self, Q_elems, R_elems, states, inputs):
        from frccontrol import lqr

        sysc = self.create_model(states, inputs)
        sysd = sysc.sample(self.dt)

        Q = np.diag(1.0 / np.square(Q_elems))
        R = np.diag(1.0 / np.square(R_elems))
        return lqr(sysd, Q, R)

    def design_controller_observer(self):
        if self.in_low_gear:
            q_vel = 1.0
        else:
            q_vel = 0.95

        q_x = 0.05
        q_y = 0.125
        q_heading = 10.0

        q = [q_x, q_y, q_heading, q_vel, q_vel]
        r = [12.0, 12.0]

        self.K = self.relinearize(q, r, self.x_hat, self.u)

        self.design_two_state_feedforward()

        q_pos = 0.5
        q_vel = 1.0
        r_gyro = 0.0001
        r_vel = 0.01
        self.design_kalman_filter(
            [q_pos, q_pos, q_heading, q_vel, q_vel], [r_gyro, r_vel, r_vel]
        )

    def update_plant(self):
        self.sysc = self.create_model(self.x, self.u)
        self.sysd = self.sysc.sample(self.dt)

        self.x = self.sysd.A @ self.x + self.sysd.B @ self.u
        self.y = self.sysd.C @ self.x + self.sysd.D @ self.u

    def update_controller(self, next_r):
        self.design_controller_observer()

        u = self.K @ (self.r - self.x_hat)
        rdot = (next_r - self.r) / self.dt
        uff = self.Kff @ (rdot - self.f(self.x_hat, np.zeros(self.u.shape)))
        self.r = next_r
        self.u = u + uff

        u_cap = np.max(np.abs(self.u))
        if u_cap > 12.0:
            self.u = self.u / u_cap * 12.0


def main():
    t = []
    refs = []

    # Radius of robot in meters
    rb = 0.59055 / 2.0

    with open("ramsete_traj.csv", "r") as trajectory_file:
        import csv

        current_t = 0

        reader = csv.reader(trajectory_file, delimiter=",")
        trajectory_file.readline()
        for row in reader:
            t.append(float(row[0]))
            x = float(row[1])
            y = float(row[2])
            theta = float(row[3])
            vl, vr = get_diff_vels(float(row[4]), float(row[5]), rb * 2.0)
            ref = np.array([[x], [y], [theta], [vl], [vr]])
            refs.append(ref)

    dt = 0.02
    x = np.array([[refs[0][0, 0] + 0.5], [refs[0][1, 0] + 0.5], [np.pi / 2], [0], [0]])
    diff_drive = DifferentialDrive(dt, x)

    state_rec, ref_rec, u_rec = diff_drive.generate_time_responses(t, refs)

    plt.figure(1)
    x_rec = np.squeeze(np.asarray(state_rec[0, :]))
    y_rec = np.squeeze(np.asarray(state_rec[1, :]))
    plt.plot(x_rec, y_rec, label="Linearized controller")
    plt.plot(ref_rec[0, :], ref_rec[1, :], label="Reference trajectory")
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
        latex.savefig("linearized_diff_drive_nonrotated_firstorder_xy")

    plt.figure(2)
    diff_drive.plot_time_responses(t, state_rec, ref_rec, u_rec)

    if "--noninteractive" in sys.argv:
        latex.savefig("linearized_diff_drive_nonrotated_firstorder_response")
    else:
        plt.show()


if __name__ == "__main__":
    main()
