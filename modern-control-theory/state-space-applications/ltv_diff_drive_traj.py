#!/usr/bin/env python3

# Runs LTV differential drive simulation

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
    import bookutil.latex as latex

import control as ct
import frccontrol as fct
import math
import matplotlib.pyplot as plt
import numpy as np

from bookutil.drivetrain import get_diff_vels
from bookutil.systems import LTVDifferentialDrive


def linearized_differential_drive(motor, num_motors, m, r, rb, J, Gl, Gr, states):
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
    J -- moment of inertia of the differential drive in kg-m²
    Gl -- gear ratio of left side of differential drive
    Gr -- gear ratio of right side of differential drive
    states -- state vector around which to linearize model

    Returns:
    StateSpace instance containing continuous model
    """
    motor = fct.models.gearbox(motor, num_motors)

    C1 = -(Gl ** 2) * motor.Kt / (motor.Kv * motor.R * r ** 2)
    C2 = Gl * motor.Kt / (motor.R * r)
    C3 = -(Gr ** 2) * motor.Kt / (motor.Kv * motor.R * r ** 2)
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
    A = np.array([[0, 0, 0, 0.5, 0.5],
                  [0, 0, v, 0, 0],
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


class DifferentialDrive(LTVDifferentialDrive):
    def __init__(self, dt, states):
        """Differential drive subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        states -- state vector around which to linearize model
        """
        LTVDifferentialDrive.__init__(self, dt, states)

    def create_model(self, states, inputs):
        """Relinearize model around given state.

        Keyword arguments:
        states -- state vector around which to linearize model
        inputs -- input vector around which to linearize model

        Returns:
        StateSpace instance containing continuous state-space model
        """
        # Number of motors per side
        num_motors = 3.0

        # Gear ratio of differential drive
        G = 60.0 / 11.0

        # Drivetrain mass in kg
        m = 52
        # Radius of wheels in meters
        r = 0.08255 / 2.0
        # Radius of robot in meters
        self.rb = 0.59055 / 2.0
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

    def update_controller(self, next_r):
        self.design_controller_observer()

        rot = self.x_hat[2, 0]
        in_robot_frame = np.array(
            [
                [math.cos(rot), math.sin(rot), 0, 0, 0],
                [-math.sin(rot), math.cos(rot), 0, 0, 0],
                [0, 0, 1, 0, 0],
                [0, 0, 0, 1, 0],
                [0, 0, 0, 0, 1],
            ]
        )
        e = self.r - self.x_hat
        u = self.K @ in_robot_frame @ e
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

    state_rec, ref_rec, u_rec, y_rec = diff_drive.generate_time_responses(t, refs)

    plt.figure(1)
    x_rec = np.squeeze(state_rec[0:1, :])
    y_rec = np.squeeze(state_rec[1:2, :])
    plt.plot(x_rec, y_rec, label="LTV controller")
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
        latex.savefig("ltv_diff_drive_traj_xy")

    diff_drive.plot_time_responses(t, state_rec, ref_rec, u_rec)

    if "--noninteractive" in sys.argv:
        latex.savefig("ltv_diff_drive_traj_response")
    else:
        plt.show()


if __name__ == "__main__":
    main()
