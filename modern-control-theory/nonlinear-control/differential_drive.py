#!/usr/bin/env python3

import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
    import bookutil.latex as latex

import frccontrol as fct
import matplotlib.pyplot as plt
import numpy as np


class DifferentialDrive(fct.System):
    def __init__(self, dt):
        """DifferentialDrive subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        """
        state_labels = [
            ("Left position", "m"),
            ("Left velocity", "m/s"),
            ("Right position", "m"),
            ("Right velocity", "m/s"),
        ]
        u_labels = [("Left voltage", "V"), ("Right voltage", "V")]
        self.set_plot_labels(state_labels, u_labels)

        u_min = np.array([[-12.0], [-12.0]])
        u_max = np.array([[12.0], [12.0]])
        fct.System.__init__(self, u_min, u_max, dt, np.zeros((4, 1)), np.zeros((2, 1)))

    def create_model(self, states, inputs):
        self.in_low_gear = False

        # Number of motors per side
        num_motors = 2.0

        # High and low gear ratios of differential drive
        Glow = 60.0 / 11.0
        Ghigh = 60.0 / 11.0

        # Drivetrain mass in kg
        m = 52
        # Radius of wheels in meters
        r = 0.08255 / 2.0
        # Radius of robot in meters
        rb = 0.59055 / 2.0
        # Moment of inertia of the differential drive in kg-m^2
        J = 6.0

        # Gear ratios of left and right sides of differential drive respectively
        if self.in_low_gear:
            Gl = Glow
            Gr = Glow
        else:
            Gl = Ghigh
            Gr = Ghigh

        return fct.models.differential_drive(
            fct.models.MOTOR_CIM, num_motors, m, r, rb, J, Gl, Gr
        )

    def design_controller_observer(self):
        if self.in_low_gear:
            q_pos = 0.12
            q_vel = 1.0
        else:
            q_pos = 0.14
            q_vel = 0.95

        q = [q_pos, q_vel, q_pos, q_vel]
        r = [12.0, 12.0]
        self.design_lqr(q, r)
        self.design_two_state_feedforward()

        q_pos = 0.05
        q_vel = 1.0
        q_voltage = 10.0
        q_encoder_uncertainty = 2.0
        r_pos = 0.0001
        r_gyro = 0.000001
        self.design_kalman_filter([q_pos, q_vel, q_pos, q_vel], [r_pos, r_pos])


def main():
    dt = 0.005
    diff_drive = DifferentialDrive(dt)

    t, xprof, vprof, aprof = fct.generate_trapezoid_profile(
        max_v=3.5, time_to_max_v=1.0, dt=dt, goal=50.0
    )

    # Generate references for simulation
    refs = []
    for i in range(len(t)):
        r = np.array([[xprof[i]], [vprof[i]], [xprof[i]], [vprof[i]]])
        refs.append(r)

    x_rec, ref_rec, u_rec, y_rec = diff_drive.generate_time_responses(t, refs)
    diff_drive.plot_time_responses(t, x_rec, ref_rec, u_rec)
    if "--noninteractive" in sys.argv:
        latex.savefig("differential_drive_response")
    else:
        plt.show()


if __name__ == "__main__":
    main()
