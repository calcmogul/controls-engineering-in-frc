#!/usr/bin/env python3

import frccontrol as frccnt
import math
import matplotlib.pyplot as plt
import numpy as np
import sys


class SingleJointedArm(frccnt.System):

    def __init__(self, dt):
        """Single-jointed arm subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        """
        state_labels = [("Angle", "rad"), ("Angular velocity", "rad/s")]
        u_labels = [("Voltage", "V")]
        self.set_plot_labels(state_labels, u_labels)

        # Number of motors
        self.num_motors = 1.0
        # Mass of arm in kg
        self.m = 2.2675
        # Length of arm in m
        self.l = 1.2192
        # Arm moment of inertia in kg-m^2
        self.J = 1 / 3 * self.m * self.l**2
        # Gear ratio
        self.G = 1.0 / 20.0

        self.model = frccnt.models.single_jointed_arm(
            frccnt.models.MOTOR_CIM, self.num_motors, self.J, self.G)
        frccnt.System.__init__(self, self.model, -12.0, 12.0, dt)

        q_pos = 0.01745
        q_vel = 0.08726
        self.design_dlqr_controller([q_pos, q_vel], [12.0])
        self.design_two_state_feedforward([q_pos, q_vel], [12.0])

        q_pos = 0.01745
        q_vel = 0.1745329
        r_pos = 0.01
        r_vel = 0.01
        self.design_kalman_filter([q_pos, q_vel], [r_pos])


def main():
    dt = 0.00505
    single_jointed_arm = SingleJointedArm(dt)
    single_jointed_arm.export_cpp_coeffs("SingleJointedArm")

    # single_jointed_arm.plot_pzmaps(1)
    plt.savefig("single_jointed_arm_pzmaps.svg")

    # Set up graphing
    l0 = 0.1
    l1 = l0 + 5.0
    l2 = l1 + 0.1
    t = np.linspace(0, l2 + 5.0, (l2 + 5.0) / dt)

    refs = []

    # Generate references for simulation
    for i in range(len(t)):
        if t[i] < l0:
            r = np.matrix([[0.0], [0.0]])
        elif t[i] < l1:
            r_pos = min(0.8726646 * (t[i] - l0) / (l1 - l0), 0.8726646)
            r_vel = 0.8726646 / (l1 - l0)
            r = np.matrix([[r_pos], [r_vel]])
        else:
            r = np.matrix([[0.0], [0.0]])
        refs.append(r)

    single_jointed_arm.plot_time_responses(2, t, refs)
    plt.savefig("single_jointed_arm_response.svg")

    if "--noninteractive" not in sys.argv:
        plt.show()


if __name__ == "__main__":
    main()
