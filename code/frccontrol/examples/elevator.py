#!/usr/bin/env python3

import frccontrol as frccnt
import matplotlib.pyplot as plt
import numpy as np
import sys


class Elevator(frccnt.System):

    def __init__(self, dt):
        """Elevator subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        """
        state_labels = [("Position", "m"), ("Velocity", "m/s")]
        u_labels = [("Voltage", "V")]
        self.set_plot_labels(state_labels, u_labels)

        # Number of motors
        self.num_motors = 2.0
        # Elevator carriage mass in kg
        self.m = 6.803886
        # Radius of pulley in meters
        self.r = 0.02762679089
        # Gear ratio
        self.G = 42.0 / 12.0 * 40.0 / 14.0

        self.model = frccnt.models.elevator(
            frccnt.models.MOTOR_CIM, self.num_motors, self.m, self.r, self.G)
        frccnt.System.__init__(self, self.model, -12.0, 12.0, dt)

        q = [0.02, 0.4]
        r = [12.0]
        self.design_dlqr_controller(q, r)
        self.design_two_state_feedforward(q, r)

        q_pos = 0.05
        q_vel = 1.0
        r_pos = 0.0001
        self.design_kalman_filter([q_pos, q_vel], [r_pos])


def main():
    dt = 0.00505
    elevator = Elevator(dt)
    elevator.export_cpp_coeffs("Elevator")

    # elevator.plot_pzmaps(1)
    plt.savefig("elevator_pzmaps.svg")

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
            r = np.matrix([[1.524], [0.0]])
        else:
            r = np.matrix([[0.0], [0.0]])
        refs.append(r)

    elevator.plot_time_responses(2, t, refs)
    plt.savefig("elevator_response.svg")

    if "--noninteractive" not in sys.argv:
        plt.show()


if __name__ == "__main__":
    main()
