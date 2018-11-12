#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
    import latexutils

import frccontrol as frccnt
import matplotlib.pyplot as plt
import numpy as np

plt.rc("text", usetex=True)


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
            frccnt.models.MOTOR_CIM, self.num_motors, self.m, self.r, self.G
        )
        frccnt.System.__init__(self, self.model, -12.0, 12.0, dt)

        q = [0.02, 0.4]
        r = [12.0]
        self.design_dlqr_controller(q, r)
        self.design_two_state_feedforward(q, r)

        q_pos = 0.05
        q_vel = 1.0
        r_pos = 0.0001
        self.design_kalman_filter([q_pos, q_vel], [r_pos])


def generate_refs(dt):
    # Set up graphing
    l0 = 0.1
    l1 = l0 + 5.0
    l2 = l1 + 0.1
    l3 = l2 + 1.0
    t = np.arange(0, l3, dt)

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

    return t, refs


def simulate(elevator, dt, method):
    t, refs = generate_refs(dt)
    elevator.sysd = elevator.sysc.sample(dt, method)
    elevator.x = np.zeros((elevator.x.shape[0], 1))
    elevator.x_hat = np.zeros((elevator.x_hat.shape[0], 1))
    state_rec, ref_rec, u_rec = elevator.generate_time_responses(t, refs)

    pos = elevator.extract_row(state_rec, 0)
    if method == "zoh":
        label = "Zero-order hold"
    elif method == "euler":
        label = "Forward Euler"
    elif method == "backward_diff":
        label = "Backward Euler"
    elif method == "bilinear":
        label = "Bilinear transform"
    label += " (T={}s)".format(dt)
    plt.plot(t, pos, label=label)


def main():
    elevator = Elevator(0.1)

    plt.figure(1)
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    simulate(elevator, 0.1, "zoh")
    simulate(elevator, 0.1, "euler")
    simulate(elevator, 0.1, "backward_diff")
    simulate(elevator, 0.1, "bilinear")
    plt.ylim([-2, 3])
    plt.legend()
    if "--noninteractive" in sys.argv:
        latexutils.savefig("sampling_simulation_010")

    plt.figure(2)
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    simulate(elevator, 0.05, "zoh")
    simulate(elevator, 0.05, "euler")
    simulate(elevator, 0.05, "backward_diff")
    simulate(elevator, 0.05, "bilinear")
    plt.ylim([-2, 3])
    plt.legend()
    if "--noninteractive" in sys.argv:
        latexutils.savefig("sampling_simulation_005")

    plt.figure(3)
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    simulate(elevator, 0.01, "zoh")
    simulate(elevator, 0.01, "euler")
    simulate(elevator, 0.01, "backward_diff")
    simulate(elevator, 0.01, "bilinear")
    plt.ylim([-0.25, 2])
    plt.legend()
    if "--noninteractive" in sys.argv:
        latexutils.savefig("sampling_simulation_004")
    else:
        plt.show()


if __name__ == "__main__":
    main()
