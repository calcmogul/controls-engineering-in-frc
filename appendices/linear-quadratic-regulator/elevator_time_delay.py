#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
import bookutil.latex as latex

import frccontrol as fct
import matplotlib.pyplot as plt
import numpy as np
import scipy as sp

plt.rc("text", usetex=True)

DT = 0.005
DELAY = 0.05


class Elevator(fct.System):
    def __init__(self, dt, latency_comp=False):
        """Elevator subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        latency_comp -- True if the controller gain should be latency-compensated
        """
        self.latency_comp = latency_comp

        state_labels = [("Position", "m"), ("Velocity", "m/s")]
        u_labels = [("Voltage", "V")]
        self.set_plot_labels(state_labels, u_labels)

        fct.System.__init__(
            self,
            np.array([[-12.0]]),
            np.array([[12.0]]),
            dt,
            np.zeros((2, 1)),
            np.zeros((1, 1)),
        )

    def create_model(self, states, inputs):
        # Number of motors
        num_motors = 2.0
        # Elevator carriage mass in kg
        m = 6.803886
        # Radius of pulley in meters
        r = 0.02762679089
        # Gear ratio
        G = 42.0 / 12.0 * 40.0 / 14.0

        return fct.models.elevator(fct.models.MOTOR_CIM, num_motors, m, r, G)

    def design_controller_observer(self):
        self.design_two_state_feedforward()
        self.design_lqr([0.02, 0.4], [12.0])

        q_pos = 0.05
        q_vel = 1.0
        r_pos = 0.0001
        self.design_kalman_filter([q_pos, q_vel], [r_pos])

        self.ubuf = []
        for i in range(int(DELAY / DT)):
            self.ubuf.append(np.zeros((1, 1)))

        if self.latency_comp:
            self.K = self.K @ sp.linalg.fractional_matrix_power(
                self.sysd.A - self.sysd.B @ self.K, DELAY / DT
            )

    def update_controller(self, next_r):
        u = self.K @ (self.r - self.x_hat)
        if self.f:
            rdot = (next_r - self.r) / self.dt
            uff = self.Kff @ (rdot - self.f(self.r, np.zeros(self.u.shape)))
        else:
            uff = self.Kff @ (next_r - self.sysd.A @ self.r)
        self.r = next_r
        self.u = np.clip(u + uff, self.u_min, self.u_max)
        self.ubuf.append(self.u)
        self.u = self.ubuf.pop(0)


def main():
    # Set up graphing
    l0 = 0.1
    l1 = l0 + 5.0
    l2 = l1 + 0.1
    t = np.arange(0, l2 + 5.0, DT)

    refs = []

    # Generate references for simulation
    for i in range(len(t)):
        if t[i] < l0:
            r = np.array([[0.0], [0.0]])
        elif t[i] < l1:
            r = np.array([[1.524], [0.0]])
        else:
            r = np.array([[0.0], [0.0]])
        refs.append(r)

    elevator = Elevator(DT)
    x_rec, ref_rec, u_rec, y_rec = elevator.generate_time_responses(t, refs)
    latex.plot_time_responses(elevator, t, x_rec, ref_rec, u_rec, 2)
    if "--noninteractive" in sys.argv:
        latex.savefig("elevator_time_delay_no_comp")

    elevator = Elevator(DT, latency_comp=True)
    x_rec, ref_rec, u_rec, y_rec = elevator.generate_time_responses(t, refs)
    latex.plot_time_responses(elevator, t, x_rec, ref_rec, u_rec, 2)
    if "--noninteractive" in sys.argv:
        latex.savefig("elevator_time_delay_comp")
    else:
        plt.show()


if __name__ == "__main__":
    main()
