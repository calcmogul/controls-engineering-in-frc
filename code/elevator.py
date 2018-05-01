#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import wpicontrol as wpicnt


class Elevator(wpicnt.System):

    def __init__(self, dt):
        self.num_motors = 2.0
        # Stall torque in N-m
        self.stall_torque = 2.402 * self.num_motors * 0.60
        # Stall current in A
        self.stall_current = 126.145 * self.num_motors
        # Free speed in RPM
        self.free_speed_rpm = 5015.562
        # Free speed in rotations per second
        self.free_speed = self.free_speed_rpm / 60
        # Free current in Amps
        self.free_current = 1.170 * self.num_motors
        # Robot mass in kg
        self.m = 52
        # Resistance of motor
        self.R = 12.0 / self.stall_current
        # Radius of pulley in meters
        self.r = 0.02762679089
        # Motor velocity constant
        self.Kv = ((self.free_speed * 2.0 * np.pi) /
                   (12.0 - self.R * self.free_current))
        # Torque constant
        self.Kt = self.stall_torque / self.stall_current
        # Gear ratio
        self.G = 42.0 / 12.0 * 40.0 / 14.0

        # State feedback matrices
        # States: [[position], [velocity]]
        # Inputs: [[voltage]]
        # Outputs: [[position]]
        A = np.matrix([[0, 1], [
            0, -self.G**2 * self.Kt / (self.R**2 * self.r * self.m * self.Kv)
        ]])
        B = np.matrix([[0], [self.G * self.Kt / (self.R * self.r * self.m)]])
        C = np.matrix([[1, 0]])
        D = np.matrix([[0]])
        wpicnt.System.__init__(self, A, B, C, D, -12.0, 12.0, dt)

        # Design LQR controller
        Q = self.make_lqr_cost_matrix([0.02, 0.4])
        R = self.make_lqr_cost_matrix([12.0])
        self.design_dlqr_controller(Q, R)


def frange(x, y, jump):
    while x < y:
        yield x
        x += jump


def main():
    dt = 0.00505
    elevator = Elevator(dt)

    # Set up graphing
    l0 = 0.1
    l1 = l0 + 5.0
    l2 = l1 + 0.1
    t = list(frange(0, l2 + 5.0, dt))

    pos = []
    r_pos = []
    vel = []
    u = []

    # Run simulation
    for i in range(len(t)):
        if t[i] < l0:
            elevator.r[0, 0] = 0.0
        elif t[i] < l1:
            elevator.r[0, 0] = 1.524
        else:
            elevator.r[0, 0] = 0.0
        elevator.update()

        # Log states for plotting
        pos.append(elevator.x[0, 0])
        r_pos.append(elevator.r[0, 0])
        vel.append(elevator.x[1, 0])
        u.append(elevator.u[0, 0])

    plt.figure(1)

    # Plot pole-zero map of open-loop system
    plt.subplot(1, 2, 1)
    wpicnt.dpzmap(elevator.sysd, title="Open-loop system")

    # Plot pole-zero map of closed-loop system
    plt.subplot(1, 2, 2)
    wpicnt.dpzmap(
        wpicnt.ss_closed_loop(elevator.sysd, elevator.K),
        title="Closed-loop system")

    plt.figure(2)

    # Plot position over time
    plt.subplot(3, 1, 1)
    plt.title("Time-domain responses")
    plt.plot(t, pos)
    plt.plot(t, r_pos)
    plt.legend(["Position (m)", "Position reference (m)"])

    # Plot velocity over time
    plt.subplot(3, 1, 2)
    plt.plot(t, vel)
    plt.legend(["Velocity (m/s)"])

    # Plot control effort over time
    plt.subplot(3, 1, 3)
    plt.plot(t, u)
    plt.xlabel("Time (s)")
    plt.legend(["Control effort (V)"])

    plt.show()


if __name__ == "__main__":
    main()
