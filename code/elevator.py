#!/usr/bin/env python3

import control as cnt
import matplotlib.pyplot as plt
import numpy as np


def frange(x, y, jump):
    while x < y:
        yield x
        x += jump


class Elevator:

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

        # States: [position; velocity]
        # Inputs: [voltage]
        # Outputs: [position]
        self.A_continuous = np.matrix([[0, 1], [
            0, -self.G**2 * self.Kt / (self.R**2 * self.r * self.m * self.Kv)
        ]])

        self.B_continuous = np.matrix(
            [[0], [self.G * self.Kt / (self.R * self.r * self.m)]])
        self.C = np.matrix([[1, 0]])
        self.D = np.matrix([[0]])
        self.sysc = cnt.ss(self.A_continuous, self.B_continuous, self.C, self.D)

        # Discretize model
        self.sysd = cnt.sample_system(self.sysc, dt)

        self.x = np.matrix([[0], [0]])
        self.u = np.matrix([[0]])

        # Create controller
        meter_to_inch = 39.37008
        Kp = 0.09 * meter_to_inch
        Ki = 0.004 * meter_to_inch
        Kd = 0.0 * meter_to_inch
        self.K = np.matrix([[Kp * 12.0, Kd * 12.0]])
        self.r = np.matrix([[0], [0]])
        self.U_min = -12.0
        self.U_max = 12.0

    def update(self):
        self.u = np.clip(self.K * (self.r - self.x), self.U_min, self.U_max)
        self.x = self.sysd.A * self.x + self.sysd.B * self.u
        self.y = self.sysd.C * self.x + self.sysd.D * self.u


def main():
    dt = 0.01
    elevator = Elevator(dt)

    # Set up graphing
    l0 = 0.1
    l1 = l0 + 5.0
    l2 = l1 + 0.1
    t = list(frange(0, l2 + 5.0, dt))

    pos = []
    r_pos = []
    vel = []

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

    # Generate plots
    plt.plot(t, pos)
    plt.plot(t, r_pos)
    plt.xlabel("Time (s)")
    plt.legend(["Position", "Position reference", "Velocity"])
    plt.show()


if __name__ == "__main__":
    main()
