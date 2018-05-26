#!/usr/bin/env python3

import frccontrol as frccnt
import matplotlib.pyplot as plt


class Elevator(frccnt.System):

    def __init__(self, dt):
        """Elevator subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        """
        # Elevator carriage mass in kg
        self.m = 6.803886
        # Radius of pulley in meters
        self.r = 0.02762679089
        # Gear ratio
        self.G = 42.0 / 12.0 * 40.0 / 14.0

        self.model = frccnt.models.elevator(frccnt.models.MOTOR_CIM, self.m,
                                            self.r, self.G)
        frccnt.System.__init__(self, self.model, -12.0, 12.0, dt)

        self.design_dlqr_controller([0.02, 0.4], [12.0])

        q_pos = 0.05
        q_vel = 1.0
        r_pos = 0.0001
        self.design_kalman_filter([q_pos, q_vel], [r_pos])


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
    plt.subplot(2, 2, 1)
    frccnt.dpzmap(elevator.sysd, title="Open-loop system")

    # Plot pole-zero map of closed-loop system
    plt.subplot(2, 2, 2)
    frccnt.dpzmap(frccnt.closed_loop_ctrl(elevator), title="Closed-loop system")

    # Plot observer poles
    plt.subplot(2, 2, 3)
    frccnt.dpzmap(frccnt.closed_loop_obsv(elevator), title="Observer poles")

    plt.figure(2)

    # Plot position over time
    plt.subplot(3, 1, 1)
    plt.title("Time-domain responses")
    plt.plot(t, pos)
    plt.plot(t, r_pos)
    plt.legend(["Position (m)", "Position ref (m)"])

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
