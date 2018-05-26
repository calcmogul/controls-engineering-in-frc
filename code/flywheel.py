#!/usr/bin/env python3

import frccontrol as frccnt
import math
import matplotlib.pyplot as plt


class Flywheel(frccnt.System):

    def __init__(self, dt):
        """Flywheel subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        """
        # Flywheel moment of inertia in kg-m^2
        self.J = 0.00032
        # Gear ratio
        self.G = 12.0 / 18.0

        self.model = frccnt.models.flywheel(frccnt.models.MOTOR_775PRO, self.J,
                                            self.G)
        frccnt.System.__init__(self, self.model, -12.0, 12.0, dt)

        self.design_dlqr_controller([9.42], [12.0])
        print("K=", self.K)
        # self.place_controller_poles([0.87])
        # print("Placed K=", self.K)

        q_vel = 1.0
        r_vel = 0.01
        self.design_kalman_filter([q_vel], [r_vel])
        print("L=", self.L)
        # self.place_observer_poles([0.3])
        # print("Placed L=", self.L)


def frange(x, y, jump):
    while x < y:
        yield x
        x += jump


def main():
    dt = 0.00505
    flywheel = Flywheel(dt)

    # Set up graphing
    l0 = 0.1
    l1 = l0 + 5.0
    l2 = l1 + 0.1
    t = list(frange(0, l2 + 5.0, dt))

    vel = []
    r_vel = []
    u = []

    # Run simulation
    for i in range(len(t)):
        if t[i] < l0:
            flywheel.r[0, 0] = 0.0
        elif t[i] < l1:
            flywheel.r[0, 0] = 9000 / 60 * 2 * math.pi
        else:
            flywheel.r[0, 0] = 0.0
        flywheel.update()

        # Log states for plotting
        vel.append(flywheel.x[0, 0])
        r_vel.append(flywheel.r[0, 0])
        u.append(flywheel.u[0, 0])

    plt.figure(1)

    # Plot pole-zero map of open-loop system
    plt.subplot(2, 2, 1)
    frccnt.dpzmap(flywheel.sysd, title="Open-loop system")

    # Plot pole-zero map of closed-loop system
    plt.subplot(2, 2, 2)
    frccnt.dpzmap(frccnt.closed_loop_ctrl(flywheel), title="Closed-loop system")

    # Plot observer poles
    plt.subplot(2, 2, 3)
    frccnt.dpzmap(frccnt.closed_loop_obsv(flywheel), title="Observer poles")

    plt.figure(2)

    # Plot angular velocity over time
    plt.subplot(2, 1, 1)
    plt.title("Time-domain responses")
    plt.plot(t, vel)
    plt.plot(t, r_vel)
    plt.legend(["Angular velocity (rad/s)", "Angular velocity ref (rad/s)"])

    # Plot control effort over time
    plt.subplot(2, 1, 2)
    plt.plot(t, u)
    plt.xlabel("Time (s)")
    plt.legend(["Control effort (V)"])

    plt.show()


if __name__ == "__main__":
    main()
