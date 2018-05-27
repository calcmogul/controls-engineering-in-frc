#!/usr/bin/env python3

import frccontrol as frccnt
import matplotlib.pyplot as plt


class Drivetrain(frccnt.System):

    def __init__(self, dt):
        """Drivetrain subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        """
        self.in_low_gear = False

        # High and low gear ratios of drivetrain
        Glow = 11.0 / 60.0
        Ghigh = 11.0 / 60.0

        # Drivetrain mass in kg
        self.m = 52
        # Radius of wheels in meters
        self.r = 0.08255 / 2.0
        # Radius of robot in meters
        self.rb = 0.59055 / 2.0
        # Moment of inertia of the drivetrain in kg-m^2
        self.J = 6.0

        # Gear ratios of left and right sides of drivetrain respectively
        if self.in_low_gear:
            self.Gl = Glow
            self.Gr = Glow
        else:
            self.Gl = Ghigh
            self.Gr = Ghigh

        self.model = frccnt.models.drivetrain(frccnt.models.MOTOR_CIM, self.m,
                                              self.r, self.rb, self.J, self.Gl,
                                              self.Gr)
        frccnt.System.__init__(self, self.model, -12.0, 12.0, dt)

        if self.in_low_gear:
            q_pos = 0.12
            q_vel = 1.0
        else:
            q_pos = 0.14
            q_vel = 0.95

        self.design_dlqr_controller([q_pos, q_vel, q_pos, q_vel], [12.0, 12.0])

        qff_pos = 0.005
        qff_vel = 1.0
        self.design_two_state_feedforward([qff_pos, qff_vel, qff_pos, qff_vel],
                                          [12.0, 12.0])

        q_pos = 0.05
        q_vel = 1.0
        q_voltage = 10.0
        q_encoder_uncertainty = 2.0
        r_pos = 0.0001
        r_gyro = 0.000001
        self.design_kalman_filter([q_pos, q_vel, q_pos, q_vel], [r_pos, r_pos])


def frange(x, y, jump):
    while x < y:
        yield x
        x += jump


def main():
    dt = 0.00505
    drivetrain = Drivetrain(dt)

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
            drivetrain.r[0, 0] = 0.0
        elif t[i] < l1:
            drivetrain.r[0, 0] = 1.524
        else:
            drivetrain.r[0, 0] = 0.0
        drivetrain.update()

        # Log states for plotting
        pos.append(drivetrain.x[0, 0])
        r_pos.append(drivetrain.r[0, 0])
        vel.append(drivetrain.x[1, 0])
        u.append(drivetrain.u[0, 0])

    plt.figure(1)

    # Plot pole-zero map of open-loop system
    plt.subplot(2, 2, 1)
    frccnt.dpzmap(drivetrain.sysd, title="Open-loop system")

    # Plot pole-zero map of closed-loop system
    plt.subplot(2, 2, 2)
    frccnt.dpzmap(
        frccnt.closed_loop_ctrl(drivetrain), title="Closed-loop system")

    # Plot observer poles
    plt.subplot(2, 2, 3)
    frccnt.dpzmap(frccnt.closed_loop_obsv(drivetrain), title="Observer poles")

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
