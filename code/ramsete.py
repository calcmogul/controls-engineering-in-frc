#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
    import latexutils

import control as cnt
import frccontrol as frccnt
import math
import matplotlib.pyplot as plt
import numpy as np


def drivetrain(motor, num_motors, m, r, rb, J, Gl, Gr):
    """Returns the state-space model for a drivetrain.

    States: [[left velocity], [right velocity]]
    Inputs: [[left voltage], [right voltage]]
    Outputs: [[left velocity], [right velocity], [angular velocity]]

    Keyword arguments:
    motor -- instance of DcBrushedMotor
    num_motors -- number of motors driving the mechanism
    m -- mass of robot in kg
    r -- radius of wheels in meters
    rb -- radius of robot in meters
    J -- moment of inertia of the drivetrain in kg-m^2
    Gl -- gear ratio of left side of drivetrain
    Gr -- gear ratio of right side of drivetrain

    Returns:
    StateSpace instance containing continuous model
    """
    motor = frccnt.models.gearbox(motor, num_motors)

    C1 = -Gl ** 2 * motor.Kt / (motor.Kv * motor.R * r ** 2)
    C2 = Gl * motor.Kt / (motor.R * r)
    C3 = -Gr ** 2 * motor.Kt / (motor.Kv * motor.R * r ** 2)
    C4 = Gr * motor.Kt / (motor.R * r)
    # fmt: off
    A = np.matrix([[(1 / m + rb**2 / J) * C1, (1 / m - rb**2 / J) * C3],
                   [(1 / m - rb**2 / J) * C1, (1 / m + rb**2 / J) * C3]])
    B = np.matrix([[(1 / m + rb**2 / J) * C2, (1 / m - rb**2 / J) * C4],
                   [(1 / m - rb**2 / J) * C2, (1 / m + rb**2 / J) * C4]])
    C = np.matrix([[1, 0],
                   [0, 1]])
    D = np.matrix([[0, 0],
                   [0, 0]])
    # fmt: on

    return cnt.ss(A, B, C, D)


def ramsete(
    b, zeta, x_desired, y_desired, theta_desired, v_desired, omega_desired, x, y, theta
):
    e_x = x_desired - x
    e_y = y_desired - y
    e_theta = theta_desired - theta
    k = 2 * zeta * math.sqrt(omega_desired ** 2 + b * v_desired ** 2)
    if abs(e_theta) < 0.001:
        sin_x_over_x = 1.0
    else:
        sin_x_over_x = math.sin(e_theta) / e_theta

    v = v_desired * math.cos(e_theta) + k * (
        math.cos(theta) * e_x + math.sin(theta) * e_y
    )
    omega = (
        omega_desired
        + b * v_desired * sin_x_over_x * (e_y * math.cos(theta) - math.sin(theta) * e_x)
        + k * e_theta
    )
    return v, omega


def get_diff_vels(v, omega, d):
    """Returns left and right wheel velocities given a central velocity and
    turning rate.

    Keyword arguments:
    v -- center velocity
    omega -- center turning rate
    d -- trackwidth
    """
    return v - omega * d / 2.0, v + omega * d / 2.0


class Drivetrain(frccnt.System):
    def __init__(self, dt):
        """Drivetrain subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        """
        state_labels = [
            ("Left velocity", "m/s"),
            ("Right velocity", "m/s"),
            ("Angular velocity", "rad/s"),
        ]
        u_labels = [("Left voltage", "V"), ("Right voltage", "V")]
        self.set_plot_labels(state_labels, u_labels)

        self.in_low_gear = False

        # Number of motors per side
        self.num_motors = 2.0

        # High and low gear ratios of drivetrain
        Glow = 60.0 / 11.0
        Ghigh = 60.0 / 11.0

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

        self.model = drivetrain(
            frccnt.models.MOTOR_CIM,
            self.num_motors,
            self.m,
            self.r,
            self.rb,
            self.J,
            self.Gl,
            self.Gr,
        )
        u_min = np.matrix([[-12.0], [-12.0]])
        u_max = np.matrix([[12.0], [12.0]])
        frccnt.System.__init__(self, self.model, u_min, u_max, dt)

        if self.in_low_gear:
            q_vel = 1.0
        else:
            q_vel = 0.95

        q = [q_vel, q_vel]
        r = [12.0, 12.0]
        self.design_dlqr_controller(q, r)

        qff_vel = 0.01
        self.design_two_state_feedforward([qff_vel, qff_vel], [12, 12])

        q_vel = 1.0
        r_vel = 0.01
        self.design_kalman_filter([q_vel, q_vel], [r_vel, r_vel])


def main():
    dt = 0.00505
    drivetrain = Drivetrain(dt)

    t, xprof, vprof, aprof = frccnt.generate_s_curve_profile(
        max_v=4.0, max_a=3.5, time_to_max_a=1.0, dt=dt, goal=10.0
    )

    # Initial robot pose
    x = 0
    y = 0
    theta = np.pi / 4.0

    # Generate references for LQR
    refs = []
    for i in range(len(t)):
        r = np.matrix([[vprof[i]], [vprof[i]]])
        refs.append(r)

    # Run LQR
    state_rec, ref_rec, u_rec = drivetrain.generate_time_responses(t, refs)
    plt.figure(1)
    drivetrain.plot_time_responses(t, state_rec, ref_rec, u_rec)
    if "--noninteractive" in sys.argv:
        latexutils.savefig("ramsete_vel_lqr_profile")

    # Initial robot pose
    x = 2
    y = 0
    theta = np.pi / 2.0

    # Ramsete tuning constants
    b = 2
    zeta = 0.7

    vl = float("inf")
    vr = float("inf")

    x_rec = []
    y_rec = []
    vref_rec = []
    omegaref_rec = []
    v_rec = []
    omega_rec = []
    ul_rec = []
    ur_rec = []

    # Run Ramsete
    drivetrain.reset()
    i = 0
    while i != len(t) - 1 or vl != 0 or vr != 0:
        # b, zeta, x_desired, y_desired, theta_desired, v_desired,
        # omega_desired, x, y, theta
        vref, omegaref = ramsete(
            b, zeta, 0, xprof[i], np.pi / 2, vprof[i], 0, x, y, theta
        )
        vl, vr = get_diff_vels(vref, omegaref, drivetrain.rb * 2.0)
        drivetrain.r = np.matrix([[vl], [vr]])
        drivetrain.update()
        vc = (drivetrain.x[0, 0] + drivetrain.x[1, 0]) / 2.0
        omega = (drivetrain.x[1, 0] - drivetrain.x[0, 0]) / (2.0 * drivetrain.rb)

        # Log data for plots
        vref_rec.append(vref)
        omegaref_rec.append(omegaref)
        x_rec.append(x)
        y_rec.append(y)
        ul_rec.append(drivetrain.u[0, 0])
        ur_rec.append(drivetrain.u[1, 0])
        v_rec.append(vc)
        omega_rec.append(omega)

        # Update nonlinear observer
        x += vc * math.cos(theta) * dt
        y += vc * math.sin(theta) * dt
        theta += omega * dt

        if i < len(t) - 1:
            i += 1

    plt.figure(2)
    plt.plot([0] * len(t), xprof, label="Reference trajectory")
    plt.plot(x_rec, y_rec, label="Ramsete controller")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.legend()

    # Equalize aspect ratio
    ylim = plt.ylim()
    width = abs(ylim[0]) + abs(ylim[1])
    plt.xlim([-width / 2, width / 2])

    if "--noninteractive" in sys.argv:
        latexutils.savefig("ramsete_response")

    plt.figure(3)
    num_plots = 4
    plt.subplot(num_plots, 1, 1)
    plt.title("Time-domain responses")
    plt.ylabel("Velocity (m/s)")
    plt.plot(t, vref_rec, label="Reference")
    plt.plot(t, v_rec, label="Estimated state")
    plt.legend()
    plt.subplot(num_plots, 1, 2)
    plt.ylabel("Angular rate (rad/s)")
    plt.plot(t, omegaref_rec, label="Reference")
    plt.plot(t, omega_rec, label="Estimated state")
    plt.legend()
    plt.subplot(num_plots, 1, 3)
    plt.ylabel("Left voltage (V)")
    plt.plot(t, ul_rec, label="Control effort")
    plt.legend()
    plt.subplot(num_plots, 1, 4)
    plt.ylabel("Right voltage (V)")
    plt.plot(t, ur_rec, label="Control effort")
    plt.legend()
    plt.xlabel("Time (s)")

    if "--noninteractive" in sys.argv:
        latexutils.savefig("ramsete_vel_lqr_response")
    else:
        plt.show()


if __name__ == "__main__":
    main()
