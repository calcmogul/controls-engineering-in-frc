#!/usr/bin/env python3

# Runs Ramsete simulation on decoupled model and compares odometry methods

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
    Outputs: [[left velocity], [right velocity]]

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
    A = np.array([[(1 / m + rb**2 / J) * C1, (1 / m - rb**2 / J) * C3],
                  [(1 / m - rb**2 / J) * C1, (1 / m + rb**2 / J) * C3]])
    B = np.array([[(1 / m + rb**2 / J) * C2, (1 / m - rb**2 / J) * C4],
                  [(1 / m - rb**2 / J) * C2, (1 / m + rb**2 / J) * C4]])
    C = np.array([[1, 0],
                  [0, 1]])
    D = np.array([[0, 0],
                  [0, 0]])
    # fmt: on

    return cnt.ss(A, B, C, D)


class Pose2d:
    def __init__(self, x=0, y=0, theta=0):
        self.x = x
        self.y = y
        self.theta = theta

    def __add__(self, other):
        return Pose2d(self.x + other.x, self.y + other.y, self.theta + other.theta)

    def __sub__(self, other):
        return Pose2d(self.x - other.x, self.y - other.y, self.theta - other.theta)

    def rotate(self, theta):
        """Rotate the pose counterclockwise by the given angle.

        Keyword arguments:
        theta -- Angle in radians
        """
        x = math.cos(theta) * self.x - math.sin(theta) * self.y
        y = math.sin(theta) * self.x + math.cos(theta) * self.y
        self.x = x
        self.y = y

    def apply(self, twist, dt):
        """Apply the given twist to update the pose.

        Keyword arguments:
        twist -- a Twist2d object containing the linear and angular velocities
                 between updates
        dt -- the time in seconds between updates
        """
        # Compute change in pose in local coordinate frame
        if twist.omega > 1e-9:
            s = math.sin(twist.omega * dt) / twist.omega
            c = (math.cos(twist.omega * dt) - 1.0) / twist.omega
        else:
            s = dt - dt ** 3 * twist.omega ** 2 / 6.0
            c = -dt ** 2 * twist.omega / 2.0
        dpose_r = Pose2d(
            twist.v_x * s + twist.v_y * c,
            twist.v_x * -c + twist.v_y * s,
            twist.omega * dt,
        )

        # Transform to global coordinate frame, then apply transformation
        self.x += dpose_r.x * math.cos(self.theta) - dpose_r.y * math.sin(self.theta)
        self.y += dpose_r.x * math.sin(self.theta) + dpose_r.y * math.cos(self.theta)
        self.theta += dpose_r.theta


class Twist2d:
    def __init__(self, v_x=0, v_y=0, omega=0):
        self.v_x = v_x
        self.v_y = v_y
        self.omega = omega


def ramsete(pose_desired, v_desired, omega_desired, pose, b, zeta):
    e = pose_desired - pose
    e.rotate(-pose.theta)

    k = 2 * zeta * math.sqrt(omega_desired ** 2 + b * v_desired ** 2)
    v = v_desired * math.cos(e.theta) + k * e.x
    omega = omega_desired + k * e.theta + b * v_desired * np.sinc(e.theta) * e.y

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
        state_labels = [("Left velocity", "m/s"), ("Right velocity", "m/s")]
        u_labels = [("Left voltage", "V"), ("Right voltage", "V")]
        self.set_plot_labels(state_labels, u_labels)

        u_min = np.array([[-12.0], [-12.0]])
        u_max = np.array([[12.0], [12.0]])
        frccnt.System.__init__(self, np.zeros((2, 1)), u_min, u_max, dt)

    def create_model(self, states):
        self.in_low_gear = False

        # Number of motors per side
        num_motors = 2.0

        # High and low gear ratios of drivetrain
        Glow = 60.0 / 11.0
        Ghigh = 60.0 / 11.0

        # Drivetrain mass in kg
        m = 52
        # Radius of wheels in meters
        r = 0.08255 / 2.0
        # Radius of robot in meters
        self.rb = 0.59055 / 2.0
        # Moment of inertia of the drivetrain in kg-m^2
        J = 6.0

        # Gear ratios of left and right sides of drivetrain respectively
        if self.in_low_gear:
            Gl = Glow
            Gr = Glow
        else:
            Gl = Ghigh
            Gr = Ghigh

        return drivetrain(frccnt.models.MOTOR_CIM, num_motors, m, r, self.rb, J, Gl, Gr)

    def design_controller_observer(self):
        if self.in_low_gear:
            q_vel = 1.0
        else:
            q_vel = 0.95

        q = [q_vel, q_vel]
        r = [12.0, 12.0]
        self.design_lqr(q, r)

        qff_vel = 0.01
        self.design_two_state_feedforward([qff_vel, qff_vel], [12, 12])

        q_vel = 1.0
        r_vel = 0.01
        self.design_kalman_filter([q_vel, q_vel], [r_vel, r_vel])

        print("ctrb cond =", np.linalg.cond(cnt.ctrb(self.sysd.A, self.sysd.B)))


def main():
    dt = 0.00505
    drivetrain = Drivetrain(dt)

    t, xprof, vprof, aprof = frccnt.generate_s_curve_profile(
        max_v=4.0, max_a=3.5, time_to_max_a=1.0, dt=dt, goal=10.0
    )

    # Initial robot pose
    pose = Pose2d(2, 0, np.pi / 2.0)
    desired_pose = Pose2d()
    twist_pose = Pose2d(2, 0, np.pi / 2.0)

    # Ramsete tuning constants
    b = 2
    zeta = 0.7

    vl = float("inf")
    vr = float("inf")

    x_rec = []
    y_rec = []
    twist_x_rec = []
    twist_y_rec = []
    vref_rec = []
    omegaref_rec = []
    v_rec = []
    omega_rec = []
    ul_rec = []
    ur_rec = []

    # Log initial data for plots
    vref_rec.append(0)
    omegaref_rec.append(0)
    x_rec.append(pose.x)
    y_rec.append(pose.y)
    twist_x_rec.append(twist_pose.x)
    twist_y_rec.append(twist_pose.y)
    ul_rec.append(drivetrain.u[0, 0])
    ur_rec.append(drivetrain.u[1, 0])
    v_rec.append(0)
    omega_rec.append(0)

    # Run Ramsete
    i = 0
    while i < len(t) - 1:
        desired_pose.x = 0
        desired_pose.y = xprof[i]
        desired_pose.theta = np.pi / 2.0

        # pose_desired, v_desired, omega_desired, pose, b, zeta
        vref, omegaref = ramsete(desired_pose, vprof[i], 0, pose, b, zeta)
        vl, vr = get_diff_vels(vref, omegaref, drivetrain.rb * 2.0)
        next_r = np.array([[vl], [vr]])
        drivetrain.update(next_r)
        vc = (drivetrain.x[0, 0] + drivetrain.x[1, 0]) / 2.0
        omega = (drivetrain.x[1, 0] - drivetrain.x[0, 0]) / (2.0 * drivetrain.rb)

        # Log data for plots
        vref_rec.append(vref)
        omegaref_rec.append(omegaref)
        x_rec.append(pose.x)
        y_rec.append(pose.y)
        twist_x_rec.append(twist_pose.x)
        twist_y_rec.append(twist_pose.y)
        ul_rec.append(drivetrain.u[0, 0])
        ur_rec.append(drivetrain.u[1, 0])
        v_rec.append(vc)
        omega_rec.append(omega)

        # Update nonlinear observer
        pose.x += vc * math.cos(pose.theta) * dt
        pose.y += vc * math.sin(pose.theta) * dt
        pose.theta += omega * dt
        twist_pose.apply(Twist2d(vc, 0, omega), dt)

        if i < len(t) - 1:
            i += 1

    plt.figure(1)
    plt.ylabel("Odometry error (m)")
    plt.plot(t, np.subtract(twist_x_rec, x_rec), label="Error in x")
    plt.plot(t, np.subtract(twist_y_rec, y_rec), label="Error in y")
    plt.legend()
    plt.xlabel("Time (s)")

    if "--noninteractive" in sys.argv:
        latexutils.savefig("ramsete_twist_odometry_error")
    else:
        plt.show()


if __name__ == "__main__":
    main()
