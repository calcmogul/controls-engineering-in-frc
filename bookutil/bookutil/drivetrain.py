import control as ct
import frccontrol as fct
import math
import numpy as np


def get_diff_vels(v, omega, d):
    """Returns left and right wheel velocities given a central velocity and
    turning rate.

    Keyword arguments:
    v -- center velocity
    omega -- center turning rate
    d -- trackwidth
    """
    return v - omega * d / 2.0, v + omega * d / 2.0


def drivetrain_decoupled(motor, num_motors, m, r, rb, J, Gl, Gr):
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
    motor = fct.models.gearbox(motor, num_motors)

    C1 = -(Gl ** 2) * motor.Kt / (motor.Kv * motor.R * r ** 2)
    C2 = Gl * motor.Kt / (motor.R * r)
    C3 = -(Gr ** 2) * motor.Kt / (motor.Kv * motor.R * r ** 2)
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

    return ct.ss(A, B, C, D)


def drivetrain_coupled(motor, num_motors, m, r, rb, J, Gl, Gr):
    """Returns the state-space model for a drivetrain.

    States: [[velocity], [angular velocity]]
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
    motor = fct.models.gearbox(motor, num_motors)

    C1 = -(Gl ** 2) * motor.Kt / (motor.Kv * motor.R * r ** 2)
    C2 = Gl * motor.Kt / (motor.R * r)
    C3 = -(Gr ** 2) * motor.Kt / (motor.Kv * motor.R * r ** 2)
    C4 = Gr * motor.Kt / (motor.R * r)
    # fmt: off
    A = np.array([[1 / m * (C1 + C3), rb / m * (-C1 + C3)],
                  [rb / J * (C1 - C3), rb ** 2 / J * (-C1 - C3)]])
    B = np.array([[1 / m * C2, 1 / m * C4],
                  [rb / J * C2, -rb / J * C4]])
    C = np.array([[1, -rb],
                  [1, rb]])
    D = np.array([[0, 0],
                  [0, 0]])
    # fmt: on

    return ct.ss(A, B, C, D)


def differential_drive(motor, num_motors, m, r, rb, J, Gl, Gr, states):
    """Returns the state-space model for a differential drive.

    States: [[x], [y], [theta], [left velocity], [right velocity]]
    Inputs: [[left voltage], [right voltage]]
    Outputs: [[theta], [left velocity], [right velocity]]

    Keyword arguments:
    motor -- instance of DcBrushedMotor
    num_motors -- number of motors driving the mechanism
    m -- mass of robot in kg
    r -- radius of wheels in meters
    rb -- radius of robot in meters
    J -- moment of inertia of the differential drive in kg-m^2
    Gl -- gear ratio of left side of differential drive
    Gr -- gear ratio of right side of differential drive
    states -- state vector around which to linearize model

    Returns:
    StateSpace instance containing continuous model
    """
    motor = fct.models.gearbox(motor, num_motors)

    C1 = -(Gl ** 2) * motor.Kt / (motor.Kv * motor.R * r ** 2)
    C2 = Gl * motor.Kt / (motor.R * r)
    C3 = -(Gr ** 2) * motor.Kt / (motor.Kv * motor.R * r ** 2)
    C4 = Gr * motor.Kt / (motor.R * r)
    x = states[0, 0]
    y = states[1, 0]
    theta = states[2, 0]
    vl = states[3, 0]
    vr = states[4, 0]
    v = (vr + vl) / 2.0
    if abs(v) < 1e-9:
        vl = 1e-9
        vr = 1e-9
        v = 1e-9
    # fmt: off
    c = math.cos(theta)
    s = math.sin(theta)
    A = np.array([[0, 0, -v * s, 0.5 * c, 0.5 * c],
                  [0, 0, v * c, 0.5 * s, 0.5 * s],
                  [0, 0, 0, -0.5 / rb, 0.5 / rb],
                  [0, 0, 0, (1 / m + rb**2 / J) * C1, (1 / m - rb**2 / J) * C3],
                  [0, 0, 0, (1 / m - rb**2 / J) * C1, (1 / m + rb**2 / J) * C3]])
    B = np.array([[0, 0],
                  [0, 0],
                  [0, 0],
                  [(1 / m + rb**2 / J) * C2, (1 / m - rb**2 / J) * C4],
                  [(1 / m - rb**2 / J) * C2, (1 / m + rb**2 / J) * C4]])
    C = np.array([[0, 0, 1, 0, 0],
                  [0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 1]])
    D = np.array([[0, 0],
                  [0, 0],
                  [0, 0]])
    # fmt: on

    return ct.StateSpace(A, B, C, D, remove_useless=False)


def ramsete(pose_desired, v_desired, omega_desired, pose, b, zeta):
    """Ramsete is a nonlinear time-varying feedback controller for unicycle
    models that drives the model to a desired pose along a two-dimensional
    trajectory.

    The reference pose, linear velocity, and angular velocity should come from
    a drivetrain trajectory.

    Keyword arguments:
    pose_desired -- the desired pose
    v_desired -- the desired linear velocity
    omega_desired -- the desired angular velocity
    pose -- the current pose
    b -- tuning parameter (b > 0) for which larger values make convergence more
         aggressive like a proportional term
    zeta -- tuning parameter (0 < zeta < 1) for which larger values provide more
            damping in response

    Returns:
    linear velocity and angular velocity commands
    """
    e = pose_desired.relative_to(pose)

    k = 2 * zeta * math.sqrt(omega_desired ** 2 + b * v_desired ** 2)
    v = v_desired * math.cos(e.theta) + k * e.x
    omega = omega_desired + k * e.theta + b * v_desired * np.sinc(e.theta) * e.y

    return v, omega
