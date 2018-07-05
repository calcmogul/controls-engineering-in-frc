import control as cnt
import numpy as np


class DcBrushedMotor:

    def __init__(self, nominal_voltage, stall_torque, stall_current,
                 free_current, free_speed):
        """Holds the constants for a DC brushed motor.

        Keyword arguments:
        nominal_voltage -- voltage at which the motor constants were measured
        stall_torque -- current draw when stalled in Newton-meters
        stall_current  -- current draw when stalled in Amps
        free_current -- current draw under no load in Amps
        free_speed -- angular velocity under no load in RPM
        """
        self.nominal_voltage = nominal_voltage
        self.stall_torque = stall_torque
        self.stall_current = stall_current
        self.free_current = free_current

        # Convert from RPM to rad/s
        self.free_speed = free_speed / 60 * (2.0 * np.pi)

        # Resistance of motor
        self.R = self.nominal_voltage / self.stall_current

        # Motor velocity constant
        self.Kv = self.free_speed / (
            self.nominal_voltage - self.R * self.free_current)

        # Torque constant
        self.Kt = self.stall_torque / self.stall_current


# CIM
MOTOR_CIM = DcBrushedMotor(12.0, 2.42, 133.0, 2.7, 5310.0)

# MiniCIM
MOTOR_MINI_CIM = DcBrushedMotor(12.0, 1.41, 89.0, 3.0, 5840.0)

# Bag motor
MOTOR_BAG = DcBrushedMotor(12.0, 0.43, 53.0, 1.8, 13180.0)

# 775 Pro
MOTOR_775PRO = DcBrushedMotor(12.0, 0.71, 134.0, 0.7, 18730.0)

# Andymark RS 775-125
MOTOR_AM_RS775_125 = DcBrushedMotor(12.0, 0.28, 18.0, 1.6, 5800.0)

# Banebots RS 775
MOTOR_BB_RS775 = DcBrushedMotor(12.0, 0.72, 97.0, 2.7, 13050.0)

# Andymark 9015
MOTOR_AM_9015 = DcBrushedMotor(12.0, 0.36, 71.0, 3.7, 14270.0)

# Banebots RS 550
MOTOR_BB_RS550 = DcBrushedMotor(12.0, 0.38, 84.0, 0.4, 19000.0)


def gearbox(motor, num_motors):
    """Returns a DcBrushedMotor with the same characteristics as the specified
    number of motors in a gearbox.
    """
    return DcBrushedMotor(motor.nominal_voltage,
                          motor.stall_torque * num_motors, motor.stall_current,
                          motor.free_current,
                          motor.free_speed / (2.0 * np.pi) * 60)


def elevator(motor, num_motors, m, r, G):
    """Returns the state-space model for an elevator.

    States: [[position], [velocity]]
    Inputs: [[voltage]]
    Outputs: [[position]]

    Keyword arguments:
    motor -- instance of DcBrushedMotor
    num_motors -- number of motors driving the mechanism
    m -- carriage mass in kg
    r -- pulley radius in meters
    G -- gear ratio from motor to carriage

    Returns:
    StateSpace instance containing continuous model
    """
    motor = gearbox(motor, num_motors)

    # yapf: disable
    A = np.matrix([[0, 1],
                   [0, -G**2 * motor.Kt / (motor.R**2 * r * m * motor.Kv)]])
    B = np.matrix([[0],
                   [G * motor.Kt / (motor.R * r * m)]])
    C = np.matrix([[1, 0]])
    D = np.matrix([[0]])
    # yapf: enable

    return cnt.ss(A, B, C, D)


def flywheel(motor, num_motors, J, G):
    """Returns the state-space model for a flywheel.

    States: [[angular velocity]]
    Inputs: [[voltage]]
    Outputs: [[angular velocity]]

    Keyword arguments:
    motor -- instance of DcBrushedMotor
    num_motors -- number of motors driving the mechanism
    J -- flywheel moment of inertia in kg-m^2
    G -- gear ratio from motor to carriage

    Returns:
    StateSpace instance containing continuous model
    """
    motor = gearbox(motor, num_motors)

    A = np.matrix([[-G**2 * motor.Kt / (motor.Kv * motor.R * J)]])
    B = np.matrix([[G * motor.Kt / (motor.R * J)]])
    C = np.matrix([[1]])
    D = np.matrix([[0]])

    return cnt.ss(A, B, C, D)


def drivetrain(motor, num_motors, m, r, rb, J, Gl, Gr):
    """Returns the state-space model for a drivetrain.

    States: [[left position], [left velocity],
             [right position], [right velocity]]
    Inputs: [[left voltage], [right voltage]]
    Outputs: [[left position], [right position]]

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
    motor = gearbox(motor, num_motors)

    C1 = -Gl**2 * motor.Kt / (motor.Kv * motor.R * r**2)
    C2 = Gl * motor.Kt / (motor.R * r)
    C3 = -Gr**2 * motor.Kt / (motor.Kv * motor.R * r**2)
    C4 = Gr * motor.Kt / (motor.R * r)
    # yapf: disable
    A = np.matrix([[0, 1, 0, 0],
                   [0, (1 / m - rb**2 / J) * C1, 0, (1 / m + rb**2 / J) * C3],
                   [0, 0, 0, 1],
                   [0, (1 / m + rb**2 / J) * C1, 0, (1 / m - rb**2 / J) * C3]])
    B = np.matrix([[0, 0],
                   [(1 / m + rb**2 / J) * C2, (1 / m - rb**2 / J) * C4],
                   [0, 0],
                   [(1 / m - rb**2 / J) * C2, (1 / m + rb**2 / J) * C4]])
    C = np.matrix([[1, 0, 0, 0],
                   [0, 0, 1, 0]])
    D = np.matrix([[0, 0],
                   [0, 0]])
    # yapf: enable

    return cnt.ss(A, B, C, D)


def single_jointed_arm(motor, num_motors, J, G):
    """Returns the state-space model for a flywheel.

    States: [[angle, angular velocity]]
    Inputs: [[voltage]]
    Outputs: [[angular velocity]]

    Keyword arguments:
    motor -- instance of DcBrushedMotor
    num_motors -- number of motors driving the mechanism
    J -- arm moment of inertia in kg-m^2
    G -- gear ratio from motor to arm

    Returns:
    StateSpace instance containing continuous model
    """
    motor = gearbox(motor, num_motors)

    A = np.matrix([[0, 1], [0, -G**2 * motor.Kt / (motor.Kv * motor.R * J)]])
    B = np.matrix([[0], [G * motor.Kt / (motor.R * J)]])
    C = np.matrix([[1, 0]])
    D = np.matrix([[0]])

    return cnt.ss(A, B, C, D)
