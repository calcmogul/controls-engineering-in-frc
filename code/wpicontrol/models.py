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
        self.free_speed = free_speed / 60  # Convert from RPM to revs/s

        # Resistance of motor
        self.R = self.nominal_voltage / self.stall_current

        # Motor velocity constant
        self.Kv = self.free_speed * (2.0 * np.pi) / (
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


def elevator(motor, m, r, G):
    """Returns the state-space model for an elevator.

    States: [[position], [velocity]]
    Inputs: [[voltage]]
    Outputs: [[position]]

    Keyword arguments:
    motor -- instance of DcBrushedMotor
    m -- carriage mass in kg
    r -- pulley radius in meters
    G -- gear ratio from motor to carriage

    Returns:
    StateSpace instance containing continuous model
    """
    A = np.matrix([[0, 1],
                   [0, -G**2 * motor.Kt / (motor.R**2 * r * m * motor.Kv)]])
    B = np.matrix([[0], [G * motor.Kt / (motor.R * r * m)]])
    C = np.matrix([[1, 0]])
    D = np.matrix([[0]])

    return cnt.ss(A, B, C, D)
