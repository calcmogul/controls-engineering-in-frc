import frccontrol as fct
import math
import numpy as np

from .drivetrain import drivetrain_coupled, drivetrain_decoupled, differential_drive


class Flywheel(fct.System):
    def __init__(self, dt):
        """Flywheel subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        """
        state_labels = [("Angular velocity", "rad/s")]
        u_labels = [("Voltage", "V")]
        self.set_plot_labels(state_labels, u_labels)

        fct.System.__init__(
            self,
            np.array([[-12.0]]),
            np.array([[12.0]]),
            dt,
            np.zeros((1, 1)),
            np.zeros((1, 1)),
        )

    def create_model(self, states, inputs):
        # Number of motors
        num_motors = 1.0
        # Flywheel moment of inertia in kg-m²
        J = 0.00032
        # Gear ratio
        G = 12.0 / 18.0

        return fct.models.flywheel(fct.models.MOTOR_775PRO, num_motors, J, G)

    def design_controller_observer(self):
        self.design_lqr([200.0], [12.0])

        q_vel = 1.0
        r_vel = 0.01
        self.design_kalman_filter([q_vel], [r_vel])


class Elevator(fct.System):
    def __init__(self, dt):
        """Elevator subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        """
        state_labels = [("Position", "m"), ("Velocity", "m/s")]
        u_labels = [("Voltage", "V")]
        self.set_plot_labels(state_labels, u_labels)

        fct.System.__init__(
            self,
            np.array([[-12.0]]),
            np.array([[12.0]]),
            dt,
            np.zeros((2, 1)),
            np.zeros((1, 1)),
        )

    def create_model(self, states, inputs):
        # Number of motors
        num_motors = 2.0
        # Elevator carriage mass in kg
        m = 6.803886
        # Radius of pulley in meters
        r = 0.02762679089
        # Gear ratio
        G = 42.0 / 12.0 * 40.0 / 14.0

        return fct.models.elevator(fct.models.MOTOR_CIM, num_motors, m, r, G)

    def design_controller_observer(self):
        self.design_lqr([0.02, 0.4], [12.0])
        self.design_two_state_feedforward()

        q_pos = 0.05
        q_vel = 1.0
        r_pos = 0.0001
        self.design_kalman_filter([q_pos, q_vel], [r_pos])


class DrivetrainDecoupledVelocity(fct.System):
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
        fct.System.__init__(self, u_min, u_max, dt, np.zeros((2, 1)), np.zeros((2, 1)))

    def create_model(self, states, inputs):
        # Number of motors per side
        num_motors = 2.0

        # Gear ratio of drivetrain
        G = 60.0 / 11.0

        # Drivetrain mass in kg
        m = 52
        # Radius of wheels in meters
        r = 0.08255 / 2.0
        # Radius of robot in meters
        self.rb = 0.59055 / 2.0
        # Moment of inertia of the drivetrain in kg-m²
        J = 6.0

        return drivetrain_decoupled(
            fct.models.MOTOR_CIM, num_motors, m, r, self.rb, J, G, G
        )

    def design_controller_observer(self):
        self.design_two_state_feedforward()
        self.design_lqr([0.95, 0.95], [12.0, 12.0])

        q_vel = 1.0
        r_vel = 0.01
        self.design_kalman_filter([q_vel, q_vel], [r_vel, r_vel])


class DrivetrainCoupledVelocity(fct.System):
    def __init__(self, dt):
        """Drivetrain subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        """
        state_labels = [("Velocity", "m/s"), ("Angular velocity", "rad/s")]
        u_labels = [("Left voltage", "V"), ("Right voltage", "V")]
        self.set_plot_labels(state_labels, u_labels)

        u_min = np.array([[-12.0], [-12.0]])
        u_max = np.array([[12.0], [12.0]])
        fct.System.__init__(self, u_min, u_max, dt, np.zeros((2, 1)), np.zeros((2, 1)))

    def create_model(self, states, inputs):
        # Number of motors per side
        num_motors = 2.0

        # Gear ratio of drivetrain
        G = 60.0 / 11.0

        # Drivetrain mass in kg
        m = 52
        # Radius of wheels in meters
        r = 0.08255 / 2.0
        # Radius of robot in meters
        self.rb = 0.59055 / 2.0
        # Moment of inertia of the drivetrain in kg-m²
        J = 6.0

        return drivetrain_coupled(
            fct.models.MOTOR_CIM, num_motors, m, r, self.rb, J, G, G
        )

    def design_controller_observer(self):
        self.design_two_state_feedforward()
        self.design_lqr([0.95, 1.0], [12.0, 12.0])

        q_vel = 1.0
        q_angular = 1.0
        r_vel = 0.01
        self.design_kalman_filter([q_vel, q_angular], [r_vel, r_vel])


class LTVDifferentialDrive(fct.System):
    def __init__(self, dt, states):
        """Differential drive subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        states -- state vector around which to linearize model
        """
        state_labels = [
            ("x position", "m"),
            ("y position", "m"),
            ("Heading", "rad"),
            ("Left velocity", "m/s"),
            ("Right velocity", "m/s"),
        ]
        u_labels = [("Left voltage", "V"), ("Right voltage", "V")]
        self.set_plot_labels(state_labels, u_labels)

        u_min = np.array([[-12.0], [-12.0]])
        u_max = np.array([[12.0], [12.0]])

        f = (
            lambda x, u: np.array(
                [
                    [(x[3, 0] + x[4, 0]) / 2.0 * math.cos(x[2, 0])],
                    [(x[3, 0] + x[4, 0]) / 2.0 * math.sin(x[2, 0])],
                    [(x[4, 0] - x[3, 0]) / (2.0 * self.rb)],
                    [self.sysc.A[3, 3] * x[3, 0] + self.sysc.A[3, 4] * x[4, 0]],
                    [self.sysc.A[4, 3] * x[3, 0] + self.sysc.A[4, 4] * x[4, 0]],
                ]
            )
            + self.sysc.B @ u
        )
        fct.System.__init__(
            self, u_min, u_max, dt, states, np.zeros((2, 1)), nonlinear_func=f
        )

    def create_model(self, states, inputs):
        """Relinearize model around given state.

        Keyword arguments:
        states -- state vector around which to linearize model
        inputs -- input vector around which to linearize model

        Returns:
        StateSpace instance containing continuous state-space model
        """
        # Number of motors per side
        num_motors = 3.0

        # Gear ratio of differential drive
        G = 60.0 / 11.0

        # Drivetrain mass in kg
        m = 52
        # Radius of wheels in meters
        r = 0.08255 / 2.0
        # Radius of robot in meters
        self.rb = 0.59055 / 2.0
        # Moment of inertia of the differential drive in kg-m²
        J = 6.0

        return differential_drive(
            fct.models.MOTOR_CIM,
            num_motors,
            m,
            r,
            self.rb,
            J,
            G,
            G,
            states,
        )

    def relinearize(self, Q_elems, R_elems, states, inputs):
        from frccontrol import lqr

        sysc = self.create_model(states, inputs)
        sysd = sysc.to_discrete(self.dt)

        Q = np.diag(1.0 / np.square(Q_elems))
        R = np.diag(1.0 / np.square(R_elems))
        return lqr(sysd, Q, R)

    def design_controller_observer(self):
        q_x = 0.05
        q_y = 0.125
        q_heading = 10.0
        q_vel = 0.95

        q = [q_x, q_y, q_heading, q_vel, q_vel]
        r = [12.0, 12.0]

        self.K = self.relinearize(q, r, self.x_hat, np.zeros((2, 1)))

        self.design_two_state_feedforward()

        q_pos = 0.5
        q_vel = 1.0
        r_gyro = 0.0001
        r_vel = 0.01
        self.design_kalman_filter(
            [q_pos, q_pos, q_heading, q_vel, q_vel], [r_gyro, r_vel, r_vel]
        )
