"""A class that simplifies creating and updating state-space models as well as
designing controllers for them.
"""

import control as cnt
import numpy as np
from . import dlqr


class System():

    def __init__(self, sysc, U_min, U_max, dt):
        """Sets up the matrices for a state-space model.

        Keyword arguments:
        sysc -- StateSpace instance containing continuous state-space model
        U_min -- vector of minimum control inputs for system
        U_max -- vector of maximum control inputs for system
        dt -- time between model/controller updates
        """
        self.sysc = sysc
        self.sysd = sysc.sample(dt)  # Discretize model

        self.x = np.zeros((sysc.A.shape[0], 1))
        self.u = np.zeros((sysc.B.shape[1], 1))
        self.r = np.zeros((sysc.A.shape[0], 1))
        self.U_min = U_min
        self.U_max = U_max
        self.K = np.zeros((sysc.B.shape[1], sysc.B.shape[0]))

    def update(self):
        """Advance the model by one timestep."""
        self.u = np.clip(self.K * (self.r - self.x), self.U_min, self.U_max)
        self.x = self.sysd.A * self.x + self.sysd.B * self.u
        self.y = self.sysd.C * self.x + self.sysd.D * self.u

    def make_lqr_cost_matrix(self, elems):
        """Creates a cost matrix from the given vector for use with LQR.

        The inverse square of each element in the input is taken and placed on
        the cost matrix diagonal.

        Keyword arguments:
        elems -- A vector. For a Q matrix, its elements contain the maximum
                 allowed excursions of the state variables. For an R matrix, its
                 elements contain the maximum allowed excursions for each
                 control input.

        Returns:
        Cost matrix.
        """
        return np.diag(1.0 / np.square(elems))

    def design_dlqr_controller(self, Q, R):
        """Design a discrete-time LQR controller for the system.

        Keyword arguments:
        Q -- The state excursion cost matrix.
        R -- The control effort cost matrix.
        """
        self.K = dlqr(self.sysd, Q, R)
