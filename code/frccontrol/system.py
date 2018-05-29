"""A class that simplifies creating and updating state-space models as well as
designing controllers for them.
"""

import control as cnt
import frccontrol as frccnt
import matplotlib.pyplot as plt
import numpy as np
from . import dlqr
from . import kalman


class System():

    def __init__(self, sysc, u_min, u_max, dt):
        """Sets up the matrices for a state-space model.

        Keyword arguments:
        sysc -- StateSpace instance containing continuous state-space model
        u_min -- vector of minimum control inputs for system
        u_max -- vector of maximum control inputs for system
        dt -- time between model/controller updates
        """
        self.sysc = sysc
        self.sysd = sysc.sample(dt)  # Discretize model

        # Model matrices
        self.x = np.zeros((sysc.A.shape[0], 1))
        self.u = np.zeros((sysc.B.shape[1], 1))
        self.y = np.zeros((sysc.C.shape[0], 1))
        self.u_min = u_min
        self.u_max = u_max

        # Controller matrices
        self.r = np.zeros((sysc.A.shape[0], 1))
        self.K = np.zeros((sysc.B.shape[1], sysc.B.shape[0]))
        self.Kff = np.zeros((sysc.A.shape[0], sysc.A.shape[1]))

        # Observer matrices
        self.x_hat = np.zeros((sysc.A.shape[0], 1))
        self.L = np.zeros((sysc.A.shape[0], sysc.C.shape[0]))

    def update(self):
        """Advance the model by one timestep."""
        self.u = np.clip(self.K * (self.r - self.x_hat), self.u_min, self.u_max)
        self.__update_observer()
        self.x = self.sysd.A * self.x + self.sysd.B * self.u
        self.y = self.sysd.C * self.x + self.sysd.D * self.u

    def predict_observer(self):
        """Runs the predict step of the observer update."""
        self.x_hat = self.A * self.x_hat + self.B * self.u

    def correct_observer(self):
        """Runs the correct step of the observer update."""
        self.x_hat += np.linalg.inv(self.sysd.A) * self.L * (
            self.y - self.sysd.C * self.x_hat - self.sysd.D * self.u)

    def design_dlqr_controller(self, Q_elems, R_elems):
        """Design a discrete-time LQR controller for the system.

        Keyword arguments:
        Q_elems -- a vector of the maximum allowed excursions of the states from
                   the reference.
        R_elems -- a vector of the maximum allowed excursions of the control
                   inputs from no actuation.
        """
        Q = self.__make_cost_matrix(Q_elems)
        R = self.__make_cost_matrix(R_elems)
        self.K = dlqr(self.sysd, Q, R)

    def place_controller_poles(self, poles):
        """Design a controller that places the closed-loop system poles at the
        given locations.

        Most users should just use design_dlqr_controller(). Only use this if
        you know what you're doing.

        Keyword arguments:
        poles -- a list of compex numbers which are the desired pole locations.
                 Complex conjugate poles must be in pairs.
        """
        self.K = cnt.place(self.sysd.A, self.sysd.B, poles)

    def design_kalman_filter(self, Q_elems, R_elems):
        """Design a discrete-time Kalman filter for the system.

        Keyword arguments:
        Q_elems -- a vector of the standard deviations of each state from how
                   the model behaves.
        R_elems -- a vector of the standard deviations of each output
                   measurement.
        """
        Q = self.__make_cov_matrix(Q_elems)
        R = self.__make_cov_matrix(R_elems)
        kalman_gain, P_steady = kalman(self.sysd, Q=Q, R=R)
        self.L = self.sysd.A * kalman_gain

    def place_observer_poles(self, poles):
        """Design a controller that places the closed-loop system poles at the
        given locations.

        Most users should just use design_kalman_filter(). Only use this if you
        know what you're doing.

        Keyword arguments:
        poles -- a list of compex numbers which are the desired pole locations.
                 Complex conjugate poles must be in pairs.
        """
        self.L = cnt.place(self.sysd.A.T, self.sysd.C.T, poles).T

    def design_two_state_feedforward(self, Q_elems, R_elems):
        """Computes the feedforward constant for a two-state controller.

        This will take the form u = K_ff * (r_{n+1} - A r_n), where K_ff is the
        feed-forwards constant. It is important that Kff is *only* computed off
        the goal and not the feedback terms.

        Keyword arguments:
        Q_elems -- a vector of the maximum allowed excursions in the state
                   tracking.
        R_elems -- a vector of the maximum allowed excursions of the control
                   inputs from no actuation.
        """
        # We want to find the optimal U such that we minimize the tracking cost.
        # This means that we want to minimize
        #   (B u - (r_{n+1} - A r_n))^T Q (B u - (r_{n+1} - A r_n)) + u^T R u
        Q = self.__make_cost_matrix(Q_elems)
        R = self.__make_cost_matrix(R_elems)
        self.Kff = np.linalg.inv(self.sysd.B.T * Q.T * self.sysd.B +
                                 R.T) * self.sysd.B.T * Q.T

    def plot_pzmaps(self, figure_num, show=True):
        """Plots pole-zero maps of open-loop system, closed-loop system, and
        observer poles.

        Keyword arguments:
        figure_num -- number of figure on which to draw pole-zero maps.
        show -- whether to show plot immediately after drawing it.
        """
        plt.figure(figure_num)

        # Plot pole-zero map of open-loop system
        plt.subplot(2, 2, 1)
        frccnt.dpzmap(self.sysd, title="Open-loop system")

        # Plot pole-zero map of closed-loop system
        plt.subplot(2, 2, 2)
        frccnt.dpzmap(frccnt.closed_loop_ctrl(self), title="Closed-loop system")

        # Plot observer poles
        plt.subplot(2, 2, 3)
        frccnt.dpzmap(frccnt.closed_loop_obsv(self), title="Observer poles")

        if show:
            plt.show()

    def plot_responses(self, figure_num, t, refs, show=True):
        """Plots time-domain responses of the system and the control inputs.

        Keyword arguments:
        figure_num -- number of figure on which to draw pole-zero maps.
        time -- list of timesteps corresponding to references.
        refs -- list of tuples of time-reference pairs.
        show -- whether to show plot immediately after drawing it.
        """
        state_rec = np.zeros((self.sysd.states, 0))
        ref_rec = np.zeros((self.sysd.states, 0))
        u_rec = np.zeros((self.sysd.inputs, 0))

        # Run simulation
        for i in range(len(refs)):
            self.r = refs[i]
            self.update()

            # Log states for plotting
            state_rec = np.concatenate((state_rec, self.x), axis=1)
            ref_rec = np.concatenate((ref_rec, self.r), axis=1)
            u_rec = np.concatenate((u_rec, self.u), axis=1)

        plt.figure(figure_num)
        subplot_max = self.sysd.states + self.sysd.inputs
        for i in range(0, self.sysd.states):
            plt.subplot(subplot_max, 1, i + 1)
            if i == 0:
                plt.title("Time-domain responses")
            plt.plot(
                t,
                np.squeeze(np.asarray(state_rec[i, :])),
                label=self.state_labels[i])
            plt.legend()
            plt.plot(
                t,
                np.squeeze(np.asarray(ref_rec[i, :])),
                label=self.ref_labels[i])
            plt.legend()

        for i in range(0, self.sysd.inputs):
            plt.subplot(subplot_max, 1, self.sysd.states + i + 1)
            plt.plot(
                t, np.squeeze(np.asarray(u_rec[i, :])), label=self.u_labels[i])
            plt.xlabel("Time (s)")
            plt.legend()

        if show:
            plt.show()

    def set_plot_labels(self, state_labels, u_labels):
        """Sets label data for time-domain response plots.

        Keyword arguments:
        state_labels -- list of tuples containing name of state and the unit.
        u_labels -- list of tuples containing name of input and the unit.
        """
        self.state_labels = [x[0] + " (" + x[1] + ")" for x in state_labels]
        self.ref_labels = [x[0] + " ref (" + x[1] + ")" for x in state_labels]
        self.u_labels = [x[0] + " (" + x[1] + ")" for x in u_labels]

    def __update_observer(self):
        """Updates the observer given the current value of u."""
        self.x_hat = self.sysd.A * self.x_hat + self.sysd.B * self.u + self.L * (
            self.y - self.sysd.C * self.x_hat - self.sysd.D * self.u)

    def __make_cost_matrix(self, elems):
        """Creates a cost matrix from the given vector for use with LQR.

        The cost matrix is constructed using Bryson's rule. The inverse square
        of each element in the input is taken and placed on the cost matrix
        diagonal.

        Keyword arguments:
        elems -- a vector. For a Q matrix, its elements are the maximum allowed
                 excursions of the states from the reference. For an R matrix,
                 its elements are the maximum allowed excursions of the control
                 inputs from no actuation.

        Returns:
        State excursion or control effort cost matrix
        """
        return np.diag(1.0 / np.square(elems))

    def __make_cov_matrix(self, elems):
        """Creates a covariance matrix from the given vector for use with Kalman
        filters.

        Each element is squared and placed on the covariance matrix diagonal.

        Keyword arguments:
        elems -- a vector. For a Q matrix, its elements are the standard
                 deviations of each state from how the model behaves. For an R
                 matrix, its elements are the standard deviations for each
                 output measurement.

        Returns:
        Process noise or measurement noise covariance matrix
        """
        return np.diag(np.square(elems))
