#!/usr/bin/env python3

# Runs linearized unicycle simulation

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
    import utils.latex as latex

import control as ct
import frccontrol as fct
import math
import matplotlib.pyplot as plt
import numpy as np


def unicycle(states, inputs):
    """Returns the state-space model for a unicycle.

    States: [[x], [y], [theta]]
    Inputs: [[velocity], [angular velocity]]
    Outputs: [[theta]]

    Keyword arguments:
    states -- state vector around which to linearize model
    inputs -- input vector around which to linearize model

    Returns:
    StateSpace instance containing continuous model
    """
    x = states[0, 0]
    y = states[1, 0]
    theta = states[2, 0]
    v = inputs[0, 0]
    omega = inputs[1, 0]
    if abs(v) < 5e-8:
        v = 5e-8
    # fmt: off
    A = np.array([[0, 0, -v * math.sin(theta)],
                  [0, 0, v * math.cos(theta)],
                  [0, 0, 0]])
    B = np.array([[math.cos(theta), 0],
                  [math.sin(theta), 0],
                  [0, 1]])
    C = np.array([[0, 0, 1]])
    D = np.array([[0, 0]])
    # fmt: on

    return ct.StateSpace(A, B, C, D, remove_useless=False)


class Unicycle(fct.System):
    def __init__(self, dt, states, inputs):
        """Unicycle subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        states -- state vector around which to linearize model
        inputs -- input vector around which to linearize model
        """
        state_labels = [("x position", "m"), ("y position", "m"), ("Heading", "rad")]
        u_labels = [("Velocity", "m/s"), ("Angular velocity", "rad/s")]
        self.set_plot_labels(state_labels, u_labels)

        u_min = np.array([[-2.5], [-2.5]])
        u_max = np.array([[2.5], [2.5]])

        f = lambda x, u: np.array(
            [[u[0, 0] * math.cos(x[2, 0])], [u[0, 0] * math.sin(x[2, 0])], [u[1, 0]]]
        )
        fct.System.__init__(self, u_min, u_max, dt, states, inputs, nonlinear_func=f)

    def create_model(self, states, inputs):
        """Relinearize model around given state.

        Keyword arguments:
        states -- state vector around which to linearize model
        inputs -- input vector around which to linearize model

        Returns:
        StateSpace instance containing continuous state-space model
        """
        return unicycle(np.asarray(states), np.asarray(inputs))

    def design_controller_observer(self):
        q_x = 0.0625
        q_y = 0.125
        q_heading = 10.0

        q = [q_x, q_y, q_heading]
        r = [2.5, 2.5]
        self.design_lqr(q, r)

        self.design_two_state_feedforward()

        q_pos = 0.5
        r_gyro = 0.0001
        self.design_kalman_filter([q_pos, q_pos, q_heading], [r_gyro])


def main():
    dt = 0.02

    vs = np.arange(-1.1, 1.1, 0.01)
    K_rec = np.zeros((2, 3, len(vs)))
    for i, v in enumerate(vs):
        x_linear = np.array([[0], [0], [0]])
        u_linear = np.array([[v], [v]])
        K_rec[:, :, i] = Unicycle(dt, x_linear, u_linear).K

    state_labels = ["$x$", "$y$", "$\\theta$"]
    input_labels = ["Velocity", "Angular velocity"]

    for i in range(len(state_labels)):
        plt.figure(i + 1)
        plt.plot(vs, K_rec[0, i, :], label=f"{input_labels[0]}")
        plt.plot(vs, K_rec[1, i, :], label=f"{input_labels[1]}")
        plt.xlabel("v (m/s)")
        plt.ylabel(f"Gain {state_labels[i]} error to input")
        plt.legend()

        if "--noninteractive" in sys.argv:
            latex.savefig(f"linearized_unicycle_cascaded_lqr_{i}")
    if "--noninteractive" not in sys.argv:
        plt.show()


if __name__ == "__main__":
    main()
