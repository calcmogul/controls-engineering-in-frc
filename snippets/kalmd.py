"""Function for computing the steady-state Kalman gain matrix."""

import numpy as np
import scipy as sp


def kalmd(A, C, Q, R):
    """
    Solves for the discrete steady-state Kalman gain.

    Args:
        A: System matrix, states x states.
        C: Output matrix, outputs x states.
        Q: Process noise covariance matrix, states x states.
        R: Measurement noise covariance matrix, inputs x inputs.

    Returns:
        Kalman gain matrix, outputs x states.
    """
    P = sp.linalg.solve_discrete_are(a=A.T, b=C.T, q=Q, r=R)
    return np.linalg.solve(C @ P @ C.T + R, C @ P).T
