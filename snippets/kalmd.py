"""Function for computing the steady-state Kalman gain matrix."""

import numpy as np
import scipy as sp


def kalmd(A, C, Q, R):
    """
    Solves for the discrete steady-state Kalman gain.

    Parameter ``A``:
        numpy.array(states x states), system matrix.

    Parameter ``C``:
        numpy.array(outputs x states), output matrix.

    Parameter ``Q``:
        numpy.array(states x states), process noise covariance matrix.

    Parameter ``R``:
        numpy.array(outputs x outputs), measurement noise covariance matrix.

    Returns:
        numpy.array(outputs x states), Kalman gain matrix.
    """
    P = sp.linalg.solve_discrete_are(a=A.T, b=C.T, q=Q, r=R)
    return np.linalg.solve(C @ P @ C.T + R, C @ P).T
