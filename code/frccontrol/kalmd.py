import control as cnt
import numpy as np
import scipy as sp


def kalmd(sys, Q, R):
    """Solves for the steady state kalman gain and error covariance matrices.

    Keyword arguments:
    sys -- discrete state-space model
    Q -- process noise covariance matrix
    R -- measurement noise covariance matrix

    Returns:
    Kalman gain, error covariance matrix.
    """
    m = sys.A.shape[0]

    observability_rank = np.linalg.matrix_rank(cnt.obsv(sys.A, sys.C))
    if observability_rank != m:
        print("Warning: Observability of %d != %d, unobservable state",
              observability_rank, m)

    # Compute the steady state covariance matrix
    P_prior = sp.linalg.solve_discrete_are(a=sys.A.T, b=sys.C.T, q=Q, r=R)
    S = sys.C * P_prior * sys.C.T + R
    K = P_prior * sys.C.T * np.linalg.inv(S)
    P = (np.eye(m) - K * sys.C) * P_prior

    return K, P
