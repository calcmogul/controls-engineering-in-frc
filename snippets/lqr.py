"""Function for computing the infinite horizon LQR."""

import numpy as np
import scipy as sp


def lqr(A, B, Q, R, N):
    """
    Solves for the optimal discrete linear-quadratic regulator (LQR).

    Args:
        A: System matrix, states x states.
        B: Input matrix, states x inputs.
        Q: State cost matrix, states x states.
        R: Control effort cost matrix, inputs x inputs.
        N: Cross weight matrix, states x inputs.

    Returns:
        Controller gain matrix, inputs x states.
    """
    P = sp.linalg.solve_discrete_are(a=A, b=B, q=Q, r=R, s=N)
    return np.linalg.solve(B.T @ P @ B + R, B.T @ P @ A + N.T)
