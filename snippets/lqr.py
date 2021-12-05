import numpy as np
import scipy as sp


def lqr(A, B, Q, R, N):
    """Solves for the optimal discrete linear-quadratic regulator (LQR).

    Keyword arguments:
    A -- numpy.array(states x states), system matrix.
    B -- numpy.array(states x inputs), input matrix.
    Q -- numpy.array(states x states), state cost matrix.
    R -- numpy.array(inputs x inputs), control effort cost matrix.
    N -- numpy.array(states x inputs), cross weight matrix.

    Returns:
    K -- numpy.array(states x inputs), controller gain matrix.
    """
    P = sp.linalg.solve_discrete_are(a=A, b=B, q=Q, r=R, s=N)
    return np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A + N.T)
