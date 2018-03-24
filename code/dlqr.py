def dlqr(A, B, Q, R):
  """Solves for the optimal lqr controller.

    x(n+1) = A * x(n) + B * u(n)
    J = sum(0, inf, x.T * Q * x + u.T * R * u)
  """

  # P = (A.T * P * A) - (A.T * P * B * numpy.linalg.inv(R + B.T * P *B) * (A.T * P.T * B).T + Q

  P, rcond, w, S, T = slycot.sb02od(
      n=A.shape[0], m=B.shape[1], A=A, B=B, Q=Q, R=R, dico='D')

  F = numpy.linalg.inv(R + B.T * P * B) * B.T * P * A
  return F
