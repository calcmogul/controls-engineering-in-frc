def kalman(A, B, C, Q, R):
  """Solves for the steady state kalman gain and covariance matricies.

    Args:
      A, B, C: SS matricies.
      Q: The model uncertantity
      R: The measurement uncertainty

    Returns:
      KalmanGain, Covariance.
  """
  I = numpy.matrix(numpy.eye(Q.shape[0]))
  Z = numpy.matrix(numpy.zeros(Q.shape[0]))
  n = A.shape[0]
  m = C.shape[0]

  controllability_rank = numpy.linalg.matrix_rank(ctrb(A.T, C.T))
  if controllability_rank != n:
    print('warning: Observability of %d != %d, unobservable state',
                 controllability_rank, n)

  # Compute the steady state covariance matrix.
  P_prior, rcond, w, S, T = slycot.sb02od(n=n, m=m, A=A.T, B=C.T, Q=Q, R=R, dico='D')
  S = C * P_prior * C.T + R
  K = numpy.linalg.lstsq(S.T, (P_prior * C.T).T)[0].T
  P = (I - K * C) * P_prior

  return K, P
