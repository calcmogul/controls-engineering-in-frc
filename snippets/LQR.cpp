#include <Eigen/Cholesky>
#include <Eigen/Core>

#include "DARE.hpp"

template <int States, int Inputs>
Eigen::Matrix<double, Inputs, States> LQR(
    const Eigen::Matrix<double, States, States>& A,
    const Eigen::Matrix<double, States, Inputs>& B,
    const Eigen::Matrix<double, States, States>& Q,
    const Eigen::Matrix<double, Inputs, Inputs>& R,
    const Eigen::Matrix<double, States, Inputs>& N) {
  using StateMatrix = Eigen::Matrix<double, States, States>;

  auto R_llt = R.llt();
  StateMatrix A_2 = A - B * R_llt.solve(N.transpose());
  StateMatrix Q_2 = Q - N * R_llt.solve(N.transpose());

  StateMatrix P = DARE<States, Inputs>(A_2, B, Q_2, R);

  return (B.transpose() * P * B + R)
      .llt()
      .solve(B.transpose() * P * A + N.transpose());
}
