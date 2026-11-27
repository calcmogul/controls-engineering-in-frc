#pragma once

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include "dare.hpp"

template <int States, int Inputs>
Eigen::Matrix<double, Inputs, States> lqr(
    const Eigen::Matrix<double, States, States>& A,
    const Eigen::Matrix<double, States, Inputs>& B,
    const Eigen::Matrix<double, States, States>& Q,
    const Eigen::Matrix<double, Inputs, Inputs>& R,
    const Eigen::Matrix<double, States, Inputs>& N) {
  auto R_llt = R.llt();
  Eigen::Matrix<double, States, States> A_2 =
      A - B * R_llt.solve(N.transpose());
  Eigen::Matrix<double, States, States> Q_2 =
      Q - N * R_llt.solve(N.transpose());

  Eigen::Matrix<double, States, States> P =
      dare<States, Inputs>(A_2, B, Q_2, R);

  return (B.transpose() * P * B + R)
      .llt()
      .solve(B.transpose() * P * A + N.transpose());
}
