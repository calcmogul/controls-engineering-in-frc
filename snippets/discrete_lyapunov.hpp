#pragma once

#include <Eigen/Core>

template <int States>
Eigen::Matrix<double, States, States> discrete_lyapunov(
    const Eigen::Matrix<double, States, States>& A,
    const Eigen::Matrix<double, States, States>& Q) {
  // Implements iteration (5) on p. 199 of [1] for X − UXV = W (2) with
  // U = A, V = Aᵀ, and W = Q.
  //
  // [1] R. A. Smith "Matrix Equation XA + BX = C", SIAM Journal on Applied
  //     Mathematics, 16:1, 198-201, 1968.
  //     DOI: 10.1137/0116017

  // A₀ = A
  // X₀ = Q
  Eigen::Matrix<double, States, States> A_k = A;
  Eigen::Matrix<double, States, States> X_k;
  Eigen::Matrix<double, States, States> X_k1 = Q;

  do {
    X_k = X_k1;

    // Xₖ₊₁ = Xₖ + AₖXₖAₖᵀ
    // Aₖ₊₁ = Aₖ²
    X_k1 = X_k + A_k * X_k * A_k.transpose();
    A_k *= A_k;
  } while ((X_k1 - X_k).norm() > 1e-10 * X_k1.norm());

  return X_k1;
}
