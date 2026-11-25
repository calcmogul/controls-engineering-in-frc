#pragma once

#include <cmath>

#include <Eigen/Core>
#include <Eigen/LU>

template <int States>
Eigen::Matrix<double, States, States> continuous_lyapunov(
    const Eigen::Matrix<double, States, States>& A,
    const Eigen::Matrix<double, States, States>& Q) {
  // Implements (2) and iteration (5) on pp. 198-199 of [1] for XA + BX = C
  // (1) with Aᵀ, A, −Q, and γ in place of A, B, C, and q.
  //
  // [1] R. A. Smith "Matrix Equation XA + BX = C", SIAM Journal on Applied
  //     Mathematics, 16:1, 198-201, 1968.
  //     DOI: 10.1137/0116017

  // The iteration converges fastest when γ ≈ |eig(A)|, so use the
  // eigenvalues' geometric mean |det(A)|^(1/n) (det(A) is their product).
  // The log avoids overflow.
  const double γ = std::exp(A.lu().logAbsDeterminant() / States);
  Eigen::Matrix<double, States, States> γI =
      γ * Eigen::Matrix<double, States, States>::Identity();

  // M⁻¹ = γI − A
  auto M_inv_solver = (γI - A).lu();

  // A bilinear transform with T = 2/γ discretizes the equation into
  // A_dXA_dᵀ − X + Q_d = 0 with the same solution, where
  //
  // A₀ = A_d = M(γI + A)
  // X₀ = Q_d = 2γM(MQ)ᵀ
  Eigen::Matrix<double, States, States> A_k = M_inv_solver.solve(γI + A);
  Eigen::Matrix<double, States, States> X_k;
  Eigen::Matrix<double, States, States> X_k1 =
      2.0 * γ * M_inv_solver.solve(M_inv_solver.solve(Q).transpose());

  do {
    X_k = X_k1;

    // Xₖ₊₁ = Xₖ + AₖXₖAₖᵀ
    // Aₖ₊₁ = Aₖ²
    X_k1 = X_k + A_k * X_k * A_k.transpose();
    A_k *= A_k;
  } while ((X_k1 - X_k).norm() > 1e-10 * X_k1.norm());

  return X_k1;
}
