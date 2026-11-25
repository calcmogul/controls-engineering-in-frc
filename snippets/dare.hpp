#pragma once

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/LU>

template <int States, int Inputs>
Eigen::Matrix<double, States, States> dare(
    const Eigen::Matrix<double, States, States>& A,
    const Eigen::Matrix<double, States, Inputs>& B,
    const Eigen::Matrix<double, States, States>& Q,
    const Eigen::Matrix<double, Inputs, Inputs>& R) {
  // Implements SDA algorithm on p. 5 of [1] (A₀, G₀, H₀ from (4)).
  //
  // [1] E. K.-W. Chu, H.-Y. Fan, W.-W. Lin & C.-S. Wang
  //     "Structure-Preserving Algorithms for Periodic Discrete-Time
  //     Algebraic Riccati Equations", International Journal of Control,
  //     77:8, 767-788, 2004.
  //     DOI: 10.1080/00207170410001714988

  // A₀ = A
  // G₀ = BR⁻¹Bᵀ
  // H₀ = Q
  Eigen::Matrix<double, States, States> A_k = A;
  Eigen::Matrix<double, States, States> G_k =
      B * R.llt().solve(B.transpose());
  Eigen::Matrix<double, States, States> H_k;
  Eigen::Matrix<double, States, States> H_k1 = Q;

  do {
    H_k = H_k1;

    // Solve WV₁ = Aₖ for V₁ and V₂Wᵀ = Gₖ for V₂ where W = I + GₖHₖ.
    // V₂ required transpose and V₂, Gₖ symmetry.
    auto W_solver =
        (Eigen::Matrix<double, States, States>::Identity() + G_k * H_k).lu();
    Eigen::Matrix<double, States, States> V_1 = W_solver.solve(A_k);
    Eigen::Matrix<double, States, States> V_2 = W_solver.solve(G_k);

    // Gₖ₊₁ = Gₖ + AₖV₂Aₖᵀ
    // Hₖ₊₁ = Hₖ + V₁ᵀHₖAₖ
    // Aₖ₊₁ = AₖV₁
    G_k += A_k * V_2 * A_k.transpose();
    H_k1 = H_k + V_1.transpose() * H_k * A_k;
    A_k *= V_1;
  } while ((H_k1 - H_k).norm() > 1e-10 * H_k1.norm());

  return H_k1;
}
