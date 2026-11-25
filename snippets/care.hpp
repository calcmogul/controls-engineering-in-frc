#pragma once

#include <cmath>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/LU>

template <int States, int Inputs>
Eigen::Matrix<double, States, States> care(
    const Eigen::Matrix<double, States, States>& A,
    const Eigen::Matrix<double, States, Inputs>& B,
    const Eigen::Matrix<double, States, States>& Q,
    const Eigen::Matrix<double, Inputs, Inputs>& R) {
  // Implements SDA algorithm on p. 60 of [1] (A₀, G₀, H₀ from (9)–(11)).
  //
  // [1] E. K.-W. Chu, H.-Y. Fan & W.-W. Lin "A structure-preserving doubling
  //     algorithm for continuous-time algebraic Riccati equations",
  //     Linear Algebra and its Applications, 396, 55-80, 2005.
  //     DOI: 10.1016/j.laa.2004.10.010

  // G = BR⁻¹Bᵀ
  Eigen::Matrix<double, States, States> G = B * R.llt().solve(B.transpose());

  // SDA converges fastest when γ ≈ |eig(H)| where H is the Hamiltonian
  // [A −G; −Q −Aᵀ], so use the eigenvalues' geometric mean |det(H)|^(1/2n)
  // (det(H) is their product). The log avoids overflow.
  Eigen::Matrix<double, 2 * States, 2 * States> H;
  H << A, -G, -Q, -A.transpose();
  double γ = std::exp(H.lu().logAbsDeterminant() / (2 * States));

  // Increase γ until A_γ = A − γI is well-conditioned
  Eigen::Matrix<double, States, States> I =
      Eigen::Matrix<double, States, States>::Identity();
  auto A_γ_solver = (A - γ * I).lu();
  while (A_γ_solver.rcond() < 1e-8) {
    γ *= 1.5;
    A_γ_solver.compute(A - γ * I);
  }
  Eigen::Matrix<double, States, States> γI = γ * I;

  // Solve A_γᵀV = Q for V
  Eigen::Matrix<double, States, States> V = A_γ_solver.transpose().solve(Q);

  // A bilinear transform with T = 2/γ discretizes the CARE into the DARE
  // X = A₀ᵀX(I + G₀X)⁻¹A₀ + H₀ with the same solution, where
  //
  // W_γ = A_γ + GV
  // A₀ = I + 2γW_γ⁻¹
  // G₀ = 2γA_γ⁻¹(W_γ⁻¹G)ᵀ
  // H₀ = W_γ⁻ᵀ(2γVᵀ)
  auto W_γ_solver = (A - γI + G * V).lu();
  Eigen::Matrix<double, States, States> A_k = I + W_γ_solver.solve(2.0 * γI);
  Eigen::Matrix<double, States, States> G_k =
      2.0 * γ * A_γ_solver.solve(W_γ_solver.solve(G).transpose());
  Eigen::Matrix<double, States, States> H_k;
  Eigen::Matrix<double, States, States> H_k1 =
      W_γ_solver.transpose().solve(2.0 * γ * V.transpose());

  do {
    H_k = H_k1;

    // Solve WV₁ = Aₖ for V₁ and V₂Wᵀ = Gₖ for V₂ where W = I + GₖHₖ.
    // V₂ required transpose and V₂, Gₖ symmetry.
    auto W_solver = (I + G_k * H_k).lu();
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
