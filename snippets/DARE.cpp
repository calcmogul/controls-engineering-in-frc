#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/LU>

template <int States, int Inputs>
Eigen::Matrix<double, States, States> DARE(
    const Eigen::Matrix<double, States, States>& A,
    const Eigen::Matrix<double, States, Inputs>& B,
    const Eigen::Matrix<double, States, States>& Q,
    const Eigen::Matrix<double, Inputs, Inputs>& R) {
  // [1] E. K.-W. Chu, H.-Y. Fan, W.-W. Lin & C.-S. Wang
  //     "Structure-Preserving Algorithms for Periodic Discrete-Time
  //     Algebraic Riccati Equations",
  //     International Journal of Control, 77:8, 767-788, 2004.
  //     DOI: 10.1080/00207170410001714988
  //
  // Implements SDA algorithm on p. 5 of [1] (initial A, G, H are from (4)).
  using StateMatrix = Eigen::Matrix<double, States, States>;

  StateMatrix A_k = A;
  StateMatrix G_k = B * R.llt().solve(B.transpose());
  StateMatrix H_k;
  StateMatrix H_k1 = Q;

  do {
    H_k = H_k1;

    StateMatrix W = StateMatrix::Identity() + G_k * H_k;

    auto W_solver = W.lu();
    StateMatrix V_1 = W_solver.solve(A_k);
    StateMatrix V_2 = W_solver.solve(G_k.transpose()).transpose();

    G_k += A_k * V_2 * A_k.transpose();
    H_k1 = H_k + V_1.transpose() * H_k * A_k;
    A_k *= V_1;
  } while ((H_k1 - H_k).norm() > 1e-10 * H_k1.norm());

  return H_k1;
}
