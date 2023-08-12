#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/LU>

Eigen::MatrixXd DARE(const Eigen::Ref<const Eigen::MatrixXd>& A,
                     const Eigen::Ref<const Eigen::MatrixXd>& B,
                     const Eigen::Ref<const Eigen::MatrixXd>& Q,
                     const Eigen::Ref<const Eigen::MatrixXd>& R) {
  // [1] E. K.-W. Chu, H.-Y. Fan, W.-W. Lin & C.-S. Wang
  //     "Structure-Preserving Algorithms for Periodic Discrete-Time
  //     Algebraic Riccati Equations",
  //     International Journal of Control, 77:8, 767-788, 2004.
  //     DOI: 10.1080/00207170410001714988
  //
  // Implements SDA algorithm on p. 5 of [1] (initial A, G, H are from (4)).
  Eigen::MatrixXd A_k = A;
  Eigen::MatrixXd G_k = B * R.llt().solve(B.transpose());
  Eigen::MatrixXd H_k;
  Eigen::MatrixXd H_k1 = Q;

  do {
    H_k = H_k1;

    Eigen::MatrixXd W =
        Eigen::MatrixXd::Identity(H_k.rows(), H_k.cols()) + G_k * H_k;

    auto W_solver = W.lu();
    Eigen::MatrixXd V_1 = W_solver.solve(A_k);
    Eigen::MatrixXd V_2 = W_solver.solve(G_k.transpose()).transpose();

    G_k += A_k * V_2 * A_k.transpose();
    H_k1 = H_k + V_1.transpose() * H_k * A_k;
    A_k *= V_1;
  } while ((H_k1 - H_k).norm() > 1e-10 * H_k1.norm());

  return H_k1;
}
