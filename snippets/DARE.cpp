#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/QR>

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
  // Implements the SSCA algorithm on page 12 of [1]. See equation (4) of [1]
  // for initial values of A, G, and H.

  Eigen::MatrixXd A_k = A;
  Eigen::MatrixXd A_k1 = A;

  Eigen::MatrixXd G_k = B * R.llt().solve(B.transpose());

  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(A.rows(), A.cols());

  Eigen::MatrixXd H_k = Q;
  Eigen::MatrixXd H_k1 = Q;

  do {
    A_k = A_k1;
    H_k = H_k1;

    Eigen::MatrixXd W = I + H_k * G_k;

    // W is symmetric positive definite, but LLT solver [2] + iterative
    // refinement to fix inaccuracy [3] was slower than just householder QR
    // solver.
    //
    // [2] https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
    // [3] https://en.wikipedia.org/wiki/Iterative_refinement
    auto W_solver = W.householderQr();

    Eigen::MatrixXd V_1 = W_solver.solve(A_k.transpose());
    Eigen::MatrixXd V_2 = W_solver.solve(H_k);

    A_k1 = V_1.transpose() * A_k;
    G_k += A_k * G_k * V_1;
    H_k1 = H_k + A_k.transpose() * V_2 * A_k;
  } while ((H_k1 - H_k).norm() > 1e-10 * H_k1.norm());

  return H_k1;
}
