#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>
#include <Eigen/SparseLU>

Eigen::SparseMatrix<double> identity(int rows) {
  return Eigen::SparseMatrix<double>{
      Eigen::VectorXd::Constant(rows, 1.0).asDiagonal()};
}

Eigen::SparseMatrix<double> dare(const Eigen::SparseMatrix<double>& A,
                                 const Eigen::SparseMatrix<double>& B,
                                 const Eigen::SparseMatrix<double>& Q,
                                 const Eigen::SparseMatrix<double>& R) {
  // [1] E. K.-W. Chu, H.-Y. Fan, W.-W. Lin & C.-S. Wang
  //     "Structure-Preserving Algorithms for Periodic Discrete-Time
  //     Algebraic Riccati Equations",
  //     International Journal of Control, 77:8, 767-788, 2004.
  //     DOI: 10.1080/00207170410001714988
  //
  // Implements SDA algorithm on p. 5 of [1] (initial A, G, H are from (4)).
  using StateMatrix = Eigen::SparseMatrix<double>;

  StateMatrix A_k = A;
  Eigen::SimplicialLLT R_llt{R};
  StateMatrix G_k = B * R_llt.solve(StateMatrix{B.transpose()});
  StateMatrix H_k;
  StateMatrix H_k1 = Q;

  do {
    H_k = H_k1;

    StateMatrix W = identity(A.rows()) + G_k * H_k;

    Eigen::SparseLU W_solver{W};
    StateMatrix V_1 = W_solver.solve(A_k);
    StateMatrix V_2 = W_solver.solve(G_k);

    G_k += StateMatrix{A_k * V_2 * A_k.transpose()};
    H_k1 = H_k + V_1.transpose() * H_k * A_k;
    A_k = A_k * StateMatrix{V_1};
  } while ((H_k1 - H_k).norm() > 1e-10 * H_k1.norm());

  return H_k1;
}
