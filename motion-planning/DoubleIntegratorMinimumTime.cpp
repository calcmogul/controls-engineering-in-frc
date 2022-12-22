#include <chrono>
#include <fstream>

#include <Eigen/Core>
#include <fmt/core.h>
#include <sleipnir/optimization/OptimizationProblem.hpp>

int main() {
  using namespace std::chrono_literals;

  constexpr auto T = 3.5s;
  constexpr auto dt = 5ms;
  constexpr int N = T / dt;

  constexpr double r = 2.0;

  sleipnir::OptimizationProblem problem;

  // 2x1 state vector with N + 1 timesteps (includes last state)
  auto X = problem.DecisionVariable(2, N + 1);

  // 1x1 input vector with N timesteps (input at last state doesn't matter)
  auto U = problem.DecisionVariable(1, N);

  // Kinematics constraint assuming constant acceleration between timesteps
  for (int k = 0; k < N; ++k) {
    constexpr double t = std::chrono::duration<double>{dt}.count();
    auto p_k1 = X(0, k + 1);
    auto v_k1 = X(1, k + 1);
    auto p_k = X(0, k);
    auto v_k = X(1, k);
    auto a_k = U(0, k);

    problem.SubjectTo(p_k1 == p_k + v_k * t);
    problem.SubjectTo(v_k1 == v_k + a_k * t);
  }

  // Start and end at rest
  problem.SubjectTo(X.Col(0) == Eigen::Matrix<double, 2, 1>{{0.0}, {0.0}});
  problem.SubjectTo(X.Col(N) == Eigen::Matrix<double, 2, 1>{{r}, {0.0}});

  // Limit velocity
  problem.SubjectTo(-1 <= X.Row(1));
  problem.SubjectTo(X.Row(1) <= 1);

  // Limit acceleration
  problem.SubjectTo(-1 <= U);
  problem.SubjectTo(U <= 1);

  // Cost function - minimize position error
  sleipnir::Variable J = 0.0;
  for (int k = 0; k < N + 1; ++k) {
    J += sleipnir::pow(r - X(0, k), 2);
  }
  problem.Minimize(J);

  problem.Solve();

  // Log states for offline viewing
  std::ofstream states{"DoubleIntegratorMinimumTimeStates.csv"};
  if (states.is_open()) {
    states << "Time (s),Position (m),Velocity (m/s)\n";

    constexpr double t = std::chrono::duration<double>{dt}.count();
    for (int k = 0; k < N + 1; ++k) {
      states << fmt::format("{},{},{}\n", k * t, X.Value(0, k),
                            X.Value(1, k));
    }
  }

  // Log inputs for offline viewing
  std::ofstream inputs{"DoubleIntegratorMinimumTimeInputs.csv"};
  if (inputs.is_open()) {
    inputs << "Time (s),Acceleration (m/s²)\n";

    constexpr double t = std::chrono::duration<double>{dt}.count();
    for (int k = 0; k < N + 1; ++k) {
      if (k < N) {
        inputs << fmt::format("{},{}\n", k * t, U.Value(0, k));
      } else {
        inputs << fmt::format("{},{}\n", k * t, 0.0);
      }
    }
  }
}
