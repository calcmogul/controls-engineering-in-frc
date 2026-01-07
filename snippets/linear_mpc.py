import frccontrol as fct
from sleipnir.optimization import Problem, bounds


class LinearMPC:
    def __init__(self, A, B, Q, R, dt, u_min, u_max, horizon):
        self.A_d, self.B_d = fct.discretize_ab(A, B, dt)
        self.Q, self.R = Q, R
        self.u_min, self.u_max = u_min, u_max
        self.N = int(horizon / dt)

    def calculate(self, x, r):
        problem = Problem()
        X = problem.decision_variable(self.A_d.shape[0], self.N + 1)
        U = problem.decision_variable(self.B_d.shape[1], self.N)

        J = 0
        for k in range(self.N + 1):
            J += (r - X[:, k : k + 1]).T @ self.Q @ (r - X[:, k : k + 1])
        for k in range(self.N):
            J += U[:, k : k + 1].T @ self.R @ U[:, k : k + 1]
        problem.minimize(J)

        problem.subject_to(X[:, :1] == x)
        for k in range(self.N):
            x_k, u_k = X[:, k : k + 1], U[:, k : k + 1]
            problem.subject_to(X[:, k + 1 : k + 2] == self.A_d @ x_k + self.B_d @ u_k)
            problem.subject_to(bounds(self.u_min, u_k, self.u_max))

        problem.solve()
        return U.value()[:, :1]
