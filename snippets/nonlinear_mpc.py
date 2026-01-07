import frccontrol as fct
from sleipnir.optimization import Problem, bounds


class NonlinearMPC:
    def __init__(self, states, inputs, f, Q, R, dt, u_min, u_max, horizon):
        self.states, self.inputs = states, inputs
        self.f = f
        self.Q, self.R = Q, R
        self.dt = dt
        self.u_min, self.u_max = u_min, u_max
        self.N = int(horizon / dt)

    def calculate(self, x, r):
        problem = Problem()
        X = problem.decision_variable(self.states, self.N + 1)
        U = problem.decision_variable(self.inputs, self.N)

        J = 0
        for k in range(self.N + 1):
            J += (r - X[:, k : k + 1]).T @ self.Q @ (r - X[:, k : k + 1])
        for k in range(self.N):
            J += U[:, k : k + 1].T @ self.R @ U[:, k : k + 1]
        problem.minimize(J)

        problem.subject_to(X[:, :1] == x)
        for k in range(self.N):
            x_k, u_k = X[:, k : k + 1], U[:, k : k + 1]
            problem.subject_to(
                X[:, k + 1 : k + 2] == fct.rk4(self.f, x_k, u_k, self.dt)
            )
            problem.subject_to(bounds(self.u_min, u_k, self.u_max))

        problem.solve()
        return U.value()[:, :1]
