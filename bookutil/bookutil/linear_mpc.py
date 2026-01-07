from collections.abc import Callable

import frccontrol as fct
import numpy as np
import numpy.typing as npt
from sleipnir.autodiff import VariableMatrix
from sleipnir.optimization import Problem


class LinearMPC:
    """Linear model predictive controller."""

    def __init__(
        self,
        A: npt.NDArray[np.float64],
        B: npt.NDArray[np.float64],
        sample_period: float,
        cost: Callable[
            [VariableMatrix, VariableMatrix, npt.NDArray[np.float64]], VariableMatrix
        ],
        constraints: Callable[[Problem, VariableMatrix, VariableMatrix], None],
        initial_guess: Callable[
            [npt.NDArray[np.float64], npt.NDArray[np.float64], int],
            tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]],
        ],
        prediction_horizon: float,
        timeout: float,
    ):
        """
        Construct a linear model predictive controller.

        Parameter ``A``:
            Continuous system matrix.

        Parameter ``B``:
            Continuous input matrix.

        Parameter ``sample_period``:
            Sample period in seconds.

        Parameter ``cost``:
            Callback for cost function.

        Parameter ``constraints``:
            Callback for setting constraints (control input limits, etc.).

        Parameter ``initial_guess``:
            Callback that takes the current state x, reference r, and number of
            samples N and returns the initial guesses for X (states x (N + 1)
            and U (inputs x N).

            Subsequent iterations warm start from the previous iteration's
            solution.

        Parameter ``prediction_horizon``:
            Prediction horizon in seconds.

        Parameter ``timeout``:
            The maximum time in seconds the solver can spend before returning a
            solution.
        """
        self.A_d, self.B_d = fct.discretize_ab(A, B, sample_period)
        self.cost = cost
        self.constraints = constraints
        self.initial_guess = initial_guess
        self.timeout = timeout

        self.N: int = int(prediction_horizon / sample_period)

        self.warm_startable: bool = False

    def calculate(
        self, x: npt.NDArray[np.float64], r: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        """
        Calculates the next control input.

        Parameter ``x``:
            The current state.

        Parameter ``r``:
            The reference.

        Returns:
            The control input to apply for this timestep.
        """
        problem = Problem()

        X = problem.decision_variable(self.A_d.shape[0], self.N + 1)
        U = problem.decision_variable(self.B_d.shape[1], self.N)

        problem.minimize(self.cost(X, U, r))

        # Initial state constraint
        problem.subject_to(X[:, :1] == x)

        # Dynamics constraints
        for k in range(self.N):
            problem.subject_to(
                X[:, k + 1 : k + 2]
                == self.A_d @ X[:, k : k + 1] + self.B_d @ U[:, k : k + 1]
            )

        self.constraints(problem, X, U)

        # Initial guess
        if not self.warm_startable:
            self.X_warm_start, self.U_warm_start = self.initial_guess(x, r, self.N)
            self.warm_startable = True
        X.set_value(self.X_warm_start)
        U.set_value(self.U_warm_start)

        problem.solve(timeout=self.timeout)

        self.x_warm_start = X.value()
        self.u_warm_start = U.value()

        return self.u_warm_start[:, :1]
