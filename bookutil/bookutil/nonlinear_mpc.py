from collections.abc import Callable

import frccontrol as fct
import numpy as np
import numpy.typing as npt
from sleipnir.autodiff import VariableMatrix
from sleipnir.optimization import Problem


class NonlinearMPC:
    """Nonlinear model predictive controller."""

    def __init__(
        self,
        states: int,
        inputs: int,
        f: Callable[[VariableMatrix, VariableMatrix], VariableMatrix],
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
        Construct a nonlinear model predictive controller.

        Parameter ``states``:
            Number of states.

        Parameter ``inputs``:
            Number of inputs.

        Parameter ``f``:
            Dynamics dx/dt = f(x, u).

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
        self.states = states
        self.inputs = inputs
        self.f = f
        self.sample_period = sample_period
        self.cost = cost
        self.constraints = constraints
        self.initial_guess = initial_guess
        self.timeout = timeout

        self.N: int = int(prediction_horizon / sample_period)

        self.problem = Problem()

        self.X = self.problem.decision_variable(self.states, self.N + 1)
        self.U = self.problem.decision_variable(self.inputs, self.N)

        # Initial state constraint (avoids 0 or 1 to prevent expression pruning
        # during initial constraint setup)
        self.x_0 = VariableMatrix.constant(self.states, 1, 2.0)
        self.problem.subject_to(self.X[:, :1] == self.x_0)

        # Dynamics constraints
        for k in range(self.N):
            self.problem.subject_to(
                self.X[:, k + 1 : k + 2]
                == fct.rk4(
                    self.f,
                    self.X[:, k : k + 1],
                    self.U[:, k : k + 1],
                    self.sample_period,
                )
            )

        self.constraints(self.problem, self.X, self.U)

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
        self.problem.minimize(self.cost(self.X, self.U, r))

        # Update initial state constraint
        self.x_0.set_value(x)

        # Initial guess
        if not self.warm_startable:
            X_warm_start, U_warm_start = self.initial_guess(x, r, self.N)
            self.X.set_value(X_warm_start)
            self.U.set_value(U_warm_start)
            self.warm_startable = True

        self.problem.solve(timeout=self.timeout, diagnostics=True)

        return self.U.value()[:, :1]
