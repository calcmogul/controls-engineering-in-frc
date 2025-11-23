"""Trajectory generation utilities."""

from __future__ import annotations

import math
from bisect import bisect_left
from dataclasses import dataclass

import frccontrol as fct
import numpy as np
from sleipnir.autodiff import VariableMatrix, cos, sin
from sleipnir.optimization import Problem


def lerp(a: float, b: float, t: float) -> float:
    """
    Computes the linear interpolation between a and b, if the parameter t is
    inside [0, 1), or the linear extrapolation otherwise.

    Parameter ``a``:
        The first value.
    Parameter ``b``:
        The second value.
    Parameter ``t``:
        The interpolant.
    """
    return a + (b - a) * t


@dataclass
class UnicycleSample:
    """Unicycle trajectory sample."""

    time: float
    x: float
    y: float
    θ: float
    v: float
    ω: float
    a: float
    α: float

    def interpolate(self, other: UnicycleSample, time: float) -> UnicycleSample:
        """
        Interpolates the sample.

        Parameter ``time``:
            The time of the interpolated state.
        """

        def f(x, u):
            #  x = [x, y, θ, v, ω]
            #  u = [a, α]
            #
            #  ẋ = v cosθ
            #  ẏ = v sinθ
            #  θ̇ = ω
            #  v̇ = a
            #  ω̇ = α
            θ = x[2, 0]
            v = x[3, 0]
            ω = x[4, 0]
            a = u[0, 0]
            α = u[1, 0]
            return np.array([[v * math.cos(θ)], [v * math.sin(θ)], [ω], [a], [α]])

        sample = fct.rk4(
            f,
            np.array([[self.x], [self.y], [self.θ], [self.v], [self.ω]]),
            np.array([[self.a], [self.α]]),
            time - self.time,
        )

        scale = (time - self.time) / (other.time - self.time)
        return UnicycleSample(
            time,
            sample[0, 0],
            sample[1, 0],
            sample[2, 0],
            sample[3, 0],
            sample[4, 0],
            lerp(self.a, other.a, scale),
            lerp(self.α, other.α, scale),
        )


class UnicycleTrajectory:
    """Represents a unicycle trajectory."""

    def __init__(self, samples: list[UnicycleSample]):
        if len(samples) == 0:
            raise RuntimeError("Trajectory must have at laest one sample")
        self.__samples: list[UnicycleSample] = samples

    def sample(self, time: float) -> UnicycleSample:
        """
        Samples the trajectory at the given time.

        Parameter ``time``:
            The time at which to sample in seconds.
        """
        if time <= 0.0:
            return self.__samples[0]
        elif time >= self.__samples[-1].time:
            return self.__samples[-1]

        i = bisect_left(self.__samples, time, key=lambda x: x.time)
        left = self.__samples[i - 1]
        right = self.__samples[i]
        return left.interpolate(right, time)

    def total_time(self) -> float:
        """Returns the trajectory's duration in seconds."""
        return self.__samples[-1].time


def generate_trajectory(
    waypoints: list[fct.Pose2d], v_max: float, ω_max: float, a_max: float, α_max: float
) -> UnicycleTrajectory:
    """
    Generates a trajectory through the given waypoints satisfying the given
    constraints.

    Parameter ``waypoints``:
        The list of waypoints.
    Parameter ``v_max``:
        The maximum linear velocity.
    Parameter ``ω_max``:
        The maximum angular velocity.
    Parameter ``a_max``:
        The maximum linear acceleration.
    Parameter ``α_max``:
        The maximum angular acceleration.
    """

    def f(x, u):
        #  x = [x, y, θ, v, ω]
        #  u = [a, α]
        #
        #  ẋ = v cosθ
        #  ẏ = v sinθ
        #  θ̇ = ω
        #  v̇ = a
        #  ω̇ = α
        θ = x[2, 0]
        v = x[3, 0]
        ω = x[4, 0]
        a = u[0, 0]
        α = u[1, 0]
        return VariableMatrix([[v * cos(θ)], [v * sin(θ)], [ω], [a], [α]])

    N_sgmt = 40
    num_segments = len(waypoints) - 1
    T_max = 5.0

    problem = Problem()

    X = problem.decision_variable(5, num_segments * N_sgmt + 1)
    U = problem.decision_variable(2, num_segments * N_sgmt)
    dts = problem.decision_variable(num_segments * N_sgmt)

    # Pose initial guess
    for sgmt in range(num_segments):
        for k in range(N_sgmt * sgmt, N_sgmt * (sgmt + 1)):
            scale = (k - N_sgmt * sgmt) / N_sgmt
            X[0, k].set_value(
                lerp(
                    waypoints[sgmt].x,
                    waypoints[sgmt + 1].x,
                    scale,
                )
            )
            X[1, k].set_value(
                lerp(
                    waypoints[sgmt].y,
                    waypoints[sgmt + 1].y,
                    scale,
                )
            )
            X[2, k].set_value(
                math.atan2(
                    waypoints[sgmt + 1].y - waypoints[sgmt].y,
                    waypoints[sgmt + 1].x - waypoints[sgmt].x,
                )
            )
    for sgmt in range(num_segments):
        X[0, N_sgmt * sgmt].set_value(waypoints[sgmt].x)
        X[1, N_sgmt * sgmt].set_value(waypoints[sgmt].y)
        X[2, N_sgmt * sgmt].set_value(waypoints[sgmt].rotation.radians)
    X[0, -1].set_value(waypoints[-1].x)
    X[1, -1].set_value(waypoints[-1].y)
    X[2, -1].set_value(waypoints[-1].rotation.radians)

    # dt constraints
    problem.subject_to(dts >= 0)
    problem.subject_to(dts <= T_max / (num_segments * N_sgmt))
    for sgmt in range(num_segments):
        for k in range(N_sgmt * sgmt, N_sgmt * (sgmt + 1) - 1):
            problem.subject_to(dts[k] == dts[k + 1])
    for k in range(X.shape[1] - 1):
        dts[k].set_value(T_max / N_sgmt)

    # Waypoint constraints
    for sgmt, waypoint in enumerate(waypoints):
        x = X[0, sgmt * N_sgmt]
        y = X[1, sgmt * N_sgmt]
        θ = X[2, sgmt * N_sgmt]
        problem.subject_to(x == waypoint.x)
        problem.subject_to(y == waypoint.y)
        problem.subject_to(cos(θ) == waypoint.rotation.cos)
        problem.subject_to(sin(θ) == waypoint.rotation.sin)

    # Dynamics constraints
    for k in range(X.shape[1] - 1):
        x_k = X[:, k]
        x_k1 = X[:, k + 1]
        u_k = U[:, k]
        dt_k = dts[k]
        problem.subject_to(x_k1 == fct.rk4(f, x_k, u_k, dt_k))

    # Initial and final velocities
    problem.subject_to(X[3:5, 0] == 0)
    problem.subject_to(X[3:5, -1] == 0)

    # Velocity limits
    problem.subject_to(X[3, :] >= -v_max)
    problem.subject_to(X[3, :] <= v_max)
    problem.subject_to(X[4, :] >= -ω_max)
    problem.subject_to(X[4, :] <= ω_max)

    # Acceleration limits
    problem.subject_to(U[0, :] >= -a_max)
    problem.subject_to(U[0, :] <= a_max)
    problem.subject_to(U[1, :] >= -α_max)
    problem.subject_to(U[1, :] <= α_max)

    problem.minimize(sum(dts))

    problem.solve()

    samples: list[UnicycleSample] = []
    t = 0.0
    for k in range(X.shape[1]):
        if k < X.shape[1] - 1:
            samples.append(
                UnicycleSample(
                    t,
                    X[0, k].value(),
                    X[1, k].value(),
                    X[2, k].value(),
                    X[3, k].value(),
                    X[4, k].value(),
                    U[0, k].value(),
                    U[1, k].value(),
                )
            )
            t += dts[k].value()
        else:
            samples.append(
                UnicycleSample(
                    t,
                    X[0, k].value(),
                    X[1, k].value(),
                    X[2, k].value(),
                    X[3, k].value(),
                    X[4, k].value(),
                    0.0,
                    0.0,
                )
            )

    return UnicycleTrajectory(samples)
