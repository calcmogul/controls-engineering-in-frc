#include <algorithm>
#include <array>
#include <cmath>

#include <Eigen/Core>
#include <units/time.h>

/**
 * Performs adaptive RKF45 integration of dx/dt = f(x, u) for dt, as described
 * in https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta%E2%80%93Fehlberg_method
 *
 * @param f        The function to integrate. It must take two arguments x and
 *                 u.
 * @param x        The initial value of x.
 * @param u        The value u held constant over the integration period.
 * @param dt       The time over which to integrate.
 * @param maxError The maximum acceptable truncation error. Usually a small
 *                 number like 1e-6.
 */
template <typename F, typename T, typename U>
T RKF45(F&& f, T x, U u, units::second_t dt, double maxError = 1e-6) {
  // See
  // https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta%E2%80%93Fehlberg_method
  // for the Butcher tableau the following arrays came from.
  constexpr int kDim = 6;

  // clang-format off
  constexpr double A[kDim - 1][kDim - 1]{
      {     1.0 / 4.0},
      {     3.0 / 32.0,       9.0 / 32.0},
      {1932.0 / 2197.0, -7200.0 / 2197.0,  7296.0 / 2197.0},
      {  439.0 / 216.0,             -8.0,   3680.0 / 513.0, -845.0 / 4104.0},
      {    -8.0 / 27.0,              2.0, -3544.0 / 2565.0, 1859.0 / 4104.0, -11.0 / 40.0}};
  // clang-format on

  constexpr std::array<double, kDim> b1{16.0 / 135.0,     0.0,
                                        6656.0 / 12825.0, 28561.0 / 56430.0,
                                        -9.0 / 50.0,      2.0 / 55.0};
  constexpr std::array<double, kDim> b2{
      25.0 / 216.0, 0.0, 1408.0 / 2565.0, 2197.0 / 4104.0, -1.0 / 5.0, 0.0};

  T newX;
  double truncationError;

  double dtElapsed = 0.0;
  double h = dt.to<double>();

  // Loop until we've gotten to our desired dt
  while (dtElapsed < dt.to<double>()) {
    do {
      // Only allow us to advance up to the dt remaining
      h = std::min(h, dt.to<double>() - dtElapsed);

      // Notice how the derivative in the Wikipedia notation is dy/dx.
      // That means their y is our x and their x is our t
      // clang-format off
      T k1 = f(x, u);
      T k2 = f(x + h * (A[0][0] * k1), u);
      T k3 = f(x + h * (A[1][0] * k1 + A[1][1] * k2), u);
      T k4 = f(x + h * (A[2][0] * k1 + A[2][1] * k2 + A[2][2] * k3), u);
      T k5 = f(x + h * (A[3][0] * k1 + A[3][1] * k2 + A[3][2] * k3 + A[3][3] * k4), u);
      T k6 = f(x + h * (A[4][0] * k1 + A[4][1] * k2 + A[4][2] * k3 + A[4][3] * k4 + A[4][4] * k5), u);
      // clang-format on

      newX = x + h * (b1[0] * k1 + b1[1] * k2 + b1[2] * k3 + b1[3] * k4 +
                      b1[4] * k5 + b1[5] * k6);
      truncationError = (h * ((b1[0] - b2[0]) * k1 + (b1[1] - b2[1]) * k2 +
                              (b1[2] - b2[2]) * k3 + (b1[3] - b2[3]) * k4 +
                              (b1[4] - b2[4]) * k5 + (b1[5] - b2[5]) * k6))
                            .norm();

      h *= 0.9 * std::pow(maxError / truncationError, 1.0 / 5.0);
    } while (truncationError > maxError);

    dtElapsed += h;
    x = newX;
  }

  return x;
}
