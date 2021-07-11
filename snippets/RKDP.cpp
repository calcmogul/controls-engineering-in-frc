#include <algorithm>
#include <array>
#include <cmath>

#include <Eigen/Core>
#include <units/time.h>

/**
 * Performs adaptive Dormand-Prince integration of dx/dt = f(x, u) for dt.
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
T RKDP(F&& f, T x, U u, units::second_t dt, double maxError = 1e-6) {
  // See https://en.wikipedia.org/wiki/Dormand%E2%80%93Prince_method for the
  // Butcher tableau the following arrays came from.

  constexpr int kDim = 7;

  // This is used for time-varying integration
  // constexpr std::array<double, kDim - 1> A{
  //     1.0 / 5.0, 3.0 / 10.0, 4.0 / 5.0, 8.0 / 9.0, 1.0, 1.0};

  // clang-format off
  constexpr double B[kDim - 1][kDim - 1]{
      {      1.0 / 5.0},
      {      3.0 / 40.0,        9.0 / 40.0},
      {     44.0 / 45.0,      -56.0 / 15.0,       32.0 / 9.0},
      {19372.0 / 6561.0, -25360.0 / 2187.0, 64448.0 / 6561.0, -212.0 / 729.0},
      { 9017.0 / 3168.0,     -355.0 / 33.0, 46732.0 / 5247.0,   49.0 / 176.0, -5103.0 / 18656.0},
      {    35.0 / 384.0,               0.0,   500.0 / 1113.0,  125.0 / 192.0,  -2187.0 / 6784.0, 11.0 / 84.0}};
  // clang-format on

  constexpr std::array<double, kDim> C1{
      35.0 / 384.0, 0.0, 500.0 / 1113.0, 125.0 / 192.0, -2187.0 / 6784.0,
      11.0 / 84.0,  0.0};
  constexpr std::array<double, kDim> C2{5179.0 / 57600.0,    0.0,
                                        7571.0 / 16695.0,    393.0 / 640.0,
                                        -92097.0 / 339200.0, 187.0 / 2100.0,
                                        1.0 / 40.0};

  T newX;
  double truncationError;

  double dtElapsed = 0.0;
  double h = dt.to<double>();

  // Loop until we've gotten to our desired dt
  while (dtElapsed < dt.to<double>()) {
    do {
      // Only allow us to advance up to the dt remaining
      h = std::min(h, dt.to<double>() - dtElapsed);

      // clang-format off
      T k1 = f(x, u) * h;
      T k2 = f(x + k1 * B[0][0], u) * h;
      T k3 = f(x + k1 * B[1][0] + k2 * B[1][1], u) * h;
      T k4 = f(x + k1 * B[2][0] + k2 * B[2][1] + k3 * B[2][2], u) * h;
      T k5 = f(x + k1 * B[3][0] + k2 * B[3][1] + k3 * B[3][2] + k4 * B[3][3], u) * h;
      T k6 = f(x + k1 * B[4][0] + k2 * B[4][1] + k3 * B[4][2] + k4 * B[4][3] + k5 * B[4][4], u) * h;
      // clang-format on

      // Since the final row of B and the array C1 have the same coefficients
      // and k7 has no effect on newX, we can reuse the calculation.
      newX = x + k1 * B[5][0] + k2 * B[5][1] + k3 * B[5][2] + k4 * B[5][3] +
             k5 * B[5][4] + k6 * B[5][5];
      T k7 = f(newX, u) * h;

      truncationError =
          (k1 * (C1[0] - C2[0]) + k2 * (C1[1] - C2[1]) + k3 * (C1[2] - C2[2]) +
           k4 * (C1[3] - C2[3]) + k5 * (C1[4] - C2[4]) + k6 * (C1[5] - C2[5]) +
           k7 * (C1[6] - C2[6]))
              .norm();

      h = 0.9 * h * std::pow(maxError / truncationError, 1.0 / 5.0);
    } while (truncationError > maxError);

    dtElapsed += h;
    x = newX;
  }

  return x;
}
