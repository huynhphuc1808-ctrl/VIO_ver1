#ifndef VIO_MSCKF_NOISEMANAGER_H
#define VIO_MSCKF_NOISEMANAGER_H

#include <math.h>

#include "utils/print.h"

namespace vio_msckf {

/**
 * @brief Struct of our imu noise parameters
 */
struct NoiseManager {

  /// Gyroscope white noise (rad/s/sqrt(hz))
  double sigma_w = 1.6968e-04;

  /// Gyroscope white noise covariance
  double sigma_w_2 = pow(1.6968e-04, 2);

  /// Gyroscope random walk (rad/s^2/sqrt(hz))
  double sigma_wb = 1.9393e-05;

  /// Gyroscope random walk covariance
  double sigma_wb_2 = pow(1.9393e-05, 2);

  /// Accelerometer white noise (m/s^2/sqrt(hz))
  double sigma_a = 2.0000e-3;

  /// Accelerometer white noise covariance
  double sigma_a_2 = pow(2.0000e-3, 2);

  /// Accelerometer random walk (m/s^3/sqrt(hz))
  double sigma_ab = 3.0000e-03;

  /// Accelerometer random walk covariance
  double sigma_ab_2 = pow(3.0000e-03, 2);

  /// Nice print function of what parameters we have loaded
  void print() {
    PRINT_DEBUG("  - gyroscope_noise_density: %.6f\n", sigma_w);
    PRINT_DEBUG("  - accelerometer_noise_density: %.5f\n", sigma_a);
    PRINT_DEBUG("  - gyroscope_random_walk: %.7f\n", sigma_wb);
    PRINT_DEBUG("  - accelerometer_random_walk: %.6f\n", sigma_ab);
  }
};

} // namespace vio_msckf

#endif // VIO_MSCKF_NOISEMANAGER_H