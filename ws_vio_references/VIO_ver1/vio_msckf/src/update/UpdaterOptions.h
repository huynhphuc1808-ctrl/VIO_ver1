#ifndef VIO_MSCKF_UPDATER_OPTIONS_H
#define VIO_MSCKF_UPDATER_OPTIONS_H

#include "utils/print.h"

namespace vio_msckf {

/**
 * @brief Struct which stores general updater options
 */
struct UpdaterOptions {

  /// What chi-squared multipler we should apply
  double chi2_multipler = 5;

  /// Noise sigma for our raw pixel measurements
  double sigma_pix = 1;

  /// Covariance for our raw pixel measurements
  double sigma_pix_sq = 1;

  /// Nice print function of what parameters we have loaded
  void print() {
    PRINT_DEBUG("    - chi2_multipler: %.1f\n", chi2_multipler);
    PRINT_DEBUG("    - sigma_pix: %.2f\n", sigma_pix);
  }
};

} // namespace vio_msckf

#endif // VIO_MSCKF_UPDATER_OPTIONS_H