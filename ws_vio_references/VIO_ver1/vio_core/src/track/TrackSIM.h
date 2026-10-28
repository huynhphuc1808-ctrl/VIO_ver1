#ifndef VIO_CORE_TRACK_SIM_H
#define VIO_CORE_TRACK_SIM_H

#include "TrackBase.h"

namespace vio_core {

/**
 * @brief Simulated tracker for when we already have uv measurements!
 *
 * This class should be used when we are using the @ref vio_msckf::Simulator class to generate measurements.
 * It simply takes the resulting simulation data and appends it to the internal feature database.
 */
class TrackSIM : public TrackBase {

public:
  /**
   * @brief Public constructor with configuration variables
   * @param cameras camera calibration object which has all camera intrinsics in it
   * @param numaruco the max id of the arucotags, so we ensure that we start our non-auroc features above this value
   */
  TrackSIM(std::unordered_map<size_t, std::shared_ptr<CamBase>> cameras, int numaruco)
      : TrackBase(cameras, 0, numaruco, false, HistogramMethod::NONE) {}

  /**
   * @brief Process a new image
   * @warning This function should not be used!! Use @ref feed_measurement_simulation() instead.
   * @param message Contains our timestamp, images, and camera ids
   */
  void feed_new_camera(const CameraData &message) override {
    PRINT_ERROR(RED "[SIM]: SIM TRACKER FEED NEW CAMERA CALLED!!!\n" RESET);
    PRINT_ERROR(RED "[SIM]: THIS SHOULD NEVER HAPPEN!\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  /**
   * @brief Feed function for a synchronized simulated cameras
   * @param timestamp Time that this image was collected
   * @param camids Camera ids that we have simulated measurements for
   * @param feats Raw uv simulated measurements
   */
  void feed_measurement_simulation(double timestamp, const std::vector<int> &camids,
                                   const std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> &feats);
};

} // namespace vio_core

#endif /* VIO_CORE_TRACK_SIM_H */
