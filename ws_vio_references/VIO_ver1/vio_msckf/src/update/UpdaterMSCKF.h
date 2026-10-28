#ifndef VIO_MSCKF_UPDATER_MSCKF_H
#define VIO_MSCKF_UPDATER_MSCKF_H

#include <Eigen/Eigen>
#include <memory>

#include "feat/FeatureInitializerOptions.h"

#include "UpdaterOptions.h"

namespace vio_core {
class Feature;
class FeatureInitializer;
} // namespace vio_core

namespace vio_msckf {

class State;

/**
 * @brief Will compute the system for our sparse features and update the filter.
 *
 * This class is responsible for computing the entire linear system for all features that are going to be used in an update.
 * This follows the original MSCKF, where we first triangulate features, we then nullspace project the feature Jacobian.
 * After this we compress all the measurements to have an efficient update and update the state.
 */
class UpdaterMSCKF {

public:
  /**
   * @brief Default constructor for our MSCKF updater
   *
   * Our updater has a feature initializer which we use to initialize features as needed.
   * Also the options allow for one to tune the different parameters for update.
   *
   * @param options Updater options (include measurement noise value)
   * @param feat_init_options Feature initializer options
   */
  UpdaterMSCKF(UpdaterOptions &options, vio_core::FeatureInitializerOptions &feat_init_options);

  /**
   * @brief Given tracked features, this will try to use them to update the state.
   *
   * @param state State of the filter
   * @param feature_vec Features that can be used for update
   */
  void update(std::shared_ptr<State> state, std::vector<std::shared_ptr<vio_core::Feature>> &feature_vec);

protected:
  /// Options used during update
  UpdaterOptions _options;

  /// Feature initializer class object
  std::shared_ptr<vio_core::FeatureInitializer> initializer_feat;

  /// Chi squared 95th percentile table (lookup would be size of residual)
  std::map<int, double> chi_squared_table;
};

} // namespace vio_msckf

#endif // VIO_MSCKF_UPDATER_MSCKF_H
