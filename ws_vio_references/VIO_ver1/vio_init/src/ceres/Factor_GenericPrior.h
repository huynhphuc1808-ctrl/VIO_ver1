#ifndef VIO_INIT_CERES_GENERICPRIOR_H
#define VIO_INIT_CERES_GENERICPRIOR_H

#include <ceres/ceres.h>

namespace vio_init {

/**
 * @brief Factor for generic state priors for specific types.
 *
 * This is a general factor which handles state priors which have non-zero linear errors.
 * In general a unitary factor will have zero error when it is created, thus this extra term can be ignored.
 * But if performing marginalization, this can be non-zero. See the following paper Section 3.2 Eq. 25-35
 * https://journals.sagepub.com/doi/full/10.1177/0278364919835021
 *
 * We have the following minimization problem:
 * @f[
 * \textrm{argmin} ||A * (x - x_{lin}) + b||^2
 * @f]
 *
 *
 * In general we have the following after marginalization:
 * - @f$(A^T*A) = Inf_{prior} @f$ (the prior information)
 * - @f$A^T*b = grad_{prior} @f$ (the prior gradient)
 *
 * For example, consider we have the following system were we wish to remove the xm states.
 * This is the problem of state marginalization.
 * @f[
 * [ Arr Arm ] [ xr ] = [ - gr ]
 * @f]
 * @f[
 * [ Amr Amm ] [ xm ] = [ - gm ]
 * @f]
 *
 * We wish to marginalize the xm states which are correlated with the other states @f$ xr @f$.
 * The Jacobian (and thus information matrix A) is computed at the current best guess @f$ x_{lin} @f$.
 * We can define the following optimal subcost form which only involves the @f$ xr @f$ states as:
 * @f[
 * cost^2 = (xr - xr_{lin})^T*(A^T*A)*(xr - xr_{lin}) + b^T*A*(xr - xr_{lin}) + b^b
 * @f]
 *
 * where we have:
 * @f[
 * A = sqrt(Arr - Arm*Amm^{-1}*Amr)
 * @f]
 * @f[
 * b = A^-1 * (gr - Arm*Amm^{-1}*gm)
 * @f]
 *
 */
class Factor_GenericPrior : public ceres::CostFunction {
public:
  /// State estimates at the time of marginalization to linearize the problem
  Eigen::MatrixXd x_lin;

  /// State type for each variable in x_lin. Can be [quat, quat_yaw, vec3, vec8]
  std::vector<std::string> x_type;

  /// The square-root of the information s.t. sqrtI^T * sqrtI = marginal information
  Eigen::MatrixXd sqrtI;

  /// Constant term inside the cost s.t. sqrtI^T * b = marginal gradient (can be zero)
  Eigen::MatrixXd b;

  /**
   * @brief Default constructor
   */
  Factor_GenericPrior(const Eigen::MatrixXd &x_lin_, const std::vector<std::string> &x_type_, const Eigen::MatrixXd &prior_Info,
                      const Eigen::MatrixXd &prior_grad);

  virtual ~Factor_GenericPrior() {}

  /**
   * @brief Error residual and Jacobian calculation
   */
  bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override;
};

} // namespace vio_init

#endif // VIO_INIT_CERES_GENERICPRIOR_H