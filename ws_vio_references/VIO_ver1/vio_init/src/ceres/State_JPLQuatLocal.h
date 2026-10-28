#ifndef VIO_INIT_CERES_JPLQUATLOCAL_H
#define VIO_INIT_CERES_JPLQUATLOCAL_H

#include <ceres/ceres.h>

#if CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 2
#include <ceres/manifold.h>
#endif

namespace vio_init {

/**
 * @brief JPL quaternion CERES state parameterization
 */
#if CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 2
class State_JPLQuatLocal : public ceres::Manifold {
#else
class State_JPLQuatLocal : public ceres::LocalParameterization {
#endif
public:
  /**
   * @brief State update function for a JPL quaternion representation.
   *
   * Implements update operation by left-multiplying the current
   * quaternion with a quaternion built from a small axis-angle perturbation.
   *
   * @f[
   * \bar{q}=norm\Big(\begin{bmatrix} 0.5*\mathbf{\theta_{dx}} \\ 1 \end{bmatrix}\Big) \hat{\bar{q}}
   * @f]
   */
  bool Plus(const double *x, const double *delta, double *x_plus_delta) const override;

#if CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 2

  bool PlusJacobian(const double *x, double *jacobian) const override;

  // Inverse update: delta = Log(q2 ⊗ inv(q1))
  bool Minus(const double* y, const double* x, double* delta) const override;

  // Jacobian of Minus
  bool MinusJacobian(const double* x, double* jacobian) const override;

  int AmbientSize() const override { return 4; }
  int TangentSize() const override { return 3; }

#else

  /**
   * @brief Computes the jacobian in respect to the local parameterization
   *
   * This essentially "tricks" ceres.
   * Instead of doing what ceres wants:
   * dr/dlocal= dr/dglobal * dglobal/dlocal
   *
   * We instead directly do:
   * dr/dlocal= [ dr/dlocal, 0] * [I; 0]= dr/dlocal.
   * Therefore we here define dglobal/dlocal= [I; 0]
   */
  bool ComputeJacobian(const double *x, double *jacobian) const override;

  int GlobalSize() const override { return 4; };

  int LocalSize() const override { return 3; };

#endif

};

} // namespace vio_init

#endif // VIO_INIT_CERES_JPLQUATLOCAL_H