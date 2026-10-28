#ifndef VIO_TYPE_TYPE_VEC_H
#define VIO_TYPE_TYPE_VEC_H

#include "Type.h"

namespace vio_type {

/**
 * @brief Derived Type class that implements vector variables
 */
class Vec : public Type {

public:
  /**
   * @brief Default constructor for Vec
   * @param dim Size of the vector (will be same as error state)
   */
  Vec(int dim) : Type(dim) {
    _value = Eigen::VectorXd::Zero(dim);
    _fej = Eigen::VectorXd::Zero(dim);
  }

  ~Vec() {}

  /**
   * @brief Implements the update operation through standard vector addition
   *
   * \f{align*}{
   * \mathbf{v} &= \hat{\mathbf{v}} + \tilde{\mathbf{v}}_{dx}
   * \f}
   *
   * @param dx Additive error state correction
   */
  void update(const Eigen::VectorXd &dx) override {
    assert(dx.rows() == _size);
    set_value(_value + dx);
  }

  /**
   * @brief Performs all the cloning
   */
  std::shared_ptr<Type> clone() override {
    auto Clone = std::shared_ptr<Type>(new Vec(_size));
    Clone->set_value(value());
    Clone->set_fej(fej());
    return Clone;
  }
};

} // namespace vio_type

#endif // VIO_TYPE_TYPE_VEC_H