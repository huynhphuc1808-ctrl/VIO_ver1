#ifndef VIO_CORE_OPENCV_LAMBDA_BODY_H
#define VIO_CORE_OPENCV_LAMBDA_BODY_H

#include <functional>
#include <opencv2/opencv.hpp>

namespace vio_core {

/*
 * @brief Helper class to do OpenCV parallelization
 *
 * This is a utility class required to build with older version of opencv
 * On newer versions this doesn't seem to be needed, but here we just use it to ensure we can work for more opencv version.
 * https://answers.opencv.org/question/65800/how-to-use-lambda-as-a-parameter-to-parallel_for_/?answer=130691#post-id-130691
 */
class LambdaBody : public cv::ParallelLoopBody {
public:
  explicit LambdaBody(const std::function<void(const cv::Range &)> &body) { _body = body; }
  void operator()(const cv::Range &range) const override { _body(range); }

private:
  std::function<void(const cv::Range &)> _body;
};

} /* namespace vio_core */

#endif /* VIO_CORE_OPENCV_LAMBDA_BODY_H */
