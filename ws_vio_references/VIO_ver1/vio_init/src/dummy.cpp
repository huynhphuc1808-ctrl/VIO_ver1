/**
 * @namespace vio_init
 * @brief State initialization code
 *
 * Right now this contains StaticInitializer and DynamicInitializer initialization code for a visual-inertial system.
 * It will wait for the platform to stationary, and then initialize its orientation in the gravity frame.
 *
 * - https://pgeneva.com/downloads/reports/tr_init.pdf
 * - https://ieeexplore.ieee.org/abstract/document/6386235
 * - https://tdongsi.github.io/download/pubs/2011_VIO_Init_TR.pdf
 *
 * If the platform is not stationary then we leverage dynamic initialization to try to recover the initial state.
 * This is an implementation of the work [Estimator initialization in vision-aided inertial navigation with unknown camera-IMU
 * calibration](https://ieeexplore.ieee.org/document/6386235) @cite Dong2012IROS which solves the initialization problem by first creating a
 * linear system for recovering tthe velocity, gravity, and feature positions.
 * After the initial recovery, a full optimization is performed to allow for covariance recovery.
 * See this [tech report](https://pgeneva.com/downloads/reports/tr_init.pdf) for a high level walk through.
 */
namespace vio_init {}
