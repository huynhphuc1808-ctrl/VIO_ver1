#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace uav_vio_mission {

inline constexpr double kTakeoffHandoffClimbM = 0.75;
inline constexpr double kTakeoffAltitudeToleranceM = 0.25;

inline double altitudeGotoDeadlineSeconds(double altitude_m)
{
  return std::clamp(45.0 + 8.0 * altitude_m, 60.0, 300.0);
}

inline double orbitDeadlineSeconds(double radius_m, int laps, double speed_m_s)
{
  constexpr double kMinimumSeconds = 60.0;
  constexpr double kMaximumSeconds = 900.0;
  const double expected_seconds = 2.0 * std::acos(-1.0) * radius_m * laps / speed_m_s;
  if (!std::isfinite(expected_seconds) || expected_seconds >= (kMaximumSeconds - 30.0) / 2.0) {
    return kMaximumSeconds;
  }
  return std::max(kMinimumSeconds, 2.0 * expected_seconds + 30.0);
}

inline double holdDeadlineSeconds(double hold_seconds)
{
  return std::min(3600.0, std::max(15.0, hold_seconds + 15.0));
}

struct MissionParameters {
  double radius_m{2.0};
  double altitude_m{2.0};
  double speed_m_s{1.0};
  bool clockwise{true};
  int laps{1};
  double hold_seconds{2.0};
  bool auto_start{false};
  double radial_tolerance_m{0.5};
  double gate_stable_seconds{2.0};

  void validate() const
  {
    if (!std::isfinite(radius_m) || radius_m < 1.0) throw std::invalid_argument("radius_m must be >= 1");
    if (!std::isfinite(altitude_m) || altitude_m < 1.0) throw std::invalid_argument("altitude_m must be >= 1");
    if (!std::isfinite(speed_m_s) || speed_m_s <= 0.0) throw std::invalid_argument("speed_m_s must be > 0");
    if (laps < 1) throw std::invalid_argument("laps must be >= 1");
    if (!std::isfinite(hold_seconds) || hold_seconds < 0.0) throw std::invalid_argument("hold_seconds must be >= 0");
    if (!std::isfinite(radial_tolerance_m) || radial_tolerance_m <= 0.0) throw std::invalid_argument("radial_tolerance_m must be > 0");
    if (!std::isfinite(gate_stable_seconds) || gate_stable_seconds <= 0.0) throw std::invalid_argument("gate_stable_seconds must be > 0");
  }
};

class LapTracker {
 public:
  LapTracker(double center_x, double center_y, double radius, bool clockwise)
      : _center_x(center_x), _center_y(center_y), _radius(radius), _direction(clockwise ? 1.0 : -1.0)
  {
  }

  double update(double x, double y, double radial_tolerance)
  {
    const double dx = x - _center_x;
    const double dy = y - _center_y;
    if (std::abs(std::hypot(dx, dy) - _radius) > radial_tolerance) return _angle;
    const double angle = std::atan2(dy, dx);
    if (!_acquired) {
      _acquired = true;
      _previous = angle;
      return _angle;
    }
    double delta = angle - _previous;
    while (delta > M_PI) delta -= 2.0 * M_PI;
    while (delta < -M_PI) delta += 2.0 * M_PI;
    _angle += _direction * delta;
    _previous = angle;
    return _angle;
  }

  bool acquired() const { return _acquired; }
  double angle() const { return _angle; }

 private:
  double _center_x;
  double _center_y;
  double _radius;
  double _direction;
  bool _acquired{false};
  double _previous{0.0};
  double _angle{0.0};
};

}  // namespace uav_vio_mission
