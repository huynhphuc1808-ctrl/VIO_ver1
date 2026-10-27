#pragma once

#include <cstdint>
#include <cmath>
#include <optional>

namespace uav_vio_mission {

// Uses monotonic receive time, never simulator time. Observe before replacing
// a sample so a gap cannot disappear when a late fresh sample arrives.
class StableReadiness {
 public:
  explicit StableReadiness(double stable_seconds) : _stable_seconds(stable_seconds) {}
  void vio(double now, bool valid) { observe(now); _vio = now; _vio_valid = valid; observe(now); }
  void local(double now, bool valid) { observe(now); _local = now; _local_valid = valid; observe(now); }
  void observe(double now) {
    if (!fresh(now)) _healthy_since.reset();
    else if (!_healthy_since) _healthy_since = now;
  }
  bool fresh(double now) const { return vioFresh(now) && localFresh(now); }
  bool vioFresh(double now) const { return sampleFresh(_vio, now) && _vio_valid; }
  bool localFresh(double now) const { return sampleFresh(_local, now) && _local_valid; }
  bool ready(double now) const {
    return fresh(now) && _healthy_since && now - *_healthy_since >= _stable_seconds;
  }
 private:
  static bool sampleFresh(const std::optional<double>& time, double now) {
    return time && now >= *time && now - *time < 0.5;
  }
  double _stable_seconds;
  std::optional<double> _vio, _local, _healthy_since;
  bool _vio_valid{false}, _local_valid{false};
};

struct FrameEpoch {
  uint8_t xy{}, z{}, vxy{}, vz{}, heading{};
  uint64_t reference{};
  bool operator==(const FrameEpoch& other) const {
    return xy == other.xy && z == other.z && vxy == other.vxy && vz == other.vz &&
           heading == other.heading && reference == other.reference;
  }
  bool operator!=(const FrameEpoch& other) const { return !(*this == other); }
};

// Initial magnetic alignment can rotate/reset local state while climbing.
// Only this bounded startup phase may rebase its temporary hold target. The
// final mission frame is immutable and never uses this path after freeze.
class InitialFrameState {
 public:
  enum class Update { Unchanged, Retarget, Invalid };
  void capture(const FrameEpoch& epoch, double ground_z, double clock_now) {
    _epoch = epoch; _ground_z = ground_z; _stable_since = clock_now; _captured = true;
  }
  Update observe(const FrameEpoch& epoch, double delta_z, double clock_now) {
    if (!_captured || clock_now < _stable_since || epoch.reference != _epoch.reference) return Update::Invalid;
    if (epoch == _epoch) return Update::Unchanged;
    const auto contiguous = [](uint8_t old_value, uint8_t new_value) {
      return old_value == new_value || static_cast<uint8_t>(old_value + 1) == new_value;
    };
    if (!contiguous(_epoch.xy, epoch.xy) || !contiguous(_epoch.z, epoch.z) ||
        !contiguous(_epoch.vxy, epoch.vxy) || !contiguous(_epoch.vz, epoch.vz) ||
        !contiguous(_epoch.heading, epoch.heading)) return Update::Invalid;
    if (epoch.z != _epoch.z) {
      if (!std::isfinite(delta_z)) return Update::Invalid;
      _ground_z += delta_z;
    }
    _epoch = epoch; _stable_since = clock_now;
    return Update::Retarget;
  }
  bool stable(double clock_now) const { return _captured && clock_now - _stable_since >= 3.0; }
  double groundZ() const { return _ground_z; }
 private:
  FrameEpoch _epoch;
  double _ground_z{}, _stable_since{};
  bool _captured{false};
};

// The upstream handler is valid across successive actions. This token is scoped
// to one invocation, so a timer/callback from an older action cannot advance it.
class InvocationGuard {
 public:
  uint64_t begin() { _running = true; return ++_generation; }
  bool valid(uint64_t token) const { return _running && token == _generation; }
  bool finish(uint64_t token) {
    if (!valid(token)) return false;
    _running = false;
    return true;
  }
  void cancel() { _running = false; ++_generation; }
 private:
  uint64_t _generation{0};
  bool _running{false};
};

inline bool canReplaceMission(bool active, bool landed, bool armed, bool fresh_vehicle_state) {
  return !active && landed && !armed && fresh_vehicle_state;
}
inline bool ownedModeReselectedWhileArmed(bool incoming_armed, bool seen_flight_mode,
                                        bool incoming_owned_mode) {
  // PX4 restores the executor's owned mode on disarm. Only an armed return to
  // that mode is a flight reselection; callback ordering may still show active.
  return incoming_armed && seen_flight_mode && incoming_owned_mode;
}
inline bool terminalLandingSuccess(bool run_started, bool final_land, bool failed,
                                   bool interrupted, bool landed, bool armed, bool state_fresh) {
  return run_started && final_land && !failed && !interrupted && state_fresh && landed && !armed;
}
}  // namespace uav_vio_mission
