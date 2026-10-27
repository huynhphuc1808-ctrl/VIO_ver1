#include "uav_vio_mission/mission_definition.hpp"

#include <fstream>
#include <set>
#include <stdexcept>

namespace uav_vio_mission {
namespace {
void keys(const nlohmann::json& value, const std::set<std::string>& allowed)
{
  if (!value.is_object()) throw std::invalid_argument("mission objects must be JSON objects");
  for (auto it = value.begin(); it != value.end(); ++it) {
    if (!allowed.count(it.key())) throw std::invalid_argument("unsupported field: " + it.key());
  }
}
double number(const nlohmann::json& item, const std::string& key, double minimum,
              double maximum, bool exclusive_minimum = false)
{
  if (!item.contains(key) || !item.at(key).is_number()) {
    throw std::invalid_argument(key + " must be a finite number");
  }
  const double value = item.at(key).get<double>();
  if (!std::isfinite(value) || value > maximum || value < minimum ||
      (exclusive_minimum && value == minimum)) {
    throw std::invalid_argument(key + " outside supported range");
  }
  return value;
}
std::string string(const nlohmann::json& item, const std::string& key)
{
  if (!item.contains(key) || !item.at(key).is_string()) {
    throw std::invalid_argument(key + " must be a string");
  }
  return item.at(key).get<std::string>();
}
void altitude(const nlohmann::json& item) { number(item, "altitude_m", 1.0, 30.0); }
}  // namespace

MissionDefinition validateMission(const nlohmann::json& json)
{
  keys(json, {"version", "mission"});
  if (!json.contains("version") || !json.at("version").is_number_integer() ||
      json.at("version") != 1) throw std::invalid_argument("mission version must be integer 1");
  if (!json.contains("mission")) throw std::invalid_argument("mission is required");
  const auto& mission = json.at("mission");
  keys(mission, {"items", "defaults"});
  if (mission.contains("defaults") && !mission.at("defaults").is_null() &&
      (!mission.at("defaults").is_object() || !mission.at("defaults").empty())) {
    throw std::invalid_argument("trajectory defaults are unsupported by local actions");
  }
  if (!mission.contains("items") || !mission.at("items").is_array() ||
      mission.at("items").size() < 2 || mission.at("items").size() > 1024) {
    throw std::invalid_argument("mission must contain 2..1024 action items");
  }
  const auto& items = mission.at("items");
  std::set<std::string> ids;
  std::string previous;
  for (std::size_t index = 0; index < items.size(); ++index) {
    const auto& item = items[index];
    const auto type = string(item, "type");
    if (item.contains("id")) {
      const auto id = string(item, "id");
      if (id.empty() || !ids.insert(id).second) throw std::invalid_argument("item id must be unique and nonempty");
    }
    if ((index == 0) != (type == "localTakeoff")) {
      throw std::invalid_argument("localTakeoff must occur exactly once, as first item");
    }
    if ((index + 1 == items.size()) != (type == "land")) {
      throw std::invalid_argument("land must occur exactly once, as last item");
    }
    if (type == "localTakeoff") {
      keys(item, {"type", "id", "altitude_m"});
      number(item, "altitude_m", 2.0, 30.0);
    } else if (type == "localGoto") {
      keys(item, {"type", "id", "frame", "x_m", "y_m", "z_m", "dwell_s", "max_horizontal_speed_m_s"});
      if (string(item, "frame") != "mission_ned") throw std::invalid_argument("localGoto frame must be mission_ned");
      number(item, "x_m", -1000.0, 1000.0);
      number(item, "y_m", -1000.0, 1000.0);
      number(item, "z_m", -30.0, 0.0);  // Reject ground targets below, including negative zero.
      if (item.at("z_m").get<double>() >= 0.0) throw std::invalid_argument("airborne localGoto z_m must be negative");
      if (item.contains("dwell_s")) number(item, "dwell_s", 0.0, 600.0);
      if (item.contains("max_horizontal_speed_m_s")) number(item, "max_horizontal_speed_m_s", 0.0, 20.0, true);
    } else if (type == "localHold") {
      keys(item, {"type", "id", "duration_s", "altitude_m"});
      number(item, "duration_s", 0.0, 600.0);
      if (item.contains("altitude_m")) altitude(item);
    } else if (type == "orbit") {
      keys(item, {"type", "id", "radius_m", "speed_m_s", "direction", "laps", "radial_tolerance_m"});
      const double radius = number(item, "radius_m", 1.0, 1000.0);
      const double speed = number(item, "speed_m_s", 0.0, 12.0, true);
      const auto direction = string(item, "direction");
      if (direction != "clockwise" && direction != "counterclockwise") throw std::invalid_argument("invalid orbit direction");
      if (!item.contains("laps") || !item.at("laps").is_number_integer()) throw std::invalid_argument("laps must be integer");
      const auto laps = number(item, "laps", 1, 1000);
      number(item, "radial_tolerance_m", 0.0, radius, true);
      // Pinned native Orbit limits centripetal acceleration to 2 m/s^2.
      // Reject a request PX4 necessarily has to silently reduce.
      if (speed * speed > 2.0 * radius) throw std::invalid_argument("orbit exceeds native 2 m/s^2 acceleration limit");
      const double expected_seconds = 2.0 * std::acos(-1.0) * radius * laps / speed;
      if (!std::isfinite(expected_seconds) || expected_seconds > 435.0) {
        throw std::invalid_argument("orbit cannot fit the bounded 900 second deadline");
      }
      // Native Orbit with NaN center samples on activation; a distinct local action
      // guarantees reactivation between repeated orbit invocations.
      if (previous == "orbit") throw std::invalid_argument("adjacent orbit actions require localHold or localGoto handoff");
    } else if (type == "land") {
      keys(item, {"type", "id"});
    } else {
      throw std::invalid_argument("unsupported action: " + type);
    }
    previous = type;
  }
  MissionDefinition result{json, json.get<px4_ros2::Mission>(), {}};
  result.revision = result.mission.checksum();
  return result;
}

MissionDefinition loadMission(const std::string& filename)
{
  std::ifstream file(filename);
  if (!file) throw std::invalid_argument("cannot open mission file: " + filename);
  return validateMission(nlohmann::json::parse(file));
}

nlohmann::json circleMission(const MissionParameters& p)
{
  p.validate();
  nlohmann::json json = {{"version", 1}, {"mission", {{"items", nlohmann::json::array({
    {{"type", "localTakeoff"}, {"altitude_m", p.altitude_m}},
    {{"type", "orbit"}, {"radius_m", p.radius_m}, {"speed_m_s", p.speed_m_s},
     // Positive integer literals are unsigned after JSON text parsing. Match
     // that representation so the upstream type-sensitive checksum survives
     // exporting a generated preset and loading it back from a file.
     {"direction", p.clockwise ? "clockwise" : "counterclockwise"},
     {"laps", static_cast<nlohmann::json::number_unsigned_t>(p.laps)},
     {"radial_tolerance_m", p.radial_tolerance_m}},
    {{"type", "localHold"}, {"duration_s", p.hold_seconds}, {"altitude_m", p.altitude_m}},
    {{"type", "land"}}
  })}}}};
  return json;
}
}  // namespace uav_vio_mission
