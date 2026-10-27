#include <gtest/gtest.h>
#include <limits>
#include "uav_vio_mission/mission_definition.hpp"

using namespace uav_vio_mission;
using nlohmann::json;
namespace {
json fixture() { return circleMission(MissionParameters{}); }
json localGoto() {
  return {{"type", "localGoto"}, {"frame", "mission_ned"}, {"x_m", 2.0}, {"y_m", 0.0}, {"z_m", -2.0}};
}
}  // namespace
TEST(MissionDefinition, GeneratedCircleUsesUpstreamActionsAndPreservesParameters)
{
  MissionParameters p;
  p.clockwise = false; p.radius_m = 3.0; p.speed_m_s = 0.8; p.laps = 2; p.altitude_m = 2.5;
  const auto result = validateMission(circleMission(p));
  ASSERT_EQ(result.mission.items().size(), 4U);
  EXPECT_FALSE(result.revision.empty());
  const auto& takeoff = std::get<px4_ros2::ActionItem>(result.mission.items()[0]);
  EXPECT_EQ(takeoff.name, "localTakeoff");
  EXPECT_DOUBLE_EQ(takeoff.arguments.at<double>("altitude_m"), 2.5);
  const auto& orbit = std::get<px4_ros2::ActionItem>(result.mission.items()[1]);
  EXPECT_EQ(orbit.arguments.at<std::string>("direction"), "counterclockwise");
  EXPECT_EQ(orbit.arguments.at<int>("laps"), 2);
  const auto roundtrip = validateMission(json(result.mission));
  EXPECT_EQ(roundtrip.revision, result.revision);
}
TEST(MissionDefinition, CircleTextExportKeepsUpstreamRevision)
{
  const auto generated = circleMission(MissionParameters{});
  const auto exported = json::parse(generated.dump());
  ASSERT_EQ(generated, exported);
  // API1 hashes JSON number types too. Saving and loading the same flight
  // intent must not produce a different revision from the parameter preset.
  EXPECT_EQ(validateMission(generated).revision, validateMission(exported).revision);
}
TEST(MissionDefinition, AllowsRepeatedLocalActionsAndOrbitWithExplicitHandoff)
{
  auto j = fixture();
  auto& items = j["mission"]["items"];
  items.insert(items.begin() + 1, localGoto());
  items.insert(items.begin() + 2, localGoto());
  const auto orbit = items[3];
  items.insert(items.begin() + 5, orbit);
  EXPECT_NO_THROW(validateMission(j));
  auto no_handoff = fixture();
  auto& other = no_handoff["mission"]["items"];
  const auto adjacent_orbit = other[1];
  other.insert(other.begin() + 2, adjacent_orbit);
  EXPECT_THROW(validateMission(no_handoff), std::invalid_argument);
}
TEST(MissionDefinition, RejectsUnknownGlobalAndLibraryInternalFields)
{
  for (const auto& type : {"waypoint", "takeoff", "hold", "rtl", "changeSettings", "onFailure", "survey"}) {
    auto j = fixture();
    j["mission"]["items"][1]["type"] = type;
    EXPECT_THROW(validateMission(j), std::invalid_argument) << type;
  }
  for (const auto& field : {"internal:resuming", "radius", "unused"}) {
    auto j = fixture(); j["mission"]["items"][1][field] = 1;
    EXPECT_THROW(validateMission(j), std::invalid_argument) << field;
  }
  auto j = fixture(); j["mission"]["defaults"] = {{"horizontalVelocity", 1.0}};
  EXPECT_THROW(validateMission(j), std::invalid_argument);
}
TEST(MissionDefinition, RejectsWrongTypesNonfiniteAndUnsafeBounds)
{
  const std::vector<json> invalid = {nullptr, "2.0", true, -1, 0, 0.9,
      std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN(), 1001};
  for (const auto& value : invalid) {
    auto j = fixture(); j["mission"]["items"][1]["radius_m"] = value;
    EXPECT_THROW(validateMission(j), std::invalid_argument) << value.dump();
  }
  auto j = fixture(); j["mission"]["items"][1]["laps"] = 1.5;
  EXPECT_THROW(validateMission(j), std::invalid_argument);
  j = fixture(); j["mission"]["items"][1]["speed_m_s"] = 3.0;
  EXPECT_THROW(validateMission(j), std::invalid_argument);  // native acceleration clamp
  j = fixture(); j["mission"]["items"][1]["laps"] = 1000;
  EXPECT_THROW(validateMission(j), std::invalid_argument);  // cannot fit 900-second deadline
  j = fixture(); j["version"] = 1.0;
  EXPECT_THROW(validateMission(j), std::invalid_argument);
}
TEST(MissionDefinition, RejectsInvalidLocalFramesGroundTargetsAndDuplicateIds)
{
  auto j = fixture();
  auto& items = j["mission"]["items"];
  items.insert(items.begin() + 1, localGoto());
  items[1]["frame"] = "vio_enu";
  EXPECT_THROW(validateMission(j), std::invalid_argument);
  items[1]["frame"] = "mission_ned";
  for (const auto& z : {0.0, 1.0, -31.0}) {
    items[1]["z_m"] = z;
    EXPECT_THROW(validateMission(j), std::invalid_argument);
  }
  items[1]["z_m"] = -2.0;
  items[0]["id"] = "duplicate"; items[1]["id"] = "duplicate";
  EXPECT_THROW(validateMission(j), std::invalid_argument);
}
TEST(MissionDefinition, EnforcesSingleFirstTakeoffAndTerminalLand)
{
  auto j = fixture(); j["mission"]["items"].erase(0);
  EXPECT_THROW(validateMission(j), std::invalid_argument);
  j = fixture(); j["mission"]["items"].erase(3);
  EXPECT_THROW(validateMission(j), std::invalid_argument);
  j = fixture();
  const auto extra = j["mission"]["items"][0];
  j["mission"]["items"].insert(j["mission"]["items"].begin() + 2, extra);
  EXPECT_THROW(validateMission(j), std::invalid_argument);
  j = fixture(); j["mission"]["items"][1] = {{"type", "land"}};
  EXPECT_THROW(validateMission(j), std::invalid_argument);
  EXPECT_THROW(loadMission("/nonexistent/vio-mission.json"), std::invalid_argument);
}
