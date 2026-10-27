#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "uav_vio_mission/mission_file_observer.hpp"

namespace {
constexpr const char* kValidMission = R"({"version":1,"mission":{"items":[{"type":"localTakeoff","altitude_m":2.0},{"type":"land"}]}})";

class MissionFileObserverTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const auto pattern = (std::filesystem::temp_directory_path() / "vio_mission_observer_XXXXXX").string();
    std::vector<char> buffer(pattern.begin(), pattern.end());
    buffer.push_back('\0');
    const auto created = ::mkdtemp(buffer.data());
    ASSERT_NE(created, nullptr);
    directory = created;
    filename = directory / "mission.json";
  }

  void TearDown() override {
    if (!directory.empty()) {
      std::error_code error;
      std::filesystem::remove_all(directory, error);
    }
  }

  void write(const std::string& contents) {
    std::ofstream stream(filename);
    ASSERT_TRUE(stream.is_open());
    stream << contents;
    stream.close();
    ASSERT_TRUE(stream.good());
  }

  std::filesystem::path directory, filename;
};

TEST_F(MissionFileObserverTest, ValidAndMalformedChangesAreObservedOnlyOnce) {
  write(kValidMission);
  uav_vio_mission::MissionFileErrorObserver observer(filename.string());
  const auto initial = observer.observe();
  ASSERT_TRUE(initial);
  EXPECT_TRUE(initial->error.empty());
  EXPECT_FALSE(observer.observe());

  write("{");
  const auto malformed = observer.observe();
  ASSERT_TRUE(malformed);
  EXPECT_FALSE(malformed->error.empty());
  EXPECT_FALSE(observer.observe());

  write(kValidMission);
  const auto recovered = observer.observe();
  ASSERT_TRUE(recovered);
  EXPECT_TRUE(recovered->error.empty());
  EXPECT_FALSE(observer.observe());
}

TEST_F(MissionFileObserverTest, StrictValidationReportsVersionAndUnknownActionErrors) {
  write(R"({"version":2,"mission":{"items":[]}})");
  uav_vio_mission::MissionFileErrorObserver observer(filename.string());
  const auto wrong_version = observer.observe();
  ASSERT_TRUE(wrong_version);
  EXPECT_NE(wrong_version->error.find("mission version"), std::string::npos);

  write(R"({"version":1,"mission":{"items":[{"type":"localTakeoff","altitude_m":2.0},{"type":"unknown"},{"type":"land"}]}})");
  const auto unknown_action = observer.observe();
  ASSERT_TRUE(unknown_action);
  EXPECT_NE(unknown_action->error.find("unsupported action"), std::string::npos);
  EXPECT_FALSE(observer.observe());
}

TEST_F(MissionFileObserverTest, MissingFileAndUnreadableSymlinkAreReportedWithoutApplication) {
  uav_vio_mission::MissionFileErrorObserver observer(filename.string());
  const auto missing = observer.observe();
  ASSERT_TRUE(missing);
  EXPECT_NE(missing->error.find("cannot open mission file"), std::string::npos);
  EXPECT_FALSE(observer.observe());

  // A symlink loop produces a deterministic read error even when tests run as root.
  std::filesystem::create_symlink(filename.filename(), filename);
  const auto unreadable = observer.observe();
  ASSERT_TRUE(unreadable);
  EXPECT_NE(unreadable->error.find("cannot open mission file"), std::string::npos);
  EXPECT_FALSE(observer.observe());

  std::filesystem::remove(filename);
  write(kValidMission);
  const auto recovered = observer.observe();
  ASSERT_TRUE(recovered);
  EXPECT_TRUE(recovered->error.empty());

  std::filesystem::remove(filename);
  const auto removed = observer.observe();
  ASSERT_TRUE(removed);
  EXPECT_FALSE(removed->error.empty());
}

TEST_F(MissionFileObserverTest, AtomicReplacementWithPreservedSizeAndMtimeIsDetected) {
  write(kValidMission);
  uav_vio_mission::MissionFileErrorObserver observer(filename.string());
  ASSERT_TRUE(observer.observe());
  const auto prior_time = std::filesystem::last_write_time(filename);
  const auto prior_size = std::filesystem::file_size(filename);
  const auto replacement = directory / "replacement.json";
  std::string content = kValidMission;
  content.replace(content.find("2.0"), 3, "3.0");
  {
    std::ofstream stream(replacement);
    ASSERT_TRUE(stream.is_open());
    stream << content;
  }
  ASSERT_EQ(std::filesystem::file_size(replacement), prior_size);
  std::filesystem::last_write_time(replacement, prior_time);
  std::filesystem::rename(replacement, filename);
  const auto changed = observer.observe();
  ASSERT_TRUE(changed);
  EXPECT_TRUE(changed->error.empty());
  EXPECT_FALSE(observer.observe());
}

TEST_F(MissionFileObserverTest, PermissionChangeIsIncludedInTheSignature) {
  write(kValidMission);
  uav_vio_mission::MissionFileErrorObserver observer(filename.string());
  ASSERT_TRUE(observer.observe());
  const auto permissions = std::filesystem::status(filename).permissions();
  const auto owner_exec = std::filesystem::perms::owner_exec;
  std::filesystem::permissions(filename, permissions ^ owner_exec);
  const auto changed = observer.observe();
  ASSERT_TRUE(changed);
  EXPECT_TRUE(changed->error.empty());
  EXPECT_FALSE(observer.observe());
}

}  // namespace
