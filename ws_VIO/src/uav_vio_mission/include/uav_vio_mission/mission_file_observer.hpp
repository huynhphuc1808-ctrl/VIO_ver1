#pragma once

#include <cerrno>
#include <exception>
#include <optional>
#include <string>
#include <sys/stat.h>
#include <utility>

#include "uav_vio_mission/mission_definition.hpp"

namespace uav_vio_mission {

// The upstream MissionFileMonitor remains the only application trigger. Its
// parser errors have no callback, so this observer only reports changed-file
// validation errors. A valid observation never applies data or clears a prior
// update rejection. Metadata polling avoids reparsing an unchanged file.
class MissionFileErrorObserver {
 public:
  struct Observation {
    std::string error;
  };

  explicit MissionFileErrorObserver(std::string filename) : _filename(std::move(filename)) {}

  std::optional<Observation> observe() {
    const auto signature = signatureOf(_filename);
    if (_last_signature && *_last_signature == signature) return std::nullopt;
    _last_signature = signature;
    try {
      (void)loadMission(_filename);
      return Observation{};
    } catch (const std::exception& exception) {
      return Observation{exception.what()};
    }
  }

 private:
  struct Signature {
    int error{};
    dev_t device{};
    ino_t inode{};
    off_t size{};
    mode_t mode{};
    timespec modified{}, changed{};

    bool operator==(const Signature& other) const {
      return error == other.error && device == other.device && inode == other.inode &&
             size == other.size && mode == other.mode &&
             modified.tv_sec == other.modified.tv_sec && modified.tv_nsec == other.modified.tv_nsec &&
             changed.tv_sec == other.changed.tv_sec && changed.tv_nsec == other.changed.tv_nsec;
    }
  };

  static Signature signatureOf(const std::string& filename) {
    struct stat state {};
    Signature signature;
    if (::stat(filename.c_str(), &state) != 0) {
      signature.error = errno;
      return signature;
    }
    signature.device = state.st_dev;
    signature.inode = state.st_ino;
    signature.size = state.st_size;
    signature.mode = state.st_mode;
    signature.modified = state.st_mtim;
    signature.changed = state.st_ctim;
    return signature;
  }

  std::string _filename;
  std::optional<Signature> _last_signature;
};

}  // namespace uav_vio_mission
