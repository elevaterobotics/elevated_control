// Copyright (c) 2025 Elevate Robotics Inc
#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <map>
#include <string>
#include <string_view>
#include <vector>

namespace config_verification {

// Key used in the reference map: (index << 8) | subindex
using SdoKey = uint32_t;
constexpr SdoKey MakeKey(uint16_t index, uint8_t subindex) {
  return (static_cast<uint32_t>(index) << 8) | subindex;
}

using RefMap = std::map<SdoKey, int32_t>;

// SDO address ranges to read for configuration verification.
// Only includes indices that are configuration (set at init), not runtime.
// Format: {start_index, end_index} (inclusive)
inline const std::vector<std::pair<uint16_t, uint16_t>> kAddressRanges = {
  { 0x2001, 0x2006 },
  { 0x2011, 0x2012 },
  { 0x6070, 0x6070 },
  { 0x6072, 0x6073 },
  { 0x6075, 0x6076 },
  { 0x607B, 0x607E },
  { 0x6080, 0x6081 },
  { 0x6091, 0x6092 }
};

inline constexpr uint8_t kMaxSubindex = 32;

inline constexpr uint16_t kFwVersionIndex    = 0x100A;
inline constexpr uint8_t  kFwVersionSubindex = 0x00;

inline constexpr std::string_view kExpectedFwPrefix = "5.6";

// Map EtherCAT slave index (1-based) to joint name / CSV file stem.
inline const std::array<std::string, 7> kJointCsvStems = {
  "yaw1",
  "yaw2",
  "SAA",
  "inertial",
  "wrist_yaw",
  "wrist_pitch",
  "wrist_roll",
};

// ---------------------------------------------------------------------------
// Parsed command-line arguments.
// ---------------------------------------------------------------------------
struct Args {
  std::string iface = "eno0";
  std::filesystem::path ref_dir;
  bool help = false;
};

Args ParseArgs(int argc, char** argv);

// ---------------------------------------------------------------------------
// Parse a P3S2 CSV reference file.
// Lines starting with '#' or 'META' are ignored.
// Format: 0xINDEX, SUBINDEX, VALUE
// ---------------------------------------------------------------------------
RefMap ParseRefCsv(const std::filesystem::path& path);

// ---------------------------------------------------------------------------
// Check whether a firmware version string contains the expected prefix.
// ---------------------------------------------------------------------------
bool CheckFirmwareVersion(const std::string& fw_version,
                          std::string_view expected_prefix);

// ---------------------------------------------------------------------------
// Result of comparing a single live register value against a reference.
// ---------------------------------------------------------------------------
struct CompareResult {
  enum Status { kMatch, kMismatch, kNoRef };
  Status status;
  int32_t expected = 0;
};

CompareResult CompareRegister(const RefMap& ref, uint16_t index,
                              uint8_t subindex, int32_t live_value);

}  // namespace config_verification
