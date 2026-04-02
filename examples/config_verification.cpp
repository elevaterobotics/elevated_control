// Copyright (c) 2025 Elevate Robotics Inc
//
// config_verification: Reads SDO configuration registers from all Synapticon
// drives on the EtherCAT bus, optionally compares them against known-good CSV
// reference files (e.g. from aladdin_config/synapticon_config/P3S2/), and
// reports any mismatches.
//
// Usage:
//   sudo ./config_verification [--ref-dir /path/to/P3S2] [--iface eno0]
//
// Without --ref-dir the tool prints the live values for human inspection.
// With    --ref-dir it prints a pass/fail comparison per joint.

#include "elevated_control/state_machine.hpp"
#include "example_config_dir.hpp"

#include <soem/ethercat.h>

#include <algorithm>
#include <charconv>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace fs = std::filesystem;
using elevated_control::ReadSDOString;
using elevated_control::ReadSDOValue;

// ---------------------------------------------------------------------------
// SDO address ranges to read for configuration verification.
// Only includes indices that are configuration (set at init), not runtime
// (e.g. controlword/statusword).
// Format: {start_index, end_index} (inclusive)
// ---------------------------------------------------------------------------
const std::vector<std::pair<uint16_t, uint16_t>> kAddressRanges = {
  { 0x2001, 0x2006 },  // Comm. offset, various others
  { 0x2011, 0x2012 },  // Velocity control config, position control config
  { 0x6070, 0x6070 },  // Velocity threshold time
  { 0x6072, 0x6073 },  // Max torque, max current
  { 0x6075, 0x6076 },  // Motor rated current, torque
  { 0x607B, 0x607E },  // Position range limit, home offset, pos limit, polarity
  { 0x6080, 0x6081 },  // Max motor speed, profile velocity
  { 0x6091, 0x6092 }   // Gear ratio, feed constant
};

// Maximum subindex to probe when the object has sub-entries.
// We'll walk 0x00 first; if it returns a count we use that, otherwise we
// probe up to kMaxSubindex.
static constexpr uint8_t kMaxSubindex = 32;

// SDO index for reading the firmware (Software Version) string.
static constexpr uint16_t kFwVersionIndex    = 0x100A;
static constexpr uint8_t  kFwVersionSubindex = 0x00;

// Expected firmware version prefix (e.g. "5.6").
static constexpr std::string_view kExpectedFwPrefix = "5.6";

// ---------------------------------------------------------------------------
// Key used in the reference map: (index << 8) | subindex  →  int32 value
// ---------------------------------------------------------------------------
using SdoKey = uint32_t;
constexpr SdoKey MakeKey(uint16_t index, uint8_t subindex) {
  return (static_cast<uint32_t>(index) << 8) | subindex;
}

// ---------------------------------------------------------------------------
// Parse a P3S2 CSV reference file.
// Format (after META header lines):
//   0xINDEX,  SUBINDEX,  VALUE
// Lines starting with '#' or 'META' are ignored.
// ---------------------------------------------------------------------------
using RefMap = std::map<SdoKey, int32_t>;

static RefMap ParseRefCsv(const fs::path& path) {
  RefMap result;
  std::ifstream f(path);
  if (!f.is_open()) {
    std::cerr << "  [warn] Cannot open reference file: " << path << "\n";
    return result;
  }
  std::string line;
  while (std::getline(f, line)) {
    // Strip leading whitespace
    auto start = line.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) continue;
    line = line.substr(start);
    if (line.empty() || line[0] == '#') continue;
    if (line.rfind("META", 0) == 0)    continue;

    // Split on commas
    std::istringstream ss(line);
    std::string tok_index, tok_sub, tok_val;
    if (!std::getline(ss, tok_index, ',')) continue;
    if (!std::getline(ss, tok_sub,   ',')) continue;
    if (!std::getline(ss, tok_val,   ',')) continue;

    // Trim whitespace
    auto trim = [](std::string& s) {
      auto b = s.find_first_not_of(" \t");
      auto e = s.find_last_not_of(" \t\r\n");
      s = (b == std::string::npos) ? "" : s.substr(b, e - b + 1);
    };
    trim(tok_index); trim(tok_sub); trim(tok_val);

    // Parse index (hex)
    uint16_t index = 0;
    if (tok_index.size() > 2 && tok_index[0] == '0' &&
        (tok_index[1] == 'x' || tok_index[1] == 'X')) {
      index = static_cast<uint16_t>(std::stoul(tok_index, nullptr, 16));
    } else {
      continue;
    }

    // Parse subindex (decimal)
    uint8_t subindex = 0;
    {
      auto r = std::from_chars(tok_sub.data(),
                               tok_sub.data() + tok_sub.size(), subindex);
      if (r.ec != std::errc{}) continue;
    }

    // Parse value — may be float in the CSV; we truncate to int32 to match
    // what ec_SDOread returns for integer objects.  Float-typed objects
    // (e.g. 0x2011) are stored as fixed-point internally; the drive returns
    // an int32 bit-pattern, so we cast the parsed double to int32.
    int32_t value = 0;
    try {
      double d = std::stod(tok_val);
      value = static_cast<int32_t>(d);
    } catch (...) {
      continue;
    }

    result[MakeKey(index, subindex)] = value;
  }
  return result;
}

// ---------------------------------------------------------------------------
// Map EtherCAT slave index (1-based) → joint name / CSV file stem.
// Matches kJointNames order from types.hpp (0-based bus order).
// ---------------------------------------------------------------------------
static const std::array<std::string, 7> kJointCsvStems = {
  "yaw1",           // slave 1
  "yaw2",           // slave 2
  "SAA",            // slave 3  (spring adjust)
  "inertial",       // slave 4
  "wrist_yaw",      // slave 5
  "wrist_pitch",    // slave 6
  "wrist_roll",     // slave 7
};

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
  std::string iface = "eno0";
  std::optional<fs::path> ref_dir;

  for (int i = 1; i < argc; ++i) {
    std::string_view arg(argv[i]);
    if ((arg == "--iface" || arg == "-i") && i + 1 < argc) {
      iface = argv[++i];
    } else if ((arg == "--ref-dir" || arg == "-r") && i + 1 < argc) {
      ref_dir = fs::path(argv[++i]);
    } else if (arg == "--help" || arg == "-h") {
      std::cout
          << "Usage: " << argv[0]
          << " [--iface <nic>] [--ref-dir <path/to/P3S2>]\n"
          << "\n"
          << "  --iface   Network interface (default: eno0)\n"
          << "  --ref-dir Path to folder containing per-joint CSV reference\n"
          << "            files (e.g. yaw1.csv, SAA.csv …).  When supplied,\n"
          << "            live values are compared against the reference and\n"
          << "            mismatches are flagged.  Without this flag the tool\n"
          << "            just prints live values.\n";
      return EXIT_SUCCESS;
    }
  }

  // -------------------------------------------------------------------------
  // 1. Initialise EtherCAT (Pre-Op only — no PDO mapping needed)
  // -------------------------------------------------------------------------
  std::cout << "Initialising EtherCAT on interface '" << iface << "' ...\n";
  if (ec_init(iface.c_str()) <= 0) {
    std::cerr << "ec_init failed. Are you root and is '" << iface
              << "' correct?\n";
    return EXIT_FAILURE;
  }

  if (ec_config_init(FALSE) <= 0) {
    std::cerr << "ec_config_init: no slaves found.\n";
    ec_close();
    return EXIT_FAILURE;
  }

  const int slave_count = ec_slavecount;
  std::cout << "Found " << slave_count << " slave(s).\n\n";

  // -------------------------------------------------------------------------
  // 2. For every slave: check FW version then read config SDOs
  // -------------------------------------------------------------------------
  int total_mismatches = 0;
  int total_checked    = 0;
  bool all_fw_ok       = true;

  for (int slave = 1; slave <= slave_count; ++slave) {
    const std::string slave_name(ec_slave[slave].name);
    std::cout << "══════════════════════════════════════════════════════\n";
    std::cout << "Slave " << slave << "  (" << slave_name << ")\n";
    std::cout << "══════════════════════════════════════════════════════\n";

    // -- Firmware version check --
    auto fw = ReadSDOString(static_cast<uint16_t>(slave),
                            kFwVersionIndex, kFwVersionSubindex);
    if (fw) {
      const bool fw_ok = fw->rfind(std::string(kExpectedFwPrefix), 0) == 0 ||
                         fw->find(std::string(kExpectedFwPrefix)) != std::string::npos;
      std::cout << "  FW version : " << *fw
                << (fw_ok ? "  ✓" : "  ✗  (expected prefix '" +
                                        std::string(kExpectedFwPrefix) + "')")
                << "\n";
      if (!fw_ok) all_fw_ok = false;
    } else {
      std::cout << "  FW version : <read failed>\n";
      all_fw_ok = false;
    }

    // -- Load reference CSV (if provided) --
    RefMap ref_values;
    if (ref_dir) {
      const std::size_t joint_idx =
          static_cast<std::size_t>(slave - 1);  // 0-based
      if (joint_idx < kJointCsvStems.size()) {
        const fs::path csv_path =
            *ref_dir / (kJointCsvStems[joint_idx] + ".csv");
        std::cout << "  Reference  : " << csv_path << "\n";
        ref_values = ParseRefCsv(csv_path);
        std::cout << "  Ref entries: " << ref_values.size() << "\n";
      }
    }
    std::cout << "\n";

    // -- Read SDO ranges --
    int slave_mismatches = 0;
    int slave_checked    = 0;

    for (const auto& [range_start, range_end] : kAddressRanges) {
      for (uint16_t index = range_start; index <= range_end; ++index) {
        // First probe subindex 0 to get the object's subindex count
        // (for record/array objects it holds the highest sub-index).
        auto sub0 = ReadSDOValue(static_cast<uint16_t>(slave), index, 0x00);
        if (!sub0) {
          // Object not present on this drive — skip silently.
          continue;
        }

        // Determine the subindex range to walk.
        // sub0 == 0 means the object is a single-value VAR (only subindex 0).
        const uint8_t max_sub =
            (*sub0 > 0 && *sub0 <= kMaxSubindex)
            ? static_cast<uint8_t>(*sub0)
            : 0;

        for (uint8_t sub = 0; sub <= max_sub; ++sub) {
          auto val_opt = ReadSDOValue(static_cast<uint16_t>(slave), index, sub);
          if (!val_opt) continue;
          const int32_t live_val = *val_opt;

          std::cout << "  0x" << std::hex << std::setw(4)
                    << std::setfill('0') << std::uppercase << index
                    << ", sub " << std::dec << std::setw(2)
                    << std::setfill(' ') << static_cast<int>(sub)
                    << " = " << std::setw(14) << live_val;

          if (!ref_values.empty()) {
            const SdoKey key = MakeKey(index, sub);
            auto it = ref_values.find(key);
            if (it != ref_values.end()) {
              ++slave_checked;
              ++total_checked;
              if (live_val == it->second) {
                std::cout << "  ✓";
              } else {
                std::cout << "  ✗  (expected " << it->second << ")";
                ++slave_mismatches;
                ++total_mismatches;
              }
            } else {
              std::cout << "  (no ref)";
            }
          }
          std::cout << "\n";
        }
      }
    }

    if (!ref_values.empty()) {
      std::cout << "\n  Slave " << slave << " result: "
                << (slave_checked - slave_mismatches) << "/" << slave_checked
                << " registers match";
      if (slave_mismatches == 0) {
        std::cout << "  ✓\n";
      } else {
        std::cout << "  ✗  (" << slave_mismatches << " mismatch"
                  << (slave_mismatches != 1 ? "es" : "") << ")\n";
      }
    }
    std::cout << "\n";
  }

  // -------------------------------------------------------------------------
  // 3. Summary
  // -------------------------------------------------------------------------
  ec_close();

  std::cout << "══════════════════════════════════════════════════════\n";
  std::cout << "SUMMARY\n";
  std::cout << "══════════════════════════════════════════════════════\n";
  std::cout << "  Firmware   : " << (all_fw_ok ? "PASS ✓" : "FAIL ✗") << "\n";
  if (ref_dir) {
    std::cout << "  Registers  : " << (total_checked - total_mismatches)
              << "/" << total_checked << " match";
    if (total_mismatches == 0) {
      std::cout << "  →  PASS ✓\n";
    } else {
      std::cout << "  →  FAIL ✗ (" << total_mismatches << " mismatch"
                << (total_mismatches != 1 ? "es" : "") << ")\n";
    }
  }
  std::cout << "══════════════════════════════════════════════════════\n";

  const bool overall_pass =
      all_fw_ok && (ref_dir ? total_mismatches == 0 : true);
  return overall_pass ? EXIT_SUCCESS : EXIT_FAILURE;
}
