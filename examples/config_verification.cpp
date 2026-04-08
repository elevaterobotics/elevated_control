// Copyright (c) 2025 Elevate Robotics Inc
//
// config_verification: Reads SDO configuration registers from all Synapticon
// drives on the EtherCAT bus, compares them against known-good CSV reference
// files (e.g. from aladdin_config/synapticon_config/P3S2/), and reports any
// mismatches.
//
// Usage:
//   sudo ./config_verification --ref-dir /path/to/P3S2 [--iface eno0]

#include "config_verification_helpers.hpp"
#include "elevated_control/state_machine.hpp"
#include "example_config_dir.hpp"

#include <soem/ethercat.h>

#include <cstdlib>
#include <iomanip>
#include <iostream>

namespace fs = std::filesystem;
using elevated_control::ReadSDOString;
using elevated_control::ReadSDOValue;
using namespace config_verification;

int main(int argc, char** argv) {
  auto args = ParseArgs(argc, argv);

  if (args.help) {
    std::cout
        << "Usage: " << argv[0]
        << " --ref-dir <path/to/P3S2> [--iface <nic>]\n"
        << "\n"
        << "  --ref-dir Path to folder containing per-joint CSV reference\n"
        << "            files (e.g. yaw1.csv, SAA.csv …).  Live values are\n"
        << "            compared against the reference and mismatches are\n"
        << "            flagged.  (Required.)\n"
        << "  --iface   Network interface (default: eno0)\n";
    return EXIT_SUCCESS;
  }

  if (args.ref_dir.empty()) {
    std::cerr << "Error: --ref-dir is required.\n";
    return EXIT_FAILURE;
  }
  if (!fs::is_directory(args.ref_dir)) {
    std::cerr << "Error: --ref-dir path does not exist or is not a directory: "
              << args.ref_dir << "\n";
    return EXIT_FAILURE;
  }

  // -------------------------------------------------------------------------
  // 1. Initialise EtherCAT (Pre-Op only — no PDO mapping needed)
  // -------------------------------------------------------------------------
  std::cout << "Initialising EtherCAT on interface '" << args.iface
            << "' ...\n";
  if (ec_init(args.iface.c_str()) <= 0) {
    std::cerr << "ec_init failed. Are you root and is '" << args.iface
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
      const bool fw_ok = CheckFirmwareVersion(*fw, kExpectedFwPrefix);
      std::cout << "  FW version : " << *fw
                << (fw_ok ? "  ✓" : "  ✗  (expected prefix '" +
                                        std::string(kExpectedFwPrefix) + "')")
                << "\n";
      if (!fw_ok) all_fw_ok = false;
    } else {
      std::cout << "  FW version : <read failed>\n";
      all_fw_ok = false;
    }

    // -- Load reference CSV --
    RefMap ref_values;
    const std::size_t joint_idx =
        static_cast<std::size_t>(slave - 1);
    if (joint_idx < kJointCsvStems.size()) {
      const fs::path csv_path =
          args.ref_dir / (kJointCsvStems[joint_idx] + ".csv");
      std::cout << "  Reference  : " << csv_path << "\n";
      ref_values = ParseRefCsv(csv_path);
      std::cout << "  Ref entries: " << ref_values.size() << "\n";
    }
    std::cout << "\n";

    // -- Read SDO ranges --
    int slave_mismatches = 0;
    int slave_checked    = 0;

    for (const auto& [range_start, range_end] : kAddressRanges) {
      for (uint16_t index = range_start; index <= range_end; ++index) {
        auto sub0 = ReadSDOValue(static_cast<uint16_t>(slave), index, 0x00);
        if (!sub0) continue;

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

          auto cmp = CompareRegister(ref_values, index, sub, live_val);
          switch (cmp.status) {
            case CompareResult::kMatch:
              ++slave_checked;
              ++total_checked;
              std::cout << "  ✓";
              break;
            case CompareResult::kMismatch:
              ++slave_checked;
              ++total_checked;
              ++slave_mismatches;
              ++total_mismatches;
              std::cout << "  ✗  (expected " << cmp.expected << ")";
              break;
            case CompareResult::kNoRef:
              if (!ref_values.empty()) {
                std::cout << "  (no ref)";
              }
              break;
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
  std::cout << "  Registers  : " << (total_checked - total_mismatches)
            << "/" << total_checked << " match";
  if (total_mismatches == 0) {
    std::cout << "  →  PASS ✓\n";
  } else {
    std::cout << "  →  FAIL ✗ (" << total_mismatches << " mismatch"
              << (total_mismatches != 1 ? "es" : "") << ")\n";
  }
  std::cout << "══════════════════════════════════════════════════════\n";

  const bool overall_pass = all_fw_ok && total_mismatches == 0;
  return overall_pass ? EXIT_SUCCESS : EXIT_FAILURE;
}
