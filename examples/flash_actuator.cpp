// Copyright (c) 2026 Elevate Robotics Inc
//
// flash_actuator: Writes an entire CSV configuration to a single Synapticon
// drive on the EtherCAT bus via SDO writes.
//
// Usage:
//   sudo ./flash_actuator --ref-dir /path/to/yaw1.csv --ethercat-index 1
//                         [--iface eno0]

#include <soem/ethercat.h>

#include <charconv>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

// ───────────────────────────────────────────────────────────────────────────
// Types
// ───────────────────────────────────────────────────────────────────────────

struct SdoEntry {
  uint16_t index;
  uint8_t subindex;
  enum Type { kInt32, kUint32, kFloat, kString };
  Type type;
  int32_t int_val;
  uint32_t uint_val;
  float float_val;
  std::string str_val;
};

struct Args {
  std::string iface = "eno0";
  fs::path csv_path;
  int ethercat_index = -1;
  bool help = false;
};

// ───────────────────────────────────────────────────────────────────────────
// Argument parsing
// ───────────────────────────────────────────────────────────────────────────

Args ParseArgs(int argc, char** argv) {
  Args args;
  for (int i = 1; i < argc; ++i) {
    std::string_view arg(argv[i]);
    if ((arg == "--iface" || arg == "-i") && i + 1 < argc) {
      args.iface = argv[++i];
    } else if ((arg == "--ref-dir" || arg == "-r") && i + 1 < argc) {
      args.csv_path = fs::path(argv[++i]);
    } else if ((arg == "--ethercat-index" || arg == "-e") && i + 1 < argc) {
      args.ethercat_index = std::atoi(argv[++i]);
      if (args.ethercat_index < 1 || args.ethercat_index > 6) {
        std::cerr << "Error: --ethercat-index must be between 1 and 6 (got "
                  << args.ethercat_index << ")\n";
        std::exit(1);
      }
    } else if (arg == "--help" || arg == "-h") {
      args.help = true;
    }
  }
  return args;
}

// ───────────────────────────────────────────────────────────────────────────
// CSV parsing (type-preserving: int32 / uint32 / float / string)
// ───────────────────────────────────────────────────────────────────────────

static inline void Trim(std::string& s) {
  auto b = s.find_first_not_of(" \t");
  auto e = s.find_last_not_of(" \t\r\n");
  s = (b == std::string::npos) ? "" : s.substr(b, e - b + 1);
}

std::vector<SdoEntry> ParseCsv(const fs::path& path) {
  std::vector<SdoEntry> entries;
  std::ifstream f(path);
  if (!f.is_open()) {
    std::cerr << "Error: cannot open CSV file: " << path << "\n";
    return entries;
  }

  std::string line;
  while (std::getline(f, line)) {
    auto start = line.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) continue;
    line = line.substr(start);
    if (line.empty() || line[0] == '#') continue;
    if (line.rfind("META", 0) == 0) continue;

    std::istringstream ss(line);
    std::string tok_index, tok_sub, tok_val;
    if (!std::getline(ss, tok_index, ',')) continue;
    if (!std::getline(ss, tok_sub, ',')) continue;
    if (!std::getline(ss, tok_val, ',')) continue;

    Trim(tok_index);
    Trim(tok_sub);
    Trim(tok_val);

    // Parse index (must be hex 0x...)
    uint16_t index = 0;
    if (tok_index.size() > 2 && tok_index[0] == '0' &&
        (tok_index[1] == 'x' || tok_index[1] == 'X')) {
      index = static_cast<uint16_t>(std::stoul(tok_index, nullptr, 16));
    } else {
      continue;
    }

    // Parse subindex
    uint8_t subindex = 0;
    {
      auto r = std::from_chars(tok_sub.data(),
                               tok_sub.data() + tok_sub.size(), subindex);
      if (r.ec != std::errc{}) continue;
    }

    // Detect value type
    SdoEntry entry{};
    entry.index = index;
    entry.subindex = subindex;

    bool has_dot = tok_val.find('.') != std::string::npos;
    try {
      if (has_dot) {
        double d = std::stod(tok_val);
        entry.type = SdoEntry::kFloat;
        entry.float_val = static_cast<float>(d);
      } else if (!tok_val.empty() && tok_val[0] == '-') {
        long long v = std::stoll(tok_val);
        if (v < std::numeric_limits<int32_t>::min() ||
            v > std::numeric_limits<int32_t>::max()) {
          entry.type = SdoEntry::kString;
          entry.str_val = tok_val;
        } else {
          entry.type = SdoEntry::kInt32;
          entry.int_val = static_cast<int32_t>(v);
        }
      } else {
        unsigned long long u = std::stoull(tok_val);
        if (u > std::numeric_limits<uint32_t>::max()) {
          entry.type = SdoEntry::kString;
          entry.str_val = tok_val;
        } else if (u <= static_cast<unsigned long long>(
                       std::numeric_limits<int32_t>::max())) {
          entry.type = SdoEntry::kInt32;
          entry.int_val = static_cast<int32_t>(u);
        } else {
          entry.type = SdoEntry::kUint32;
          entry.uint_val = static_cast<uint32_t>(u);
        }
      }
    } catch (...) {
      entry.type = SdoEntry::kString;
      entry.str_val = tok_val;
    }

    entries.push_back(std::move(entry));
  }

  return entries;
}

// ───────────────────────────────────────────────────────────────────────────
// Main
// ───────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
  auto args = ParseArgs(argc, argv);

  if (args.help) {
    std::cout
        << "Usage: " << argv[0]
        << " --ref-dir <path/to/config.csv> --ethercat-index <N>"
           " [--iface <nic>]\n"
        << "\n"
        << "  --ref-dir         Path to a CSV file containing the motor\n"
        << "                    configuration to flash.  (Required.)\n"
        << "  --ethercat-index  1-based index of the target slave on the\n"
        << "                    EtherCAT bus.  (Required.)\n"
        << "  --iface           Network interface (default: eno0)\n";
    return EXIT_SUCCESS;
  }

  if (args.csv_path.empty()) {
    std::cerr << "Error: --ref-dir is required.\n";
    return EXIT_FAILURE;
  }
  if (!fs::is_regular_file(args.csv_path)) {
    std::cerr << "Error: --ref-dir path does not exist or is not a file: "
              << args.csv_path << "\n";
    return EXIT_FAILURE;
  }
  if (args.ethercat_index < 1) {
    std::cerr << "Error: --ethercat-index must be >= 1.\n";
    return EXIT_FAILURE;
  }

  // -----------------------------------------------------------------------
  // 1. Parse CSV
  // -----------------------------------------------------------------------
  auto entries = ParseCsv(args.csv_path);
  if (entries.empty()) {
    std::cerr << "Error: no valid entries found in " << args.csv_path << "\n";
    return EXIT_FAILURE;
  }
  std::cout << "Parsed " << entries.size() << " SDO entries from "
            << args.csv_path << "\n";

  // -----------------------------------------------------------------------
  // 2. Initialise EtherCAT (Pre-Op only — no PDO mapping needed)
  // -----------------------------------------------------------------------
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
  std::cout << "Found " << slave_count << " slave(s).\n";

  const uint16_t slave = static_cast<uint16_t>(args.ethercat_index);
  if (slave > slave_count) {
    std::cerr << "Error: --ethercat-index " << args.ethercat_index
              << " exceeds slave count (" << slave_count << ").\n";
    ec_close();
    return EXIT_FAILURE;
  }

  std::cout << "Target slave " << slave << "  ("
            << ec_slave[slave].name << ")\n\n";

  // -----------------------------------------------------------------------
  // 3. Write SDO entries
  // -----------------------------------------------------------------------
  int written = 0;
  int failed = 0;
  int skipped = 0;

  for (const auto& entry : entries) {
    const bool skip = [&]() {
      switch (entry.index) {
        case 0x2001: // commutation offset; should be re-run
        case 0x200C:
        case 0x200D:
        case 0x2018:
        case 0x2019:
        case 0x2050:
        case 0x2051:
        case 0x2052:
        case 0x2053:
        case 0x2054:
        case 0x2060:
        case 0x2150:
        case 0x2151:
        case 0x2152:
        case 0x2500:
        case 0x60F2:
          return true;
        default:
          break;
      }
      const uint8_t s = entry.subindex;
      // motor phase configuration (inverted vs normal); may change when commutation offset re-runs
      if (entry.index == 0x2003 && (s == 5 || s == 6)) return true;
      if (entry.index == 0x2004 && (s == 6 || s == 12)) return true;
      if (entry.index == 0x2005 && s == 1) return true;
      if (entry.index == 0x2008 && s == 1) return true;
      if (entry.index == 0x200A && s >= 3 && s <= 5) return true;
      if (entry.index == 0x2010 && s >= 13 && s <= 17) return true;
      if (entry.index == 0x2017 && s == 2) return true;
      if (entry.index == 0x2023 && s == 4) return true;
      if (entry.index == 0x2038 && s == 1) return true;
      if (entry.index == 0x2212 && s == 1) return true;
      return false;
    }();
    if (skip) {
      ++skipped;
      std::cout << "  0x" << std::hex << std::setw(4) << std::setfill('0')
                << std::uppercase << entry.index << ", sub " << std::dec
                << std::setw(2) << std::setfill(' ')
                << static_cast<int>(entry.subindex) << "  SKIPPED\n";
      continue;
    }

    int result = 0;

    switch (entry.type) {
      case SdoEntry::kInt32: {
        int32_t val = entry.int_val;
        result = ec_SDOwrite(slave, entry.index, entry.subindex, FALSE,
                             sizeof(val), &val, EC_TIMEOUTRXM);
        break;
      }
      case SdoEntry::kUint32: {
        uint32_t val = entry.uint_val;
        result = ec_SDOwrite(slave, entry.index, entry.subindex, FALSE,
                             sizeof(val), &val, EC_TIMEOUTRXM);
        break;
      }
      case SdoEntry::kFloat: {
        float val = entry.float_val;
        result = ec_SDOwrite(slave, entry.index, entry.subindex, FALSE,
                             sizeof(val), &val, EC_TIMEOUTRXM);
        break;
      }
      case SdoEntry::kString: {
        result = ec_SDOwrite(
            slave, entry.index, entry.subindex, FALSE,
            static_cast<int>(entry.str_val.size()),
            const_cast<char*>(entry.str_val.data()), EC_TIMEOUTRXM);
        break;
      }
    }

    std::cout << "  0x" << std::hex << std::setw(4) << std::setfill('0')
              << std::uppercase << entry.index << ", sub " << std::dec
              << std::setw(2) << std::setfill(' ')
              << static_cast<int>(entry.subindex) << "  ";

    if (result > 0) {
      ++written;
      switch (entry.type) {
        case SdoEntry::kInt32:
          std::cout << "= " << std::setw(14) << entry.int_val << "  OK\n";
          break;
        case SdoEntry::kUint32:
          std::cout << "= " << std::setw(14) << entry.uint_val << "  OK\n";
          break;
        case SdoEntry::kFloat:
          std::cout << "= " << std::setw(14) << std::fixed
                    << std::setprecision(4) << entry.float_val << "  OK\n";
          break;
        case SdoEntry::kString:
          std::cout << "= " << entry.str_val << "  OK\n";
          break;
      }
    } else {
      ++failed;
      std::cerr << "  FAILED (ec_SDOwrite returned " << result << ")\n";
    }
  }

  // -----------------------------------------------------------------------
  // 4. Summary
  // -----------------------------------------------------------------------
  ec_close();

  std::cout << "\n";
  std::cout << "══════════════════════════════════════════════════════\n";
  std::cout << "SUMMARY\n";
  std::cout << "══════════════════════════════════════════════════════\n";
  std::cout << "  Written : " << written << " / " << entries.size() << "\n";
  std::cout << "  Skipped : " << skipped << "\n";
  std::cout << "  Failed  : " << failed << "\n";
  std::cout << "  Result  : " << (failed == 0 ? "PASS" : "FAIL") << "\n";
  std::cout << "══════════════════════════════════════════════════════\n";

  return (failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
