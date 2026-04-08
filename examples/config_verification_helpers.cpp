// Copyright (c) 2025 Elevate Robotics Inc
#include "config_verification_helpers.hpp"

#include <charconv>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace config_verification {

Args ParseArgs(int argc, char** argv) {
  Args args;
  for (int i = 1; i < argc; ++i) {
    std::string_view arg(argv[i]);
    if ((arg == "--iface" || arg == "-i") && i + 1 < argc) {
      args.iface = argv[++i];
    } else if ((arg == "--ref-dir" || arg == "-r") && i + 1 < argc) {
      args.ref_dir = std::filesystem::path(argv[++i]);
    } else if (arg == "--help" || arg == "-h") {
      args.help = true;
    }
  }
  return args;
}

RefMap ParseRefCsv(const std::filesystem::path& path) {
  RefMap result;
  std::ifstream f(path);
  if (!f.is_open()) {
    std::cerr << "  [warn] Cannot open reference file: " << path << "\n";
    return result;
  }
  std::string line;
  while (std::getline(f, line)) {
    auto start = line.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) continue;
    line = line.substr(start);
    if (line.empty() || line[0] == '#') continue;
    if (line.rfind("META", 0) == 0)    continue;

    std::istringstream ss(line);
    std::string tok_index, tok_sub, tok_val;
    if (!std::getline(ss, tok_index, ',')) continue;
    if (!std::getline(ss, tok_sub,   ',')) continue;
    if (!std::getline(ss, tok_val,   ',')) continue;

    auto trim = [](std::string& s) {
      auto b = s.find_first_not_of(" \t");
      auto e = s.find_last_not_of(" \t\r\n");
      s = (b == std::string::npos) ? "" : s.substr(b, e - b + 1);
    };
    trim(tok_index); trim(tok_sub); trim(tok_val);

    uint16_t index = 0;
    if (tok_index.size() > 2 && tok_index[0] == '0' &&
        (tok_index[1] == 'x' || tok_index[1] == 'X')) {
      index = static_cast<uint16_t>(std::stoul(tok_index, nullptr, 16));
    } else {
      continue;
    }

    uint8_t subindex = 0;
    {
      auto r = std::from_chars(tok_sub.data(),
                               tok_sub.data() + tok_sub.size(), subindex);
      if (r.ec != std::errc{}) continue;
    }

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

bool CheckFirmwareVersion(const std::string& fw_version,
                          std::string_view expected_prefix) {
  const std::string prefix(expected_prefix);
  return fw_version.rfind(prefix, 0) == 0 ||
         fw_version.find(prefix) != std::string::npos;
}

CompareResult CompareRegister(const RefMap& ref, uint16_t index,
                              uint8_t subindex, int32_t live_value) {
  if (ref.empty()) {
    return {CompareResult::kNoRef, 0};
  }
  const SdoKey key = MakeKey(index, subindex);
  auto it = ref.find(key);
  if (it == ref.end()) {
    return {CompareResult::kNoRef, 0};
  }
  if (live_value == it->second) {
    return {CompareResult::kMatch, it->second};
  }
  return {CompareResult::kMismatch, it->second};
}

}  // namespace config_verification
