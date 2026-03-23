#pragma once

#include <filesystem>
#include <string>

namespace fs = std::filesystem;

// Config paths are relative to the current working directory. Resolve the
// directory that holds this executable (build tree or install prefix) so
// example binaries work from any cwd once YAML is copied/installed there.
inline fs::path ExampleConfigDir() {
  std::error_code ec;
  const fs::path exe = fs::read_symlink("/proc/self/exe", ec);
  if (!ec) {
    const fs::path dir = fs::weakly_canonical(exe).parent_path();
    if (fs::exists(dir / "joint_limits.yaml")) return dir;
    const fs::path share = dir.parent_path() / "share" / "elevated_control";
    if (fs::exists(share / "joint_limits.yaml")) return share;
    return dir;
  }
  return fs::current_path();
}
