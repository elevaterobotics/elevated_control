#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "config_verification_helpers.hpp"

namespace fs = std::filesystem;
using namespace config_verification;

namespace {

// Resolve the test/resources directory relative to the source tree.
// CMake defines TEST_RESOURCES_DIR for us; fall back to __FILE__-relative.
#ifdef TEST_RESOURCES_DIR
const fs::path kResourcesDir = TEST_RESOURCES_DIR;
#else
const fs::path kResourcesDir =
    fs::path(__FILE__).parent_path() / "resources";
#endif

class ConfigVerificationTest : public ::testing::Test {
 protected:
  void SetUp() override {
    tmp_dir_ = fs::temp_directory_path() / "config_verification_test";
    fs::create_directories(tmp_dir_);
  }

  void TearDown() override {
    if (fs::exists(tmp_dir_)) {
      fs::remove_all(tmp_dir_);
    }
  }

  fs::path WriteTempCsv(const std::string& filename,
                        const std::string& content) {
    fs::path p = tmp_dir_ / filename;
    std::ofstream f(p);
    f << content;
    return p;
  }

  fs::path tmp_dir_;
};

// ---------------------------------------------------------------------------
// MakeKey tests
// ---------------------------------------------------------------------------

TEST(MakeKeyTest, Basic) {
  EXPECT_EQ(MakeKey(0x2001, 0), 0x200100u);
  EXPECT_EQ(MakeKey(0x2001, 1), 0x200101u);
  EXPECT_EQ(MakeKey(0x607B, 2), 0x607B02u);
}

TEST(MakeKeyTest, DistinctKeysForDifferentInputs) {
  EXPECT_NE(MakeKey(0x2001, 0), MakeKey(0x2001, 1));
  EXPECT_NE(MakeKey(0x2001, 0), MakeKey(0x2002, 0));
  EXPECT_NE(MakeKey(0x6072, 0), MakeKey(0x6073, 0));
}

// ---------------------------------------------------------------------------
// ParseRefCsv tests
// ---------------------------------------------------------------------------

TEST_F(ConfigVerificationTest, ParseValidCsv) {
  auto ref = ParseRefCsv(kResourcesDir / "fake_joint.csv");
  EXPECT_EQ(ref.size(), 8u);
  EXPECT_EQ(ref[MakeKey(0x2001, 0)], 42);
  EXPECT_EQ(ref[MakeKey(0x2001, 1)], 100);
  EXPECT_EQ(ref[MakeKey(0x2001, 2)], -500);
  EXPECT_EQ(ref[MakeKey(0x2002, 0)], 1000);
  EXPECT_EQ(ref[MakeKey(0x6072, 0)], 3000);
  EXPECT_EQ(ref[MakeKey(0x6073, 0)], 2000);
  EXPECT_EQ(ref[MakeKey(0x607B, 1)], -180);
  EXPECT_EQ(ref[MakeKey(0x607B, 2)], 180);
}

TEST_F(ConfigVerificationTest, ParseSkipsCommentsAndMeta) {
  std::string csv =
      "META something\n"
      "# a comment\n"
      "0x2001,  0,  42\n"
      "META another\n"
      "# more comments\n"
      "0x2002,  0,  99\n";
  auto path = WriteTempCsv("comments.csv", csv);
  auto ref = ParseRefCsv(path);
  EXPECT_EQ(ref.size(), 2u);
  EXPECT_EQ(ref[MakeKey(0x2001, 0)], 42);
  EXPECT_EQ(ref[MakeKey(0x2002, 0)], 99);
}

TEST_F(ConfigVerificationTest, ParseHandlesWhitespace) {
  std::string csv = "  0x2001 ,  0 ,  42  \n";
  auto path = WriteTempCsv("whitespace.csv", csv);
  auto ref = ParseRefCsv(path);
  EXPECT_EQ(ref.size(), 1u);
  EXPECT_EQ(ref[MakeKey(0x2001, 0)], 42);
}

TEST_F(ConfigVerificationTest, ParseMalformedLinesSkipped) {
  auto ref = ParseRefCsv(kResourcesDir / "malformed.csv");
  // Only "0x2001,0,42" and "0x2005,1,999" are valid.
  EXPECT_EQ(ref.size(), 2u);
  EXPECT_EQ(ref[MakeKey(0x2001, 0)], 42);
  EXPECT_EQ(ref[MakeKey(0x2005, 1)], 999);
}

TEST_F(ConfigVerificationTest, ParseNonExistentFile) {
  auto ref = ParseRefCsv(tmp_dir_ / "does_not_exist.csv");
  EXPECT_TRUE(ref.empty());
}

TEST_F(ConfigVerificationTest, ParseEmptyFile) {
  auto path = WriteTempCsv("empty.csv", "");
  auto ref = ParseRefCsv(path);
  EXPECT_TRUE(ref.empty());
}

TEST_F(ConfigVerificationTest, ParseFloatTruncation) {
  std::string csv = "0x2001,  0,  3.7\n0x2001,  1,  -2.9\n";
  auto path = WriteTempCsv("floats.csv", csv);
  auto ref = ParseRefCsv(path);
  EXPECT_EQ(ref.size(), 2u);
  EXPECT_EQ(ref[MakeKey(0x2001, 0)], 3);
  EXPECT_EQ(ref[MakeKey(0x2001, 1)], -2);
}

// ---------------------------------------------------------------------------
// CheckFirmwareVersion tests
// ---------------------------------------------------------------------------

TEST(CheckFirmwareVersionTest, MatchingPrefix) {
  EXPECT_TRUE(CheckFirmwareVersion("5.6.0.1234", "5.6"));
}

TEST(CheckFirmwareVersionTest, NonMatchingPrefix) {
  EXPECT_FALSE(CheckFirmwareVersion("4.9.1", "5.6"));
}

TEST(CheckFirmwareVersionTest, EmptyString) {
  EXPECT_FALSE(CheckFirmwareVersion("", "5.6"));
}

TEST(CheckFirmwareVersionTest, PrefixInMiddle) {
  EXPECT_TRUE(CheckFirmwareVersion("ver-5.6-beta", "5.6"));
}

TEST(CheckFirmwareVersionTest, ExactMatch) {
  EXPECT_TRUE(CheckFirmwareVersion("5.6", "5.6"));
}

// ---------------------------------------------------------------------------
// CompareRegister tests
// ---------------------------------------------------------------------------

TEST(CompareRegisterTest, MatchingValue) {
  RefMap ref;
  ref[MakeKey(0x2001, 0)] = 42;
  auto result = CompareRegister(ref, 0x2001, 0, 42);
  EXPECT_EQ(result.status, CompareResult::kMatch);
  EXPECT_EQ(result.expected, 42);
}

TEST(CompareRegisterTest, MismatchedValue) {
  RefMap ref;
  ref[MakeKey(0x2001, 0)] = 42;
  auto result = CompareRegister(ref, 0x2001, 0, 99);
  EXPECT_EQ(result.status, CompareResult::kMismatch);
  EXPECT_EQ(result.expected, 42);
}

TEST(CompareRegisterTest, KeyNotInReference) {
  RefMap ref;
  ref[MakeKey(0x2001, 0)] = 42;
  auto result = CompareRegister(ref, 0x2002, 0, 42);
  EXPECT_EQ(result.status, CompareResult::kNoRef);
}

TEST(CompareRegisterTest, EmptyRefMap) {
  RefMap ref;
  auto result = CompareRegister(ref, 0x2001, 0, 42);
  EXPECT_EQ(result.status, CompareResult::kNoRef);
}

// ---------------------------------------------------------------------------
// ParseArgs tests
// ---------------------------------------------------------------------------

TEST(ParseArgsTest, ValidRefDirAndIface) {
  const char* argv[] = {"prog", "--ref-dir", "/some/path", "--iface", "eth0"};
  auto args = ParseArgs(5, const_cast<char**>(argv));
  EXPECT_EQ(args.ref_dir, fs::path("/some/path"));
  EXPECT_EQ(args.iface, "eth0");
  EXPECT_FALSE(args.help);
}

TEST(ParseArgsTest, DefaultIface) {
  const char* argv[] = {"prog", "--ref-dir", "/some/path"};
  auto args = ParseArgs(3, const_cast<char**>(argv));
  EXPECT_EQ(args.ref_dir, fs::path("/some/path"));
  EXPECT_EQ(args.iface, "eno0");
}

TEST(ParseArgsTest, ShortFlags) {
  const char* argv[] = {"prog", "-r", "/dir", "-i", "wlan0"};
  auto args = ParseArgs(5, const_cast<char**>(argv));
  EXPECT_EQ(args.ref_dir, fs::path("/dir"));
  EXPECT_EQ(args.iface, "wlan0");
}

TEST(ParseArgsTest, HelpFlag) {
  const char* argv[] = {"prog", "--help"};
  auto args = ParseArgs(2, const_cast<char**>(argv));
  EXPECT_TRUE(args.help);
}

TEST(ParseArgsTest, EmptyArgs) {
  const char* argv[] = {"prog"};
  auto args = ParseArgs(1, const_cast<char**>(argv));
  EXPECT_TRUE(args.ref_dir.empty());
  EXPECT_EQ(args.iface, "eno0");
  EXPECT_FALSE(args.help);
}

}  // namespace
