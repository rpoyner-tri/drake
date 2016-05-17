#include "drake/util/drakeAppUtil.h"
#include "gtest/gtest.h"

namespace drake {
namespace util {
namespace {

TEST(drakeAppUtilTest, NoArgsOption) {
  char nothing[]{0};
  char* arg = nothing;
  char** arglist = &arg;
  EXPECT_EQ(nullptr, getCommandLineOption(arglist, arglist, "something"));
}

TEST(drakeAppUtilTest, OneArgMatchOption) {
  char cstr[] = "something";
  char* arg = cstr;
  char** arglist = &arg;
  EXPECT_EQ(nullptr, getCommandLineOption(arglist, arglist + 1, "something"));
}

TEST(drakeAppUtilTest, OneNoMatchOption) {
  char cstr[] = "else";
  char* arg = cstr;
  char** arglist = &arg;
  EXPECT_EQ(nullptr, getCommandLineOption(arglist, arglist + 1, "something"));
}

TEST(drakeAppUtilTest, TwoArgMatchOption) {
  const char* cstrs[] = { "something", "true" };
  EXPECT_EQ(cstrs[1], getCommandLineOption(cstrs, cstrs + 2, "something"));
}

TEST(drakeAppUtilTest, TwoNoMatchOption) {
  const char* cstrs[] = { "else", "true" };
  EXPECT_EQ(nullptr, getCommandLineOption(cstrs, cstrs + 2, "something"));
}

TEST(drakeAppUtilTest, NoArgsExists) {
  char nothing[]{0};
  char* arg = nothing;
  char** arglist = &arg;
  EXPECT_FALSE(commandLineOptionExists(arglist, arglist, "something"));
}

TEST(drakeAppUtilTest, OneArgMatchExists) {
  char cstr[] = "something";
  char* arg = cstr;
  char** arglist = &arg;
  EXPECT_TRUE(commandLineOptionExists(arglist, arglist + 1, "something"));
}

TEST(drakeAppUtilTest, OneNoMatchExists) {
  char cstr[] = "else";
  char* arg = cstr;
  char** arglist = &arg;
  EXPECT_FALSE(commandLineOptionExists(arglist, arglist + 1, "something"));
}

TEST(drakeAppUtilTest, TwoArgMatchExists) {
  const char* cstrs[] = { "true", "something" };
  EXPECT_TRUE(commandLineOptionExists(cstrs, cstrs + 2, "something"));
}

TEST(drakeAppUtilTest, TwoNoMatchExists) {
  const char* cstrs[] = { "else", "true" };
  EXPECT_FALSE(commandLineOptionExists(cstrs, cstrs + 2, "something"));
}

}  // namespace
}  // namespace util
}  // namespace drake
