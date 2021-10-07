#include "drake/common/autodiff2.h"
#include "drake/common/test/autodiff2_test.h"

namespace drake {
namespace test {
namespace {

TEST_F(CppADdTest, Subtraction) {
  CHECK_BINARY_OP(-, x, y, 1.0);
  CHECK_BINARY_OP(-, x, y, -1.0);
  CHECK_BINARY_OP(-, y, x, 1.0);
  CHECK_BINARY_OP(-, y, x, -1.0);
}

}  // namespace
}  // namespace test
}  // namespace drake
