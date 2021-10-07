#include "drake/common/autodiff2.h"
#include "drake/common/test/autodiff2_test.h"

namespace drake {
namespace test {
namespace {

TEST_F(CppADdTest, Acos) {
  CHECK_UNARY_FUNCTION(acos, x, y, 0.1);
  CHECK_UNARY_FUNCTION(acos, x, y, -0.1);
  CHECK_UNARY_FUNCTION(acos, y, x, 0.1);
  CHECK_UNARY_FUNCTION(acos, y, x, -0.1);
}

}  // namespace
}  // namespace test
}  // namespace drake
