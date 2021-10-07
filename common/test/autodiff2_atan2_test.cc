#include <limits>

#include "drake/common/autodiff2.h"
#include "drake/common/test/autodiff2_test.h"

namespace drake {
namespace test {
namespace {

TEST_F(CppADdTest, Atan2) {
  CHECK_BINARY_FUNCTION_ADS_ADS(atan2, x, y, 0.1);
  CHECK_BINARY_FUNCTION_ADS_ADS(atan2, x, y, -0.1);
  CHECK_BINARY_FUNCTION_ADS_ADS(atan2, y, x, 0.4);
  CHECK_BINARY_FUNCTION_ADS_ADS(atan2, y, x, -0.4);
}

TEST_F(CppADdTest, Atan2Sanity) {
  // Test all pairs of {-1, 0, 1}.
  const std::vector<double> kTestPoints{
    -1.0,        -1.0,
    -1.0,         0.0,
    -1.0,         1.0,
     0.0,        -1.0,
     0.0,         0.0,
     0.0,         1.0,
     1.0,        -1.0,
     1.0,         0.0,
     1.0,         1.0,
  };
  static constexpr double kEps = std::numeric_limits<double>::epsilon();
  // Perturb test points none at all, or up, down, left, right.
  const std::vector kNudges{
      0.0,   0.0,
      0.0, -kEps,
    -kEps,   0.0,
      0.0,  kEps,
     kEps,   0.0,
  };

  // See https://en.wikipedia.org/wiki/Atan2#Derivative.
  // Note this is defined everywhere except 0, 0.
  auto analytic_jacobian = [](double y, double x) {
    double radius_sq = y * y + x * x;
    return std::vector<double>{ x / radius_sq, -y / radius_sq };
  };

  // Build the CppAD tape, and turn off NaN checking.
  std::vector<CppADd> tape_inputs{1., 1.};
  ::CppAD::Independent(tape_inputs);
  const CppADd tape_output{atan2(tape_inputs[0], tape_inputs[1])};
  ::CppAD::ADFun<double> adfun;
  adfun.Dependent(tape_inputs, std::vector<CppADd>{tape_output});
  adfun.check_for_nan(false);  // yee haw.

  for (size_t k = 0; k < kTestPoints.size(); k += 2) {
    for (size_t m = 0; m < kNudges.size(); m += 2) {
      std::vector point{
        kTestPoints[k] + kNudges[m], kTestPoints[k + 1] + kNudges[m + 1]};
      SCOPED_TRACE(fmt::format("for atan2({}, {}), k={}, m={}",
                               point[0], point[1], k, m));

      // We know the c library atan2() doesn't NaN.
      ASSERT_TRUE(std::isfinite(std::atan2(point[0], point[1])));

      auto values = adfun.Forward(0, point);
      // CppAD atan2 NaNs at the origin.
      EXPECT_TRUE(std::isfinite(values[0]) ||
                  (point[0] == 0.0 && point[1] == 0.0));
      if (std::isfinite(values[0])) {
        EXPECT_DOUBLE_EQ(values[0], std::atan2(point[0], point[1]));
      }

      auto auto_jac = adfun.Jacobian(point);
      auto analytic_jac = analytic_jacobian(point[0], point[1]);

      // Finities must match. No exceptions.
      ASSERT_EQ(std::isfinite(auto_jac[0]), std::isfinite(analytic_jac[0]));
      ASSERT_EQ(std::isfinite(auto_jac[1]), std::isfinite(analytic_jac[1]));
      ASSERT_EQ(std::isfinite(auto_jac[0]), std::isfinite(auto_jac[1]));
      ASSERT_EQ(std::isfinite(analytic_jac[0]), std::isfinite(analytic_jac[1]));
      ASSERT_EQ(!std::isfinite(auto_jac[0]),
                (point[0] == 0.0 && point[1] == 0.0));

      if (std::isfinite(auto_jac[0])) {
        EXPECT_DOUBLE_EQ(auto_jac[0], analytic_jac[0]);
      }
      if (std::isfinite(auto_jac[1])) {
        EXPECT_DOUBLE_EQ(auto_jac[1], analytic_jac[1]);
      }
    }
  }
}

}  // namespace
}  // namespace test
}  // namespace drake
