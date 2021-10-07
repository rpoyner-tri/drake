#pragma once

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/autodiff2.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"

namespace drake {
namespace test {

class CppADdTest : public ::testing::Test {
 protected:
  // Evaluates a given function f with values of CppADd and values with
  // AutoDiffd<3>. It checks if the values and the derivatives of those
  // evaluation results are matched.
  //
  // The CppAD library detects NaN and throws in a lot of annoying
  // places. Since we accept combinations of functions and expressions that are
  // guaranteed to yield NaN, this check method has to check and potentially
  // give up in multiple places where NaNs might have turned up.
  template <typename F>
  void Check(const F& f, const char* expr) {
    SCOPED_TRACE(fmt::format("Testing expression '{}'", expr));

    std::vector<double> inputs{0.4, 0.3};
    if (!std::isfinite(f(inputs[0], inputs[1]))) { return; }  // punt inanities.

    // CppADd inputs -- xs.
    std::vector<CppADd> xs_cpp{{inputs[0]}, {inputs[1]}};
    ::CppAD::Independent(xs_cpp);

    // Compute the tape.
    const CppADd e_cpp{f(xs_cpp[0], xs_cpp[1])};
    ::CppAD::ADFun<double> adfun;
    adfun.Dependent(xs_cpp, std::vector<CppADd>{e_cpp});

    // We can get here and still be full of NaN, because libraries disagree on
    // some math corner case definitions, e.g. atan2(). So, we turn off library
    // throw-on-NaN behavior here and rely on more explicit checks below.
    adfun.check_for_nan(false);

    // Compute values and derivatives.
    constexpr int kNumInputs = 2;
    constexpr int kNumOutputs = 1;
    std::vector<double> weights{1.0};
    auto values = adfun.Forward(0, inputs);

    // Gradients forward.
    std::vector<double> grads_f(kNumInputs);
    for (int k = 0; k < kNumInputs; k++) {
      std::vector<double> grad_inputs(kNumInputs, 0.0);
      grad_inputs[k] = 1.0;
      grads_f[k] = adfun.Forward(1, grad_inputs)[0];
    }
    // Gradients reverse.
    auto grads_r = adfun.Reverse(1, weights);
    // Jacobian.
    auto jac = adfun.Jacobian(inputs);

    DRAKE_ASSERT(values.size() == kNumOutputs);
    if (!std::isfinite(values[0])) { return; }  // punt inanities.
    EXPECT_DOUBLE_EQ(values[0], f(inputs[0], inputs[1]));

    // Check gradient values.
    DRAKE_ASSERT(grads_r.size() == kNumInputs);
    DRAKE_ASSERT(jac.size() == kNumInputs);

    VectorX<AutoDiffXd> xs_xd(kNumInputs);
    math::InitializeAutoDiff(
        Eigen::Map<Eigen::VectorXd>(inputs.data(), kNumInputs), &xs_xd);
    const AutoDiffXd e_xd{f(xs_xd[0], xs_xd[1])};

    for (int k = 0; k < kNumInputs; k++) {
      SCOPED_TRACE(fmt::format("Checking partial at index {}", k));
      auto& deriv_k = e_xd.derivatives()(k);
      if (!std::isfinite(deriv_k)) { continue; }
      if (std::isfinite(grads_f[k])) {
        EXPECT_NEAR(grads_f[k], deriv_k, 2e-12);
      }
      if (std::isfinite(grads_f[k])) {
        EXPECT_NEAR(grads_r[k], deriv_k, 2e-12);
        EXPECT_NEAR(jac[k], grads_f[k], 2e-12);
      }
      if (std::isfinite(grads_r[k])) {
        EXPECT_NEAR(jac[k], deriv_k, 2e-12);
        EXPECT_NEAR(jac[k], grads_r[k], 2e-12);
      }
    }
  }
};

// clang warns on C++2a extension here.
#ifdef __clang__
#pragma GCC diagnostic ignored "-Wc++2a-extensions"
#endif
#define CHECK_EXPR(expr)                                                \
  Check([]<typename T>(const T& x, const T& y) -> T { return expr; }, #expr)


#define CHECK_BINARY_OP(bop, x, y, c) \
  CHECK_EXPR((x bop x)bop(y bop y));  \
  CHECK_EXPR((x bop y)bop(x bop y));  \
  CHECK_EXPR((x bop y)bop c);         \
  CHECK_EXPR((x bop y)bop c);         \
  CHECK_EXPR((x bop c)bop y);         \
  CHECK_EXPR((c bop x)bop y);         \
  CHECK_EXPR(x bop(y bop c));         \
  CHECK_EXPR(x bop(c bop y));         \
  CHECK_EXPR(c bop(x bop y));

// The multiplicative factor 0.9 < 1.0 let us call function such as asin, acos,
// etc. whose arguments must be in (-1, 1).
// The additive factor 5.0 let us call functions whose arguments must be
// positive.
#define CHECK_UNARY_FUNCTION(f, x, y, c) \
  CHECK_EXPR(f(x + x) + (y + y));        \
  CHECK_EXPR(f(x + y) + (x + y));        \
  CHECK_EXPR(f(x - x + 5.0) + (y - y));  \
  CHECK_EXPR(f(x - y + 5.0) + (x - y));  \
  CHECK_EXPR(f(x * x) + (y * y));        \
  CHECK_EXPR(f(x * y) + (x * y));        \
  CHECK_EXPR(f(0.9 * x / x) + (y / y));  \
  CHECK_EXPR(f(x / y) + (x / y));        \
  CHECK_EXPR(f(x + c) + y);              \
  CHECK_EXPR(f(x - c + 5.0) + y);        \
  CHECK_EXPR(f(x * c + 5.0) + y);        \
  CHECK_EXPR(f(x + 5.0) + y / c);        \
  CHECK_EXPR(f(c + x + 5.0) + y);        \
  CHECK_EXPR(f(c - x + 5.0) + y);        \
  CHECK_EXPR(f(c * x + 5.0) + y);        \
  CHECK_EXPR(f(c / x  + 5.0) + y);       \
  CHECK_EXPR(f(-x  + 5.0) + y);

#define CHECK_BINARY_FUNCTION_ADS_ADS(f, x, y, c) \
  CHECK_EXPR(f(x + x, y + y) + x);                \
  CHECK_EXPR(f(x + x, y + y) + y);                \
  CHECK_EXPR(f(x + y, y + y) + x);                \
  CHECK_EXPR(f(x + y, y + y) + y);                \
  CHECK_EXPR(f(x - x, y - y) - x);                \
  CHECK_EXPR(f(x - x, y - y) - y);                \
  CHECK_EXPR(f(x - y, y - y) - x);                \
  CHECK_EXPR(f(x - y, y - y) - y);                \
  CHECK_EXPR(f(x* x, y* y) * x);                  \
  CHECK_EXPR(f(x* x, y* y) * y);                  \
  CHECK_EXPR(f(x* y, y* y) * x);                  \
  CHECK_EXPR(f(x* y, y* y) * y);                  \
  CHECK_EXPR(f(x / x, y / y) / x);                \
  CHECK_EXPR(f(x / x, y / y) / y);                \
  CHECK_EXPR(f(x / y, y / y) / x);                \
  CHECK_EXPR(f(x / y, y / y) / y);                \
  CHECK_EXPR(f(x + c, y + c) + x);                \
  CHECK_EXPR(f(c + x, c + x) + y);                \
  CHECK_EXPR(f(x* c, y* c) + x);                  \
  CHECK_EXPR(f(c* x, c* x) + y);                  \
  CHECK_EXPR(f(-x, -y) + y)

// The additive factor 5.0 let us call functions whose arguments must be
// positive in order to have well defined derivatives. Eg.: sqrt, pow.
#define CHECK_BINARY_FUNCTION_ADS_SCALAR(f, x, y, c) \
  CHECK_EXPR(f(x, c) + y);                           \
  CHECK_EXPR(f(x + x, c) + y);                       \
  CHECK_EXPR(f(x + y, c) + y);                       \
  CHECK_EXPR(f(x - x + 5.0, c) - y);                 \
  CHECK_EXPR(f(x - x + 5.0, c) - y);                 \
  CHECK_EXPR(f(x* x, c) * y);                        \
  CHECK_EXPR(f(x* x, c) * y);                        \
  CHECK_EXPR(f(x / x, c) / y);                       \
  CHECK_EXPR(f(x / x, c) / y);                       \
  CHECK_EXPR(f(x + c, c) + y);                       \
  CHECK_EXPR(f(c + x, c) + y);                       \
  CHECK_EXPR(f(x* c, c) + y);                        \
  CHECK_EXPR(f(c* x, c) + y);                        \
  CHECK_EXPR(f(-x, c) + y);

}  // namespace test
}  // namespace drake
