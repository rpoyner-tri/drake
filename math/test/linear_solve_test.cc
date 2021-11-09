#include "drake/math/linear_solve.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {
namespace math {
namespace {

template <typename Derived>
VectorX<typename Derived::Scalar> flatcopy(const Eigen::MatrixBase<Derived>& m) {
  // Force resolution of expression types and recover a concrete Derived object.
  Derived dm = m;
  return Eigen::Map<VectorX<typename Derived::Scalar>>(dm.data(), dm.size());
}

template <typename Derived>
Derived unflatcopy(VectorX<typename Derived::Scalar>& mflat,
                   const Eigen::MatrixBase<Derived>& m) {
      return Eigen::Map<Derived>(mflat.data(), m.rows(), m.cols());
}

template <template <typename, int...> typename LinearSolverType,
          typename DerivedA, typename DerivedB>
typename std::enable_if<internal::is_autodiffy_v<typename DerivedA::Scalar> ||
                        std::is_same_v<typename DerivedA::Scalar, double>>::type
TestSolveLinearSystem(const Eigen::MatrixBase<DerivedA>& A,
                      const Eigen::MatrixBase<DerivedB>& b) {
  using ScalarA = typename DerivedA::Scalar;
  using ScalarB = typename DerivedB::Scalar;
  for (const bool use_deprecated : {true, false}) {
    // Note: we can't declare Independent() here because input types are const.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    const auto x = use_deprecated ? LinearSolve<LinearSolverType>(A, b)
                                  : SolveLinearSystem<LinearSolverType>(A, b);
#pragma GCC diagnostic pop  // pop -Wdeprecated-declarations
    if constexpr (std::is_same_v<ScalarA, double> &&
                  std::is_same_v<ScalarB, double>) {
      static_assert(std::is_same_v<typename decltype(x)::Scalar, double>,
                    "The returned  x should have scalar type = double.");
    } else {
      static_assert(
          !std::is_same_v<typename decltype(x)::Scalar, double>,
          "The returned  x should have scalar type as AutoDiffScalar.");
    }

    // TODO(rpoyner-tri): checking code for CppADd is going to be very
    // different.

    // Now check Ax = z and A*∂x/∂z + ∂A/∂z * x = ∂b/∂z
    const auto Ax = A * x;
    Eigen::MatrixXd Ax_val, b_val;
    std::vector<Eigen::MatrixXd> Ax_grad;
    std::vector<Eigen::MatrixXd> b_grad;
    using ScalarAx = typename decltype(Ax)::Scalar;
    if constexpr (std::is_same_v<ScalarAx, double>) {
      Ax_val = Ax;
      for (int i = 0; i < Ax.cols(); ++i) {
        Ax_grad.push_back(
            Eigen::Matrix<double, DerivedA::RowsAtCompileTime, 0>::Zero(
                Ax.rows(), 0));
      }
    } else if constexpr (internal::is_autodiff2_v<ScalarAx>) {
        auto Aflat = flatcopy(A);
        MatrixX<CppADd> Axconcrete = Ax;
        auto Axflat = flatcopy(Axconcrete);
        ::CppAD::ADFun<double> Ax_fun(Aflat, Axflat);
        auto astate = ExtractDoubleOrThrow(Aflat);
        Ax_val = Ax_fun.Forward(0, astate);
        auto jac = Ax_fun.Jacobian(astate);
        // for (int i = 0; i < Ax.cols(); ++i) {
        //   Ax_grad.push_back(ExtractGradient(Ax.col(i)));
        // }
    } else {
      Ax_val = ExtractValue(Ax);
      for (int i = 0; i < Ax.cols(); ++i) {
        Ax_grad.push_back(ExtractGradient(Ax.col(i)));
      }
    }

    if constexpr (std::is_same_v<ScalarB, double>) {
      b_val = b;
      for (int i = 0; i < b.cols(); ++i) {
        b_grad.push_back(
            Eigen::Matrix<double, DerivedB::RowsAtCompileTime, 0>::Zero(
                b.rows(), 0));
      }
    } else {
      b_val = ExtractValue(b);
      for (int i = 0; i < b.cols(); ++i) {
        b_grad.push_back(ExtractGradient(b.col(i)));
      }
    }
    const double tol = 2E-12;
    EXPECT_TRUE(CompareMatrices(Ax_val, b_val, tol));
    EXPECT_EQ(b_grad.size(), Ax_grad.size());
    for (int i = 0; i < static_cast<int>(b_grad.size()); ++i) {
      if (b_grad[i].size() == 0 && Ax_grad[i].size() == 0) {
      } else if (b_grad[i].size() != 0 && Ax_grad[i].size() == 0) {
        EXPECT_TRUE(CompareMatrices(
            b_grad[i],
            Eigen::MatrixXd::Zero(b_grad[i].rows(), b_grad[i].cols()), tol));
      } else if (b_grad[i].size() == 0 && Ax_grad[i].size() != 0) {
        EXPECT_TRUE(CompareMatrices(
            Ax_grad[i],
            Eigen::MatrixXd::Zero(Ax_grad[i].rows(), Ax_grad[i].cols()), tol));
      } else {
        EXPECT_TRUE(CompareMatrices(Ax_grad[i], b_grad[i], tol));
      }
    }
    // Also use LinearSolver class, make sure it gives the same result as
    // SolveLinearSystem.
    const LinearSolver<LinearSolverType, DerivedA> solver(A);
    const auto x_result = solver.Solve(b);
    static_assert(std::is_same_v<typename decltype(x_result)::Scalar,
                                 typename decltype(x)::Scalar>);
    using Scalar_x_result = typename decltype(x_result)::Scalar;
    if constexpr (std::is_same_v<Scalar_x_result, double>) {
      EXPECT_TRUE(CompareMatrices(x_result, x));
    } else if constexpr (internal::is_autodiff2_v<Scalar_x_result>) {
      DRAKE_DEMAND(false);
    } else {
      EXPECT_TRUE(CompareMatrices(ExtractValue(x_result), ExtractValue(x)));
      EXPECT_TRUE(
          CompareMatrices(ExtractGradient(x_result), ExtractGradient(x)));
    }
  }
}

class LinearSolveTest : public ::testing::Test {
 public:
  LinearSolveTest() {
    A_val_ << 1, 3, 3, 10;
    b_vec_val_ << 3, 5;
    Eigen::Matrix<double, 2, Eigen::Dynamic> b_grad(2, 3);
    b_grad << 1, 2, 3, 4, 5, 6;
    b_vec_ad_ = InitializeAutoDiff(b_vec_val_, b_grad);
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < 2; ++j) {
        A_ad_(i, j).value() = A_val_(i, j);
      }
    }
    A_ad_(0, 0).derivatives() = Eigen::Vector3d(1, 2, 3);
    A_ad_(0, 1).derivatives() = Eigen::Vector3d(4, 5, 6);
    A_ad_(1, 0).derivatives() = Eigen::Vector3d(7, 8, 9);
    A_ad_(1, 1).derivatives() = Eigen::Vector3d(10, 11, 12);

    b_mat_val_ << 3, 5, 8, 1, -2, -3;
    b_mat_ad_ = b_mat_val_.cast<AutoDiffXd>();
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < 3; ++j) {
        b_mat_ad_(i, j).derivatives() = Eigen::Vector3d(i, j, i * j + 1);
      }
    }

    A_sym_ << symbolic::Expression(1), symbolic::Expression(3),
        symbolic::Expression(3), symbolic::Expression(10);
    const symbolic::Variable sym_u("u");
    const symbolic::Variable sym_v("v");
    b_sym_ << sym_u, 1, sym_v, -sym_u + sym_v, 3, 2;

    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < 2; ++j) {
        A_ad_fixed_der_size_(i, j).value() = A_ad_(i, j).value();
        A_ad_fixed_der_size_(i, j).derivatives() = A_ad_(i, j).derivatives();
      }
    }
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < 3; ++j) {
        b_ad_fixed_der_size_(i, j).value() = b_mat_ad_(i, j).value();
        b_ad_fixed_der_size_(i, j).derivatives() =
            b_mat_ad_(i, j).derivatives();
      }
    }
  }

  void MarkIndependent() {
    ::CppAD::Independent(b_vec_cp_);
// clang warns on C++2a extension here.
#ifdef __clang__
#pragma GCC diagnostic ignored "-Wc++2a-extensions"
#endif
    auto Independent = []<typename T>(T& m) {
      // VectorX<CppADd> mflat =
      //     Eigen::Map<VectorX<CppADd>>(m.data(), m.size());
      auto mflat = flatcopy(m);
      ::CppAD::Independent(mflat);
      // m = Eigen::Map<std::remove_reference_t<decltype(m)>>(
      //     mflat.data(), m.rows(), m.cols());
      m = unflatcopy(mflat, m);
    };  // NOLINT(readability/braces)
    Independent(A_cp_);
    Independent(b_mat_cp_);
  }

 protected:
  Eigen::Matrix2d A_val_;
  Eigen::Vector2d b_vec_val_;
  Eigen::Matrix<double, 2, 3> b_mat_val_;
  Eigen::Matrix<AutoDiffXd, 2, 2> A_ad_;
  Eigen::Matrix<AutoDiffXd, 2, 1> b_vec_ad_;
  Eigen::Matrix<AutoDiffXd, 2, 3> b_mat_ad_;
  MatrixX<CppADd> A_cp_{2, 2};
  VectorX<CppADd> b_vec_cp_{2};
  MatrixX<CppADd> b_mat_cp_{2, 3};
  Eigen::Matrix<symbolic::Expression, 2, 2> A_sym_;
  Eigen::Matrix<symbolic::Expression, 2, 3> b_sym_;
  // Use fixed-sized AutoDiffScalar.
  Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::Vector3d>, 2, 2>
      A_ad_fixed_der_size_;
  Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::Vector3d>, 2, 3>
      b_ad_fixed_der_size_;
};

TEST_F(LinearSolveTest, TestDoubleAandb) {
  // Both A and b are double matrices.
  TestSolveLinearSystem<Eigen::LLT>(A_val_, b_vec_val_);
  TestSolveLinearSystem<Eigen::LDLT>(A_val_, b_vec_val_);
  TestSolveLinearSystem<Eigen::ColPivHouseholderQR>(A_val_, b_vec_val_);
  TestSolveLinearSystem<Eigen::PartialPivLU>(A_val_, b_vec_val_);
  TestSolveLinearSystem<Eigen::LLT>(A_val_, b_mat_val_);
  TestSolveLinearSystem<Eigen::LDLT>(A_val_, b_mat_val_);
  TestSolveLinearSystem<Eigen::ColPivHouseholderQR>(A_val_, b_mat_val_);
  TestSolveLinearSystem<Eigen::PartialPivLU>(A_val_, b_mat_val_);
}

template <template <typename, int...> typename LinearSolverType,
          typename DerivedA, typename DerivedB>
void TestSolveLinearSystemSymbolic(const Eigen::MatrixBase<DerivedA>& A,
                                   const Eigen::MatrixBase<DerivedB>& b) {
  for (const bool use_deprecated : {true, false}) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    const auto x = use_deprecated ? LinearSolve<LinearSolverType>(A, b)
                                  : SolveLinearSystem<LinearSolverType>(A, b);
#pragma GCC diagnostic pop  // pop -Wdeprecated-declarations
    static_assert(
        std::is_same_v<typename decltype(x)::Scalar, symbolic::Expression>,
        "The scalar type should be symbolic expression");
    const Eigen::Matrix<symbolic::Expression, DerivedA::RowsAtCompileTime,
                        DerivedB::ColsAtCompileTime>
        Ax = A * x;
    EXPECT_EQ(Ax.rows(), b.rows());
    EXPECT_EQ(Ax.cols(), b.cols());
    for (int i = 0; i < b.rows(); ++i) {
      for (int j = 0; j < b.cols(); ++j) {
        EXPECT_PRED2(symbolic::test::ExprEqual, Ax(i, j).Expand(), b(i, j));
      }
    }
  }
}

TEST_F(LinearSolveTest, TestSymbolicAandb) {
  TestSolveLinearSystemSymbolic<Eigen::LLT>(A_sym_, b_sym_);
}

TEST_F(LinearSolveTest, TestAutoDiffAandDoubleB) {
  // A contains AutoDiffXd and b contains double.
  TestSolveLinearSystem<Eigen::LLT>(A_ad_, b_vec_val_);
  TestSolveLinearSystem<Eigen::LDLT>(A_ad_, b_vec_val_);
  TestSolveLinearSystem<Eigen::ColPivHouseholderQR>(A_ad_, b_vec_val_);
  TestSolveLinearSystem<Eigen::PartialPivLU>(A_ad_, b_vec_val_);
  TestSolveLinearSystem<Eigen::LLT>(A_ad_, b_mat_val_);
  TestSolveLinearSystem<Eigen::LDLT>(A_ad_, b_mat_val_);
  TestSolveLinearSystem<Eigen::ColPivHouseholderQR>(A_ad_, b_mat_val_);
  TestSolveLinearSystem<Eigen::PartialPivLU>(A_ad_, b_mat_val_);
}

TEST_F(LinearSolveTest, TestCppADdAandDoubleB) {
  // A contains AutoDiffXd and b contains double.
  MarkIndependent();
  TestSolveLinearSystem<Eigen::LLT>(A_cp_, b_vec_val_);
  // TestSolveLinearSystem<Eigen::LDLT>(A_cp_, b_vec_val_);
  // TestSolveLinearSystem<Eigen::ColPivHouseholderQR>(A_cp_, b_vec_val_);
  // TestSolveLinearSystem<Eigen::PartialPivLU>(A_cp_, b_vec_val_);
  // TestSolveLinearSystem<Eigen::LLT>(A_cp_, b_mat_val_);
  // TestSolveLinearSystem<Eigen::LDLT>(A_cp_, b_mat_val_);
  // TestSolveLinearSystem<Eigen::ColPivHouseholderQR>(A_cp_, b_mat_val_);
  // TestSolveLinearSystem<Eigen::PartialPivLU>(A_cp_, b_mat_val_);
}

TEST_F(LinearSolveTest, TestDoubleAandAutoDiffB) {
  // A contains double and b contains AutoDiffXd.
  TestSolveLinearSystem<Eigen::LLT>(A_val_, b_vec_ad_);
  TestSolveLinearSystem<Eigen::LDLT>(A_val_, b_vec_ad_);
  TestSolveLinearSystem<Eigen::ColPivHouseholderQR>(A_val_, b_vec_ad_);
  TestSolveLinearSystem<Eigen::PartialPivLU>(A_val_, b_vec_ad_);
  TestSolveLinearSystem<Eigen::LLT>(A_val_, b_mat_ad_);
  TestSolveLinearSystem<Eigen::LDLT>(A_val_, b_mat_ad_);
  TestSolveLinearSystem<Eigen::ColPivHouseholderQR>(A_val_, b_mat_ad_);
  TestSolveLinearSystem<Eigen::PartialPivLU>(A_val_, b_mat_ad_);
}

TEST_F(LinearSolveTest, TestNoGrad) {
  // A and b both contain AutoDiffXd but has empty gradient.
  TestSolveLinearSystem<Eigen::LLT>(A_val_.cast<AutoDiffXd>(),
                                    b_vec_val_.cast<AutoDiffXd>());
  TestSolveLinearSystem<Eigen::LLT>(A_val_.cast<AutoDiffXd>(),
                                    b_mat_val_.cast<AutoDiffXd>());
}

TEST_F(LinearSolveTest, TestBwithGrad) {
  // Test SolveLinearSystem with A containing empty gradient while b
  // contains meaningful gradient.
  TestSolveLinearSystem<Eigen::LLT>(A_val_.cast<AutoDiffXd>(), b_vec_ad_);
  TestSolveLinearSystem<Eigen::LDLT>(A_val_.cast<AutoDiffXd>(), b_vec_ad_);
  TestSolveLinearSystem<Eigen::ColPivHouseholderQR>(A_val_.cast<AutoDiffXd>(),
                                                    b_vec_ad_);
  TestSolveLinearSystem<Eigen::PartialPivLU>(A_val_.cast<AutoDiffXd>(),
                                             b_vec_ad_);
  TestSolveLinearSystem<Eigen::LLT>(A_val_.cast<AutoDiffXd>(), b_mat_ad_);
  TestSolveLinearSystem<Eigen::LDLT>(A_val_.cast<AutoDiffXd>(), b_mat_ad_);
  TestSolveLinearSystem<Eigen::ColPivHouseholderQR>(A_val_.cast<AutoDiffXd>(),
                                                    b_mat_ad_);
  TestSolveLinearSystem<Eigen::PartialPivLU>(A_val_.cast<AutoDiffXd>(),
                                             b_mat_ad_);
}

TEST_F(LinearSolveTest, TestAwithGrad) {
  // Test SolveLinearSystem with A containing gradient while b contains
  // no gradient.
  TestSolveLinearSystem<Eigen::LLT>(A_ad_, b_vec_val_.cast<AutoDiffXd>());
  TestSolveLinearSystem<Eigen::LDLT>(A_ad_, b_vec_val_.cast<AutoDiffXd>());
  TestSolveLinearSystem<Eigen::ColPivHouseholderQR>(
      A_ad_, b_vec_val_.cast<AutoDiffXd>());
  TestSolveLinearSystem<Eigen::PartialPivLU>(A_ad_,
                                             b_vec_val_.cast<AutoDiffXd>());
  TestSolveLinearSystem<Eigen::LLT>(A_ad_, b_mat_val_.cast<AutoDiffXd>());
  TestSolveLinearSystem<Eigen::LDLT>(A_ad_, b_mat_val_.cast<AutoDiffXd>());
  TestSolveLinearSystem<Eigen::ColPivHouseholderQR>(
      A_ad_, b_mat_val_.cast<AutoDiffXd>());
  TestSolveLinearSystem<Eigen::PartialPivLU>(A_ad_,
                                             b_mat_val_.cast<AutoDiffXd>());
}

TEST_F(LinearSolveTest, TestFixedDerivativeSize) {
  // Test SolveLinearSystem with either or both A and b containing
  // AutoDiffScalar, The AutoDiffScalar has a fixed derivative size.

  // Both A and B contain AutoDiffScalar.
  TestSolveLinearSystem<Eigen::LLT>(A_ad_fixed_der_size_, b_ad_fixed_der_size_);
  TestSolveLinearSystem<Eigen::LDLT>(A_ad_fixed_der_size_,
                                     b_ad_fixed_der_size_);
  TestSolveLinearSystem<Eigen::ColPivHouseholderQR>(A_ad_fixed_der_size_,
                                                    b_ad_fixed_der_size_);
  TestSolveLinearSystem<Eigen::PartialPivLU>(A_ad_fixed_der_size_,
                                             b_ad_fixed_der_size_);

  // Only b contains AutoDiffScalar, A contains double.
  TestSolveLinearSystem<Eigen::LLT>(A_val_, b_ad_fixed_der_size_);
  TestSolveLinearSystem<Eigen::LDLT>(A_val_, b_ad_fixed_der_size_);
  TestSolveLinearSystem<Eigen::ColPivHouseholderQR>(A_val_,
                                                    b_ad_fixed_der_size_);
  TestSolveLinearSystem<Eigen::PartialPivLU>(A_val_, b_ad_fixed_der_size_);

  // Only A contains AutoDiffScalar, b contains double.
  TestSolveLinearSystem<Eigen::LLT>(A_ad_fixed_der_size_, b_mat_val_);
  TestSolveLinearSystem<Eigen::LDLT>(A_ad_fixed_der_size_, b_mat_val_);
  TestSolveLinearSystem<Eigen::ColPivHouseholderQR>(A_ad_fixed_der_size_,
                                                    b_mat_val_);
  TestSolveLinearSystem<Eigen::PartialPivLU>(A_ad_fixed_der_size_, b_mat_val_);
}

TEST_F(LinearSolveTest, TestAbWithGrad) {
  // Test SolveLinearSystem with both A and b containing gradient.
  TestSolveLinearSystem<Eigen::LLT>(A_ad_, b_vec_ad_);
  TestSolveLinearSystem<Eigen::LDLT>(A_ad_, b_vec_ad_);
  TestSolveLinearSystem<Eigen::ColPivHouseholderQR>(A_ad_, b_vec_ad_);
  TestSolveLinearSystem<Eigen::PartialPivLU>(A_ad_, b_vec_ad_);
  TestSolveLinearSystem<Eigen::LLT>(A_ad_, b_mat_ad_);
  TestSolveLinearSystem<Eigen::LDLT>(A_ad_, b_mat_ad_);
  TestSolveLinearSystem<Eigen::ColPivHouseholderQR>(A_ad_, b_mat_ad_);
  TestSolveLinearSystem<Eigen::PartialPivLU>(A_ad_, b_mat_ad_);
}

TEST_F(LinearSolveTest, TestAbWithMaybeEmptyGrad) {
  // Test SolveLinearSystem with both A and b containing gradient in
  // some entries, and empty gradient in some other entries.
  A_ad_(1, 0).derivatives() = Eigen::VectorXd(0);
  b_vec_ad_(1).derivatives() = Eigen::VectorXd(0);
  TestSolveLinearSystem<Eigen::LLT>(A_ad_, b_vec_ad_);
  TestSolveLinearSystem<Eigen::LDLT>(A_ad_, b_vec_ad_);
}

TEST_F(LinearSolveTest, TestWrongGradientSize) {
  const Eigen::LLT<Eigen::Matrix2d> linear_solver(A_val_);
  // A's gradient has inconsistent size.
  auto A_ad_error = A_ad_;
  A_ad_error(0, 1).derivatives() = Eigen::Vector2d(1, 2);
  DRAKE_EXPECT_THROWS_MESSAGE(
      SolveLinearSystem<Eigen::LLT>(A_ad_error, b_vec_ad_),
      ".* has size 2, while another entry has size 3");

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  DRAKE_EXPECT_THROWS_MESSAGE(LinearSolve<Eigen::LLT>(A_ad_error, b_vec_ad_),
                              ".* has size 2, while another entry has size 3");
#pragma GCC diagnostic pop  // pop -Wdeprecated-declarations

  // b's gradient has inconsistent size.
  auto b_vec_ad_error = b_vec_ad_;
  b_vec_ad_error(1).derivatives() = Eigen::Vector2d(1, 2);
  DRAKE_EXPECT_THROWS_MESSAGE(
      SolveLinearSystem<Eigen::LLT>(A_ad_, b_vec_ad_error),
      ".* has size 2, while another entry has size 3");

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  DRAKE_EXPECT_THROWS_MESSAGE(LinearSolve<Eigen::LLT>(A_ad_, b_vec_ad_error),
                              ".* has size 2, while another entry has size 3");
#pragma GCC diagnostic pop  // pop -Wdeprecated-declarations

  // A and b have different number of derivatives.
  auto b_vec_ad_error2 = b_vec_ad_;
  b_vec_ad_error2(0).derivatives() = Eigen::Vector4d::Ones();
  b_vec_ad_error2(1).derivatives() = Eigen::Vector4d::Ones();
  DRAKE_EXPECT_THROWS_MESSAGE(
      SolveLinearSystem<Eigen::LLT>(A_ad_, b_vec_ad_error2),
      ".*A contains derivatives for 3 variables, while b contains derivatives "
      "for 4 variables");

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  DRAKE_EXPECT_THROWS_MESSAGE(
      LinearSolve<Eigen::LLT>(A_ad_, b_vec_ad_error2),
      ".*A contains derivatives for 3 variables, while b contains derivatives "
      "for 4 variables");
#pragma GCC diagnostic pop  // pop -Wdeprecated-declarations
}

template <template <typename, int...> typename LinearSolverType,
          typename DerivedA>
void CheckGetLinearSolver(const Eigen::MatrixBase<DerivedA>& A) {
  const auto linear_solver = GetLinearSolver<LinearSolverType>(A);
  if constexpr (std::is_same_v<typename DerivedA::Scalar, double> ||
                std::is_same_v<typename DerivedA::Scalar,
                               symbolic::Expression>) {
    static_assert(
        std::is_same_v<typename decltype(linear_solver)::MatrixType::Scalar,
                       typename DerivedA::Scalar>,
        "The scalar type don't match");
  } else {
    static_assert(
        std::is_same_v<typename decltype(linear_solver)::MatrixType::Scalar,
                       double>,
        "The scalar type should be double");
  }

  // Cast away the enum types of the matrix metrics so we can compare without
  // compiler warnings.
  constexpr int RowsSolver =
      static_cast<int>(decltype(linear_solver)::MatrixType::RowsAtCompileTime);
  constexpr int RowsA = static_cast<int>(DerivedA::RowsAtCompileTime);
  static_assert(RowsSolver == RowsA, "The matrix rows don't match");
  constexpr int ColsSolver =
      static_cast<int>(decltype(linear_solver)::MatrixType::ColsAtCompileTime);
  constexpr int ColsA = static_cast<int>(DerivedA::ColsAtCompileTime);
  static_assert(ColsSolver == ColsA, "The matrix rows don't match");
}

TEST_F(LinearSolveTest, GetLinearSolver) {
  // Check double-valued A matrix.
  CheckGetLinearSolver<Eigen::LLT>(A_val_);
  CheckGetLinearSolver<Eigen::LDLT>(A_val_);
  CheckGetLinearSolver<Eigen::PartialPivLU>(A_val_);
  CheckGetLinearSolver<Eigen::ColPivHouseholderQR>(A_val_);

  // Check symbolic::Expression-valued A matrix.
  CheckGetLinearSolver<Eigen::LLT>(A_sym_);

  // Check AutoDiffXd-valued A matrix.
  CheckGetLinearSolver<Eigen::LLT>(A_ad_);
  CheckGetLinearSolver<Eigen::LDLT>(A_ad_);
  CheckGetLinearSolver<Eigen::PartialPivLU>(A_ad_);
  CheckGetLinearSolver<Eigen::ColPivHouseholderQR>(A_ad_);
}
}  // namespace
}  // namespace math
}  // namespace drake
