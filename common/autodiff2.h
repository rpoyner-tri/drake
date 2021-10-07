#pragma once

// A header for prototyping inclusion of autodiff via CppAD.

#include <algorithm>
#include <limits>

#include <Eigen/Core>
#include <cppad/cppad.hpp>
#include <cppad/example/cppad_eigen.hpp>

#include "drake/common/cond.h"
#include "drake/common/dummy_value.h"
#include "drake/common/extract_double.h"

namespace CppAD {

template <class T>
bool isfinite(const AD<T>& x) {
  return !(isinf(x) || isnan(x));
}

template <class T>
bool isinf(const AD<T>& x) {
  AD<T> infinity{std::numeric_limits<T>::infinity()};
  return !(x == infinity || x == - infinity);
}

template <class T>
AD<T> ceil(const AD<T>& x) {
  AD<T> xint{Integer(x)};
  return xint >= x ? xint : xint + 1;
}

template <class T>
AD<T> floor(const AD<T>& x) {
  AD<T> xint{Integer(x)};
  return xint <= x ? xint : xint - 1;
}

/// Provides if-then-else expression for CppADd type.
template <class T>
inline AD<T> if_then_else(bool f_cond, const AD<T>& x, const AD<T>& y) {
  return f_cond ? x : y;
}

template <class T>
inline AD<T> max(const AD<T>& x, const AD<T>& y) {
  return x >= y ? x : y;
}

template <class T>
inline AD<T> max(const AD<T>& x, const T& y) {
  return max(x, AD<T>{y});
}

template <class T>
inline AD<T> max(const T& x, const AD<T>& y) {
  return max(y, x);
}

template <class T>
inline AD<T> min(const AD<T>& x, const AD<T>& y) {
  return x <= y ? x : y;
}

template <class T>
inline AD<T> min(const AD<T>& x, const T& y) {
  return min(x, AD<T>{y});
}

template <class T>
inline AD<T> min(const T& x, const AD<T>& y) {
  return min(y, x);
}

template <typename T>
double nexttoward(const AD<T>& from, long double to) {
  return std::nexttoward(Value(Var2Par(from)), to);
}

}  // namespace CppAD

namespace drake {

template <typename Base>
using CppAD = CppAD::AD<Base>;

using CppADd = CppAD<double>;

/// Returns the autodiff scalar's value() as a double.  Never throws.
/// Overloads ExtractDoubleOrThrow from common/extract_double.h.
inline double ExtractDoubleOrThrow(const CppADd& scalar) {
  return ::CppAD::Value(::CppAD::Var2Par(scalar));
}

/// Returns @p matrix as an Eigen::Matrix<double, ...> with the same size
/// allocation as @p matrix.  Calls ExtractDoubleOrThrow on each element of the
/// matrix, and therefore throws if any one of the extractions fail.
template <int RowsAtCompileTime, int ColsAtCompileTime,
          int Options, int MaxRowsAtCompileTime, int MaxColsAtCompileTime>
auto ExtractDoubleOrThrow(
    const Eigen::MatrixBase<Eigen::Matrix<
        CppADd, RowsAtCompileTime, ColsAtCompileTime,
        Options, MaxRowsAtCompileTime, MaxColsAtCompileTime>>& matrix) {
  return matrix
      .unaryExpr([](const CppADd& value) {
        return ExtractDoubleOrThrow(value);
      })
      .eval();
}

/// Specializes common/dummy_value.h.
template <>
struct dummy_value<CppADd> {
  static CppADd get() {
    return ::CppAD::numeric_limits<CppADd>::quiet_NaN();
  }
};

}  // namespace drake

namespace Eigen {

template <typename BinOp>
struct ScalarBinaryOpTraits<::drake::CppADd, double, BinOp> {
  typedef ::drake::CppADd ReturnType;
};

template<typename BinOp>
struct ScalarBinaryOpTraits<double, ::drake::CppADd, BinOp> {
  typedef ::drake::CppADd ReturnType;
};

}  // namespace Eigen

namespace std {

/* Provides std::numeric_limits<drake::CppADd>. */
template <>
struct numeric_limits<drake::CppADd>
    : public std::numeric_limits<double> {};

}  // namespace std
