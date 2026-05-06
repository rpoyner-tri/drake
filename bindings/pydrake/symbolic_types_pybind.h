#pragma once

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/symbolic/expression.h"
#include "drake/common/symbolic/polynomial.h"
#include "drake/common/symbolic/rational_function.h"

// Whenever we want to cast any array / matrix type of `T` in C++ (e.g.,
// `Eigen::MatrixX<T>`) to a NumPy array, we should have it in the following
// list.
DRAKE_NB_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::symbolic::Expression)
DRAKE_NB_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::symbolic::Formula)
DRAKE_NB_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::symbolic::Monomial)
DRAKE_NB_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::symbolic::Polynomial)
DRAKE_NB_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::symbolic::RationalFunction)
DRAKE_NB_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::symbolic::Variable)

namespace drake {
namespace symbolic {
/* Internal use only. */
class VariableIdPythonAttorney {
 public:
  VariableIdPythonAttorney() = delete;
  static uint64_t hi(const Variable::Id& id) { return id.hi_; }
  static uint64_t lo(const Variable::Id& id) { return id.lo_; }
  static Variable::Id Construct(uint64_t hi, uint64_t lo) {
    // We need to maintain Id's invariant that the low byte of hi_ is the
    // Variable::Type, by rejecting out-of-bounds types.
    const uint8_t var_type = static_cast<uint8_t>(hi);
    if (var_type > static_cast<uint8_t>(Variable::Type::RANDOM_EXPONENTIAL)) {
      throw std::domain_error("Ill-formed Variable::Id");
    }
    Variable::Id result;
    result.hi_ = hi;
    result.lo_ = lo;
    return result;
  }
};
}  // namespace symbolic
}  // namespace drake

namespace nanobind {
namespace detail {
template <>
struct type_caster<drake::symbolic::Variable::Id> {
 public:
  using Attorney = drake::symbolic::VariableIdPythonAttorney;

  NB_TYPE_CASTER(drake::symbolic::Variable::Id, const_name("int"));

  bool from_python(handle src, uint8_t, cleanup_list*) {
    if (!src) {
      return false;
    }

    nanobind::int_ concat;
    try {
      concat = nanobind::cast<nanobind::int_>(src);
    } catch (...) {
      return false;
    }

    const nanobind::object hi_py = concat >> nanobind::int_(64);
    const nanobind::object lo_py = concat & nanobind::int_(~uint64_t{});
    const uint64_t hi = cast<uint64_t>(hi_py);
    const uint64_t lo = cast<uint64_t>(lo_py);
    // N.B. "value" is a magic variable declared by pybind11 where we're
    // supposed to put the loaded result.
    value = Attorney::Construct(hi, lo);

    return true;
  }

  static handle from_cpp(
      const drake::symbolic::Variable::Id& src, rv_policy, cleanup_list*) {
    const nanobind::int_ hi_py{Attorney::hi(src)};
    const nanobind::int_ lo_py{Attorney::lo(src)};
    nanobind::object concat = (hi_py << nanobind::int_(64)) + lo_py;
    return concat.release();
  }
};
}  // namespace detail
}  // namespace nanobind
