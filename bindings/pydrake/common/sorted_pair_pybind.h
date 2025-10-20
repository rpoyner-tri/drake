#pragma once

#include <tuple>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/sorted_pair.h"

namespace nanobind {
namespace detail {

// Casts `SortedPair<T>` as `Tuple[T]` comprised of `(first, second)`.
template <typename T>
struct type_caster<drake::SortedPair<T>> {
  using Value = drake::SortedPair<T>;
  using InnerCaster = make_caster<T>;

  template <typename U> using Cast = Value;

  static constexpr auto Name =
      const_name("Tuple[") + type_caster<T>::Name + const_name("]");

  bool from_python(handle src, uint8_t flags, cleanup_list* cleanup) noexcept {
    nanobind::tuple t = nanobind::tuple(src);
    if (t.size() != 2) return false;
    if (!first.from_python(t[0], flags, cleanup) ||
        !second.from_python(t[1], flags, cleanup)) {
      return false;
    }
    return true;
  }

  template <typename U>
  static handle from_cpp(
      U&& value, rv_policy policy, cleanup_list* cleanup) noexcept {
    object out =
        make_tuple(steal(InnerCaster::from_cpp(value.first(), policy, cleanup)),
            steal(InnerCaster::from_cpp(value.second(), policy, cleanup)));
    return out.release();
  }

  template <typename U>
  bool can_cast() const noexcept {
    return first.template can_cast<U>() && second.template can_cast<U>();
  }

  explicit operator Value() {
    return Value(first.operator cast_t<T>(), second.operator cast_t<T>());
  }

  InnerCaster first;
  InnerCaster second;
};

}  // namespace detail
}  // namespace nanobind
