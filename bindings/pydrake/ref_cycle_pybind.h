#pragma once

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace internal {

template <size_t Peer0, size_t Peer1>
struct ref_cycle {};

void ref_cycle_impl(size_t Peer0, size_t Peer1,
    const pybind11::detail::function_call& call, pybind11::handle ret);

}  // namespace internal
}  // namespace pydrake
}  // namespace drake

namespace pybind11 {
namespace detail {

template <size_t Peer0, size_t Peer1>
struct process_attribute<drake::pydrake::internal::ref_cycle<Peer0, Peer1>>
    : public process_attribute_default<
          drake::pydrake::internal::ref_cycle<Peer0, Peer1>> {
  // NOLINTNEXTLINE(runtime/references)
  static void precall(function_call& call) {
    if constexpr (Peer0 != 0 && Peer1 != 0) {
      drake::pydrake::internal::ref_cycle_impl(Peer0, Peer1, call, handle());
    }
  }
  // NOLINTNEXTLINE(runtime/references)
  static void postcall(function_call& call, handle ret) {
    if constexpr (Peer0 == 0 || Peer1 == 0) {
      drake::pydrake::internal::ref_cycle_impl(Peer0, Peer1, call, ret);
    }
  }
};

}  // namespace detail
}  // namespace pybind11
