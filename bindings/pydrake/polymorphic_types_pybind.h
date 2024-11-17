#pragma once

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/drake_assert.h"

// Specialize pybind11's polymorphic_type_hook to tolerate ODR violations. This
// is required by Xcode 16 and later.
namespace pybind11 {

template <typename itype>
struct polymorphic_type_hook<itype,
    std::enable_if_t<std::is_polymorphic<itype>::value>> {
  static const void* get(const itype* src, const std::type_info*& type) {
    DRAKE_DEMAND(false);
    type = src ? &typeid(*src) : nullptr;
    return dynamic_cast<const void*>(src);
  }
};

}  // namespace pybind11
