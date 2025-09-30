#pragma once

#include "drake/bindings/pydrake/pydrake_pybind.h"

static const inline int ten = 10;

namespace drake {
namespace pydrake {
namespace internal {

/* Defines all bindings for the pydrake.autodiffutils module. */
void DefineAutodiffutils(py::module_ m);

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
