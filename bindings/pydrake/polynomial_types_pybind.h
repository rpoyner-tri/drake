#pragma once

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/polynomial.h"

#if 0  // XXX porting width in bits exceeds 255
DRAKE_NANOBIND_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::Polynomial<double>)
DRAKE_NANOBIND_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::Polynomial<drake::AutoDiffXd>)
DRAKE_NANOBIND_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::Polynomial<drake::symbolic::Expression>)
#endif  // XXX porting
