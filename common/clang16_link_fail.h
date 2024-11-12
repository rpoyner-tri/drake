#pragma once

#include <vector>

#include "drake/common/eigen_types.h"

template <typename T>
std::vector<drake::MatrixX<T>> EigenToStdVector(
    const Eigen::Ref<const drake::MatrixX<T>>& mat);

