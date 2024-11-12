#include <vector>
#include "drake/common/eigen_types.h"

// This redundant include fixes the symptom.
// #include "drake/common/fmt_eigen.h"

// If this declaration is before fmt_eigen.h, the symptom happens.
template <typename T>
std::vector<drake::MatrixX<T>> CouldBeAnything(
    const Eigen::Ref<const drake::MatrixX<T>>& mat);

// Normally, we get fmt_eigen.h from here.
#include "drake/common/test_utilities/eigen_matrix_compare.h"

int main(int argc, const char* argv[]) {
  Eigen::MatrixXd A(3, 4);
  A << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12;
  Eigen::MatrixXd mat = A;
  auto stuff = drake::CompareMatrices(mat, A);
}
