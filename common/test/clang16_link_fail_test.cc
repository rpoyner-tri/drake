#include "drake/common/clang16_link_fail.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

GTEST_TEST(Clang16LinkFailTest, LinkFail) {
  Eigen::MatrixXd A(3, 4);
  A << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12;
  Eigen::MatrixXd mat = A;
  EXPECT_TRUE(drake::CompareMatrices(mat, A));
}
