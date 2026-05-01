#pragma once

#include <vector>

#include "drake/math/partial_permutation.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

struct ReducedMapping {
  math::internal::PartialPermutation velocity_permutation;
  math::internal::PartialPermutation clique_permutation;
  std::vector<math::internal::PartialPermutation> clique_dof_permutations;
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
