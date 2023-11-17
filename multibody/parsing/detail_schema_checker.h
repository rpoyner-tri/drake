#pragma once

#include <filesystem>

#include "drake/common/diagnostic_policy.h"

namespace drake {
namespace multibody {
namespace internal {

int CheckDocumentAgainstRngSchema(
    const drake::internal::DiagnosticPolicy& diagnostic,
    const std::filesystem::path& rng_schema,
    const std::filesystem::path& document);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
