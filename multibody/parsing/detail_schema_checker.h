#pragma once

#include <filesystem>

#include "drake/common/diagnostic_policy.h"

namespace drake {
namespace multibody {
namespace internal {

int CheckDocumentAgainstRncSchema(
    const drake::internal::DiagnosticPolicy& diagnostic,
    const std::filesystem::path& rnc_schema,
    const std::filesystem::path& document);

}  // namespace internal
}  // namespace multibody
}  // namespace drake

