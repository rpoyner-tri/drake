#pragma once

#include <filesystem>
#include <string>

#include "drake/common/diagnostic_policy.h"

namespace drake {
namespace multibody {
namespace internal {

int CheckDocumentAgainstRngSchema(
    const drake::internal::DiagnosticPolicy& diagnostic,
    const std::filesystem::path& rng_schema,
    const std::filesystem::path& document);

int CheckDocumentAgainstRncSchemaAsString(
    const drake::internal::DiagnosticPolicy& diagnostic,
    const std::string& rnc_schema_string,
    const std::filesystem::path& document);

}  // namespace internal
}  // namespace multibody
}  // namespace drake

