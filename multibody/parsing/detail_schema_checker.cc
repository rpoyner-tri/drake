#include "drake/multibody/parsing/detail_schema_checker.h"

#include <libxml++/validators/relaxngvalidator.h>

namespace drake {
namespace multibody {
namespace internal {

int CheckDocumentAgainstRncSchema(
    const drake::internal::DiagnosticPolicy& diagnostic,
    const std::filesystem::path& rnc_schema,
    const std::filesystem::path& document) {
  return 0;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
