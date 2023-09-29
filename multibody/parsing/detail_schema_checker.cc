#include "drake/multibody/parsing/detail_schema_checker.h"

#include <cstdio>
#include <cstring>

#include "er.h"

#include "drake/common/unused.h"

extern "C" int xcl_main(int argc, const char** argv);

namespace drake {
namespace multibody {
namespace internal {

namespace {

const drake::internal::DiagnosticPolicy* g_the_policy{};

// TODO(rpoyner-tri): this hook is too low-level. Audit the library to see if
// we can do better.
int drake_er_vprintf(char* format, va_list ap) {
  std::array<char, 4096> buffer{};
  vsnprintf(buffer.data(), buffer.size(), format, ap);
  buffer[buffer.size() - 1] = 0;
  std::string result(buffer.data());
  auto found = result.find("error:");
  if (found==std::string::npos) {
    return result.size();
  }
  g_the_policy->Error(result);
  return result.size();
}

class PrintRedirectGuard {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PrintRedirectGuard)
  PrintRedirectGuard(const drake::internal::DiagnosticPolicy* policy) {
    g_the_policy = policy;
    er_vprintf = &drake_er_vprintf;
  }
  ~PrintRedirectGuard() {
    g_the_policy = nullptr;
  }
};

}

int CheckDocumentAgainstRncSchema(
    const drake::internal::DiagnosticPolicy& diagnostic,
    const std::filesystem::path& rnc_schema,
    const std::filesystem::path& document) {
  PrintRedirectGuard guard(&diagnostic);
  std::array<const char*, 4> args{
    "drake", rnc_schema.c_str(), document.c_str(), nullptr};
  return xcl_main(args.size() - 1, args.data());
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
