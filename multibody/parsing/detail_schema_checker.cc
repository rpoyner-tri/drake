#include "drake/multibody/parsing/detail_schema_checker.h"

#include <string>

#include <libxml/parser.h>
#include <libxml/relaxng.h>
#include <libxml/xmlmemory.h>

#include "drake/common/scope_exit.h"

namespace drake {
namespace multibody {
namespace internal {

using drake::internal::DiagnosticDetail;
using drake::internal::DiagnosticPolicy;

namespace {
class ErrorHandler {
 public:
  ErrorHandler(
      const DiagnosticPolicy& diagnostic) : diagnostic_(diagnostic) {}

  static void ReceiveStructuredError(void* userData, xmlErrorPtr error) {
    ErrorHandler* handler = static_cast<ErrorHandler*>(userData);
    handler->Receive(error);
  }

  void Receive(xmlErrorPtr error) {
    DRAKE_DEMAND(error != nullptr);
    switch (error->level) {
      case XML_ERR_NONE: { break; }
      case XML_ERR_WARNING: {
        diagnostic_.Warning(Convert(error));
        break;
      }
      case XML_ERR_ERROR:
      case XML_ERR_FATAL: {
        diagnostic_.Error(Convert(error));
        break;
      }
    }
  }

 private:
  DiagnosticDetail Convert(xmlErrorPtr error) {
    DRAKE_DEMAND(error != nullptr);
    DiagnosticDetail detail;
    detail.filename = std::string(error->file == nullptr ? "" : error->file);
    detail.line = error->line;
    detail.message = std::string(error->message);
    return detail;
  }

  const DiagnosticPolicy& diagnostic_;
};
}  // namespace

int CheckDocumentAgainstRngSchema(
    const DiagnosticPolicy& diagnostic,
    const std::filesystem::path& rng_schema,
    const std::filesystem::path& document) {
  ErrorHandler error_handler(diagnostic);

  xmlDoc *doc = xmlParseFile(document.c_str());
  ScopeExit doc_guard([&doc]() { xmlFreeDoc(doc); });

  xmlRelaxNGParserCtxtPtr rngparser =
      xmlRelaxNGNewParserCtxt(rng_schema.c_str());
  ScopeExit rngparser_guard([&rngparser]() {
    xmlRelaxNGFreeParserCtxt(rngparser);
  });
  xmlRelaxNGSetParserStructuredErrors(
      rngparser, ErrorHandler::ReceiveStructuredError, &error_handler);

  xmlRelaxNGPtr schema = xmlRelaxNGParse(rngparser);
  ScopeExit schema_guard([&schema]() { xmlRelaxNGFree(schema); });

  xmlRelaxNGValidCtxtPtr validctxt = xmlRelaxNGNewValidCtxt(schema);
  ScopeExit validctxt_guard([&validctxt]() {
    xmlRelaxNGFreeValidCtxt(validctxt);
  });
  xmlRelaxNGSetValidStructuredErrors(
      validctxt, ErrorHandler::ReceiveStructuredError, &error_handler);

  int status = xmlRelaxNGValidateDoc(validctxt, doc);

  return status;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
