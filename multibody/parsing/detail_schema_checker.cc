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

  // Received a structured error from a libxml2 parser.
  // @param userData   the "context" associated with this callback.
  // @param error      the error description struct from the parser.
  // @pre userData must be a pointer to an `ErrorHandler` instance.
  //
  // Note: this entry point is designed to match the `xmlStructuredErrorFunc`
  // type required by libxml2. The precondition is not checked (nor is it
  // checkable); mistakes will result in undefined behavior (most likely
  // segfault).
  static void ReceiveStructuredError(void* userData, xmlErrorPtr error) {
    DRAKE_DEMAND(userData != nullptr);
    ErrorHandler* handler = static_cast<ErrorHandler*>(userData);
    handler->Receive(error);
  }

 private:
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

  DiagnosticDetail Convert(xmlErrorPtr error) {
    DRAKE_DEMAND(error != nullptr);
    DRAKE_DEMAND(error->message != nullptr);
    DiagnosticDetail detail;
    detail.filename = std::string(error->file == nullptr ? "" : error->file);
    detail.line = error->line;
    detail.message = std::string(error->message);
    return detail;
  }

  const DiagnosticPolicy& diagnostic_;
};


int CheckDocumentAgainstRngParser(
    const DiagnosticPolicy& diagnostic,
    xmlRelaxNGParserCtxtPtr rngparser,
    const std::filesystem::path& document) {
  DRAKE_DEMAND(rngparser != nullptr);

  ErrorHandler error_handler(diagnostic);

  xmlRelaxNGSetParserStructuredErrors(
      rngparser, ErrorHandler::ReceiveStructuredError, &error_handler);

  xmlDoc *doc = xmlParseFile(document.c_str());
  ScopeExit doc_guard([&doc]() { xmlFreeDoc(doc); });

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

}  // namespace

int CheckDocumentAgainstRngSchema(
    const DiagnosticPolicy& diagnostic,
    const std::filesystem::path& rng_schema,
    const std::filesystem::path& document) {
  xmlRelaxNGParserCtxtPtr rngparser =
      xmlRelaxNGNewParserCtxt(rng_schema.c_str());
  ScopeExit rngparser_guard([&rngparser]() {
    xmlRelaxNGFreeParserCtxt(rngparser);
  });
  return CheckDocumentAgainstRngParser(diagnostic, rngparser, document);
}

int CheckDocumentAgainstRngSchemaAsString(
    const DiagnosticPolicy& diagnostic,
    const std::string& rng_schema_string,
    const std::filesystem::path& document) {
  xmlRelaxNGParserCtxtPtr rngparser =
      xmlRelaxNGNewMemParserCtxt(rng_schema_string.c_str(),
                                 rng_schema_string.size());
  ScopeExit rngparser_guard([&rngparser]() {
    xmlRelaxNGFreeParserCtxt(rngparser);
  });
  return CheckDocumentAgainstRngParser(diagnostic, rngparser, document);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
