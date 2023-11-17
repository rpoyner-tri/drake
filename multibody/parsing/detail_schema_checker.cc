#include "drake/multibody/parsing/detail_schema_checker.h"

#include <libxml/parser.h>
#include <libxml/relaxng.h>
#include <libxml/xmlmemory.h>

#include "drake/common/unused.h"

namespace drake {
namespace multibody {
namespace internal {

int CheckDocumentAgainstRngSchema(
    const drake::internal::DiagnosticPolicy& diagnostic,
    const std::filesystem::path& rng_schema,
    const std::filesystem::path& document) {
  unused(diagnostic);
  int status;
  xmlDoc *doc;
  xmlRelaxNGPtr schema;
  xmlRelaxNGValidCtxtPtr validctxt;
  xmlRelaxNGParserCtxtPtr rngparser;

  doc = xmlParseFile(document.c_str());

  rngparser = xmlRelaxNGNewParserCtxt(rng_schema.c_str());
  schema = xmlRelaxNGParse(rngparser);
  validctxt = xmlRelaxNGNewValidCtxt(schema);

  status = xmlRelaxNGValidateDoc(validctxt, doc);
  printf("status == %d\n", status);

  xmlRelaxNGFree(schema);
  xmlRelaxNGFreeValidCtxt(validctxt);
  xmlRelaxNGFreeParserCtxt(rngparser);
  xmlFreeDoc(doc);
  return status;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
