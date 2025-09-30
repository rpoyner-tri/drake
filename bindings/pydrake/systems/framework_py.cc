#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/framework_py_semantics.h"
#include "drake/bindings/pydrake/systems/framework_py_systems.h"
#include "drake/bindings/pydrake/systems/framework_py_values.h"

namespace drake {
namespace pydrake {

NB_MODULE(framework, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);
  m.doc() = "Bindings for the core Systems framework.";

  // Import autodiff and symbolic modules so that their types are declared for
  // use as template parameters.
  py::module_::import_("pydrake.autodiffutils");
  py::module_::import_("pydrake.symbolic");

  // This list of calls to helpers must remain in topological dependency order.
  DefineFrameworkPyValues(m);
  DefineFrameworkPySemantics(m);
  DefineFrameworkPySystems(m);
  DefineFrameworkDiagramBuilder(m);
}

}  // namespace pydrake
}  // namespace drake
