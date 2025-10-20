#include <functional>

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {

NB_MODULE(compatibility_test_util, m) {
  // XXX porting -- can we write this with std::function??
  m.def("invoke_callback", [](py::callable callback) {
    // Trivial callback test.
    callback();
  });
}

}  // namespace pydrake
}  // namespace drake
