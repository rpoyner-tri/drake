#include "drake/bindings/pydrake/common/test/serialize_test_foo_py.h"

#include "drake/bindings/pydrake/common/serialize_pybind.h"

namespace drake {
namespace pydrake {
namespace test {

NB_MODULE(serialize_test_foo, m) {
  py::class_<Foo> cls(m, "Foo");
  DefAttributesUsingSerialize(&cls);
}

}  // namespace test
}  // namespace pydrake
}  // namespace drake
