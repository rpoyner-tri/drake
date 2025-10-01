#include <memory>
#include <string>

#include "drake/bindings/pydrake/common/wrap_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace pydrake {
namespace {

// A simple struct with a value.
struct MyValue {
  double value{0.};
  explicit MyValue(double value_in) : value(value_in) {}
};

// A simple struct with a bare pointer member.
struct MyContainerRawPtr {
  const MyValue* member{nullptr};
};

struct MyContainerUniquePtr {
  explicit MyContainerUniquePtr(MyValue member_in, MyValue copyable_member_in)
      : member(new MyValue(member_in)),
        copyable_member(new MyValue(copyable_member_in)) {}
  std::unique_ptr<MyValue> member;
  copyable_unique_ptr<MyValue> copyable_member;
};

struct TypeConversionExample {
  std::string value;
};

// Wrapper for TypeConversionExample.
struct wrapper_type_conversion_exaple {
  using Type = TypeConversionExample;
#if 0   // XXX porting
  static constexpr auto original_name =
      py::detail::const_name("TypeConversionExample");
#endif  // XXX porting
  using WrappedType = std::string;
  static constexpr auto wrapped_name = py::detail::const_name("str");

  static TypeConversionExample unwrap(const std::string& value) {
    return TypeConversionExample{value};
  }
  static std::string wrap(const TypeConversionExample& obj) {
    return obj.value;
  }
};

#if 0   // XXX porting
TypeConversionExample MakeTypeConversionExample() {
  return TypeConversionExample{"hello"};
}

bool CheckTypeConversionExample(const TypeConversionExample& obj) {
  return obj.value == "hello";
}
#endif  // XXX porting

class NotCopyable {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NotCopyable);
  NotCopyable() {}
};

using CallbackNeedsWrapping = std::function<void(const NotCopyable&)>;

CallbackNeedsWrapping FunctionNeedsWrapCallbacks(
    CallbackNeedsWrapping callback) {
  NotCopyable value;
  // Invoke the callback if it's defined.
  if (callback) {
    callback(value);
  }
  // Return original callback.
  return callback;
}

}  // namespace
}  // namespace pydrake
}  // namespace drake

namespace nanobind {
namespace detail {
template <>
struct type_caster<drake::pydrake::TypeConversionExample>
    : public drake::pydrake::internal::type_caster_wrapped<
          drake::pydrake::wrapper_type_conversion_exaple> {};
}  // namespace detail
}  // namespace nanobind

namespace drake {
namespace pydrake {
NB_MODULE(wrap_test_util, m) {
  py::class_<MyValue>(m, "MyValue")
      .def(py::init<double>(), py::arg("value"))
      .def_rw("value", &MyValue::value, py_rvp::reference_internal);

  py::class_<MyContainerRawPtr> my_container(m, "MyContainerRawPtr");
  my_container  // BR
      .def(py::init());
#if 0  // XXX porting
  DefReadWriteKeepAlive(&my_container, "member", &MyContainerRawPtr::member,
      "MyContainerRawPtr doc");
#endif

  py::class_<MyContainerUniquePtr> my_unique(m, "MyContainerUniquePtr");
  my_unique.def(py::init<MyValue, MyValue>(), py::arg("member"),
      py::arg("copyable_member"));
#if 0  // XXX porting
  DefReadUniquePtr(&my_unique, "member", &MyContainerUniquePtr::member,
      "MyContainerUniquePtr doc");
  DefReadUniquePtr(&my_unique, "copyable_member",
      &MyContainerUniquePtr::copyable_member, "MyContainerUniquePtr doc");
#endif

#if 0   // XXX porting
  m.def("MakeTypeConversionExample", &MakeTypeConversionExample);
  m.def("MakeTypeConversionExampleBadRvp", &MakeTypeConversionExample,
      py_rvp::reference);
  m.def("CheckTypeConversionExample", &CheckTypeConversionExample,
      py::arg("obj"));
#endif  // XXX porting

  py::class_<NotCopyable>(m, "NotCopyable")  // BR
      .def(py::init());

  // Using WrapCallbacks() -> Good.
  m.def(
      "FunctionNeedsWrapCallbacks", WrapCallbacks(&FunctionNeedsWrapCallbacks));
  // No use of WrapCallbacks() -> Bad.
  m.def("FunctionNeedsWrapCallbacks_Bad", &FunctionNeedsWrapCallbacks);
}
}  // namespace pydrake
}  // namespace drake
