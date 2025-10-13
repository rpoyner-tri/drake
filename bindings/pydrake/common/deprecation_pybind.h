#pragma once

/// @file
/// Provides access to Python deprecation utilities from C++.
/// For example usages, please see `deprecation_example/cc_module_py.cc`.

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "drake/bindings/pydrake/common/wrap_function.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {

// N.B. We cannot use `py::str date = py::none()` because the pybind Python C++
// API converts `None` to a string.
// See: https://github.com/pybind/pybind11/issues/2361

/// Deprecates an attribute `name` of a class `cls`.
/// This *only* works with class attributes (unbound members or methods) as it
/// is implemented with a Python property descriptor.
inline void DeprecateAttribute(py::object cls, py::str name, py::str message,
    std::optional<std::string> date = {}) {
  py::object deprecated =
      py::module_::import_("pydrake.common.deprecation").attr("deprecated");
  py::object original = cls.attr(name);
  cls.attr(name) = deprecated(message, py::arg("date") = date)(original);
}

/// Raises a deprecation warning.
///
/// @note If you are deprecating a class's member or method, please use
/// `DeprecateAttribute` so that the warning is issued immediately when
/// accessed, not only when it is called.
inline void WarnDeprecated(
    const std::string& message, std::optional<std::string> date = {}) {
  py::gil_scoped_acquire guard;
  py::object warn_deprecated =
      py::module_::import_("pydrake.common.deprecation")
          .attr("_warn_deprecated");
  warn_deprecated(message, py::arg("date") = date);
}

namespace internal {

template <typename Func, typename Return, typename... Args>
auto WrapDeprecatedImpl(std::string message,
    function_info<Func, Return, Args...>&& info,
    std::enable_if_t<std::is_same_v<Return, void>, void*> = {}) {
  return [info = std::move(info), message = std::move(message)](Args... args) {
    WarnDeprecated(message);
    info.func(std::forward<Args>(args)...);
  };
}

// N.B. `decltype(auto)` is used in both places to (easily) achieve perfect
// forwarding of the return type.
template <typename Func, typename Return, typename... Args>
decltype(auto) WrapDeprecatedImpl(std::string message,
    function_info<Func, Return, Args...>&& info,
    std::enable_if_t<!std::is_same_v<Return, void>, void*> = {}) {
  return [info = std::move(info), message = std::move(message)](
             Args... args) -> decltype(auto) {
    WarnDeprecated(message);
    return info.func(std::forward<Args>(args)...);
  };
}

}  // namespace internal

/// Wraps any callable (function pointer, method pointer, lambda, etc.) to emit
/// a deprecation message.
template <typename Func>
auto WrapDeprecated(std::string message, Func&& func) {
  return internal::WrapDeprecatedImpl(std::move(message),
      internal::infer_function_info(std::forward<Func>(func)));
}

/// Deprecated wrapping of `py::init<>`.
template <typename CppClass, typename... Args>
struct py_init_deprecated
    : py::def_visitor<py_init_deprecated<CppClass, Args...>> {
  py_init_deprecated(std::string message) : message_(std::move(message)) {}
  template <typename Class, typename... Extra>
  void execute(Class& cl, const Extra&...) {
    cl.def("__init__",
        [message = std::move(message_)](CppClass* self, Args... args) {
          WarnDeprecated(message);
          new (self) CppClass(std::forward<Args>(args)...);
        });
  }
  std::string message_;
};

/// Deprecated wrapping of `py::init(placement-new-callable)`.
/// XXX porting how do we do specialization so we can reuse the name?
template <typename CppClass, typename Func>
struct py_init_deprecated2
    : py::def_visitor<py_init_deprecated2<CppClass, Func>> {
  py_init_deprecated2(std::string message) : message_(std::move(message)) {}
  template <typename Class, typename... Extra>
  void execute(Class& cl, const Extra&...) {
    cl.def("__init__",
        [message = std::move(message_)](CppClass* self, Func&& func) {
          WarnDeprecated(message);
          func(self);
        });
  }
  std::string message_;
};

template <typename CppClass>
struct DeprecatedParamInit : py::def_visitor<DeprecatedParamInit<CppClass>> {
  DeprecatedParamInit(std::string message) : message_(std::move(message)) {}
  template <typename Class, typename... Extra>
  void execute(Class& cl, const Extra&...) {
    cl.def("__init__",
        WrapDeprecated(std::move(message_), [](Class* self, py::kwargs kwargs) {
          new (self) Class();
          py::object py_obj = py::cast(self, py_rvp::reference);
          py::module_::import_("pydrake").attr("_setattr_kwargs")(
              py_obj, kwargs);
        }));
  }
  std::string message_;
};

}  // namespace pydrake
}  // namespace drake
