#include "drake/bindings/pydrake/common/cpp_param_pybind.h"

namespace drake {
namespace pydrake {

// N.B. py::handle::inc_ref() and ::dec_ref() use the Py_X* variants, implying
// that they are safe to use on nullptr, thus we do not need any
// `ptr != nullptr` checks.
Object::Object(::PyObject* ptr) : ptr_(ptr) {
  inc_ref();
}
Object::~Object() {
  dec_ref();
}
Object::Object(const Object& other) : Object(other.ptr()) {}
Object::Object(Object&& other) {
  ptr_ = other.ptr_;
  other.ptr_ = nullptr;
}
Object& Object::operator=(const Object& other) {
  dec_ref();
  ptr_ = other.ptr();
  inc_ref();
  return *this;
}
Object& Object::operator=(Object&& other) {
  dec_ref();
  ptr_ = other.ptr_;
  other.ptr_ = nullptr;
  return *this;
}

Object Object::Clone() const {
  py::object py_copy = py::module_::import_("copy").attr("deepcopy");
  py::object copied = py_copy(to_pyobject<py::object>());
  return from_pyobject(copied);
}

void Object::inc_ref() {
  py::handle(ptr_).inc_ref();
}
void Object::dec_ref() {
  py::handle(ptr_).dec_ref();
}

namespace internal {
namespace {

// Creates a Python object that should uniquely hash for a primitive C++
// type.
py::object GetPyHash(const std::type_info& tinfo) {
  return py::make_tuple("cpp_type", tinfo.hash_code());
}

// Registers C++ type.
template <typename T>
void RegisterType(
    py::module_ m, py::object param_aliases, const std::string& canonical_str) {
  // Create an object that is a unique hash.
  py::object canonical =
      py::eval(py::str(canonical_str.c_str()), m.attr("__dict__"), m);
  py::list aliases;
  aliases.append(GetPyHash(typeid(T)));
  param_aliases.attr("register")(canonical, aliases);
}

// Registers common C++ types.
void RegisterCommon(py::module_ m, py::object param_aliases) {
  // Make mappings for C++ RTTI to Python types.
  // Unfortunately, this is hard to obtain from `pybind11`.
  RegisterType<bool>(m, param_aliases, "bool");
  RegisterType<std::string>(m, param_aliases, "str");
  RegisterType<double>(m, param_aliases, "float");
  RegisterType<float>(m, param_aliases, "np.float32");
  RegisterType<int>(m, param_aliases, "int");
  RegisterType<int16_t>(m, param_aliases, "np.int16");
  RegisterType<int64_t>(m, param_aliases, "np.int64");
  RegisterType<uint8_t>(m, param_aliases, "np.uint8");
  RegisterType<uint16_t>(m, param_aliases, "np.uint16");
  RegisterType<uint32_t>(m, param_aliases, "np.uint32");
  RegisterType<uint64_t>(m, param_aliases, "np.uint64");
  // For supporting generic Python types.
  RegisterType<Object>(m, param_aliases, "object");
}

}  // namespace

py::object GetParamAliases() {
  py::module_ m = py::module_::import_("pydrake.common.cpp_param");
  py::object param_aliases = m.attr("_param_aliases");
  const char registered_check[] = "_register_common_cpp";
  if (!py::hasattr(m, registered_check)) {
    RegisterCommon(m, param_aliases);
    m.attr(registered_check) = true;
  }
  return param_aliases;
}

py::object GetPyParamScalarImpl(const std::type_info& tinfo) {
  py::object param_aliases = GetParamAliases();
  py::object py_hash = GetPyHash(tinfo);
  if (py::cast<bool>(param_aliases.attr("is_aliased")(py_hash))) {
    // If it's an alias, return the canonical type.
    return param_aliases.attr("get_canonical")(py_hash);
  } else {
    // This type is not aliased. Get the pybind-registered type,
    // erroring out if it's not registered.
    // WARNING: Internal API :(
    auto* info = py::detail::nb_type_lookup(&tinfo);
    if (!info) {
      // TODO(eric.cousineau): Use NiceTypeName::Canonicalize(...Demangle(...))
      // once simpler dependencies are used (or something else is used to
      // justify linking in `libdrake.so`).
      const std::string name = tinfo.name();
      throw std::runtime_error("C++ type is not registered in pybind: " + name);
    }
    py::handle h(reinterpret_cast<PyObject*>(info));
    return py::borrow<py::object>(h);
  }
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
