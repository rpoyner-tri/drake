#include "drake/bindings/pydrake/ref_cycle_pybind.h"

#include <iostream>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"

using pybind11::handle;
using pybind11::detail::function_call;

namespace drake {
namespace pydrake {
namespace internal {

namespace {
void do_ref_cycle_impl(handle p0, handle p1) {
  if (!p0 || !p1) {
    pybind11::pybind11_fail("Could not activate ref_cycle!");
  }

  if (p1.is_none() || p0.is_none()) {
    return; /* Nothing to keep alive or nothing to be kept alive by */
  }

  // each peer will have a new/updated attribute, containing a set of
  // handles. Insert each into the other's handle set. Create the set first
  // if it is not yet existing.
  auto make_link = [](handle a, handle b) {
    static const char refcycle_peers[] = "_pydrake_ref_cycle_peers";
    handle peers;
    if (hasattr(a, refcycle_peers)) {
      peers = a.attr(refcycle_peers);
    } else {
      peers = PySet_New(nullptr);
      DRAKE_DEMAND(PyType_IS_GC(Py_TYPE(a.ptr())));
      DRAKE_DEMAND(PyType_IS_GC(Py_TYPE(b.ptr())));
      DRAKE_DEMAND(PyType_IS_GC(Py_TYPE(peers.ptr())));
      a.attr(refcycle_peers) = peers;
      Py_DECREF(peers.ptr());  // XXX WTF!?!
    }
    std::cerr << fmt::format("just made peers set: cnt {}\n",
                             Py_REFCNT(peers.ptr()));
    // XXX are ref counts correct here?
    std::cerr << fmt::format("adding peer ref {} to set on {}\n",
                             fmt::ptr(b.ptr()), fmt::ptr(a.ptr()));
    PySet_Add(peers.ptr(), b.ptr());
    std::cerr << fmt::format("done peers set: cnt {}\n",
                             Py_REFCNT(peers.ptr()));
  };
  std::cerr << fmt::format("before: p0cnt {} p1cnt {}\n",
                           Py_REFCNT(p0.ptr()), Py_REFCNT(p1.ptr()));
  make_link(p0, p1);
  make_link(p1, p0);
  std::cerr << fmt::format("after: p0cnt {} p1cnt {}\n",
                           Py_REFCNT(p0.ptr()), Py_REFCNT(p1.ptr()));
}
}  // namespace

void ref_cycle_impl(
    size_t Peer0, size_t Peer1, const function_call& call, handle ret) {
  auto get_arg = [&](size_t n) {
    if (n == 0) {
      return ret;
    }
    if (n == 1 && call.init_self) {
      return call.init_self;
    }
    if (n <= call.args.size()) {
      return call.args[n - 1];
    }
    return handle();
  };

  do_ref_cycle_impl(get_arg(Peer0), get_arg(Peer1));
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
