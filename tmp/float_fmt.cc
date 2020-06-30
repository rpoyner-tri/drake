#include "drake/tmp/float_fmt.h"

#include <fmt/format.h>
#include <fmt/ostream.h>

namespace drake {
namespace tmp {

DEFINE_int64(float_precision, 4,
             "Precision to use when string formatting doubles for comparison");

std::string float_fmt(double x) {
  return fmt::format("{:.{}g}", x, FLAGS_float_precision);
}
}
}
