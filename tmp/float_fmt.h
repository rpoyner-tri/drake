#include <string>

#include <gflags/gflags.h>

namespace drake {
namespace tmp {

DECLARE_int64(float_precision);

std::string float_fmt(double x);

template <typename T>
std::string vec_fmt(const T& v) {
  std::string result("[");
  result += float_fmt(v(0));
  for (int k = 1; k < v.size(); k++) {
    result += " ";
    result += float_fmt(v(k));
  }
  result += "]";
  return result;
}

}
}
