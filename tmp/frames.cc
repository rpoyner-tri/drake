#include "drake/tmp/frames.h"

#include <functional>
#include <sstream>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "drake/common/drake_assert.h"
#include "drake/tmp/differ.h"
#include "drake/tmp/float_fmt.h"
#include "drake/tmp/text.h"

namespace drake {
namespace tmp {

Frames* Frames::Current::instance_{};

namespace {
std::string text_diff(const std::string& a, const std::string& b) {
  auto a_lines = split(a);
  auto b_lines = split(b);
  Diff diff;
  std::string result;
  if (a_lines.size() == b_lines.size()) {
    for (int k = 0; k < static_cast<int>(a_lines.size()); k++) {
      auto cmp_lines = diff.compare({a_lines[k]}, {b_lines[k]});
      for (const auto& line : cmp_lines) { result += line + "\n"; }
    }
  } else {
    auto cmp_lines = diff.compare(a_lines, b_lines);
    for (const auto& line : cmp_lines) { result += line + "\n"; }
  }
  return result;
}

template <class T>
inline void hash_combine(std::size_t& seed, const T& v)
{
  std::hash<T> hasher;
  seed ^= hasher(v) + 0x9e3779b9 + (seed<<6) + (seed>>2);
}

template<typename It> void hash_range(std::size_t& seed, It first, It last) {
  for(; first != last; ++first)
  {
    hash_combine(seed, *first);
  }
}
}

int Frames::add(double t, const std::string& value) {
  auto index = size();
  times_.push_back(t);
  values_.push_back(value);
  return index;
}

std::tuple<double, std::string> Frames::get(int i) const {
  if (i < 0) {
    i += size();  // Pythonic indexing.
  }
  DRAKE_ASSERT(i >= 0 && i < size());
  return {times_[i], values_[i]};
}

std::optional<int> Frames::first_diff(const Frames& other) const {
  // Return index for both items at first difference, or None.
  DRAKE_ASSERT(size() == other.size());
  for (int i = 0; i < size(); i++) {
    auto [t, value] = get(i);
    auto [t_other, value_other] = other.get(i);
    DRAKE_ASSERT(t == t_other);
    if (value != value_other) {
      return i;
    }
  }
  return {};
}

size_t Frames::hash() const {
  size_t seed = 0;
  hash_range(seed, times_.begin(), times_.end());
  hash_range(seed, values_.begin(), values_.end());
  return seed;
}

std::string Frames::text_diff(const Frames& other) const {
  auto index = first_diff(other);
  DRAKE_ASSERT(!!index);
  std::stringstream ss;
  auto fmt_diff = [&](int index_actual, int index_label) {
    auto [t, value] = get(index_actual);
    auto [_, value_other] = other.get(index_actual);
    _ = {}; // ignore.
    ss << fmt::format("diff[{}], t: {}\n", index_label, float_fmt(t))
       << indent(tmp::text_diff(value, value_other), "  ") << "\n";
  };
  fmt_diff(*index, 0);
  fmt_diff(-1, -1);
  return ss.str();
}
}
}
