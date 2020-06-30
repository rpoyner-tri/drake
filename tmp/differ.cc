#include "drake/tmp/differ.h"

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "drake/tmp/third-party/difflib/src/difflib.h"

#include "drake/common/drake_assert.h"

namespace {
template <typename T>
void append(std::vector<T>* dst, const std::vector<T>& src) {
  dst->reserve(dst->size() + src.size());
  for (const auto& x : src) { dst->push_back(x); }
};
}

namespace drake {
namespace tmp {

std::vector<std::string> Diff::compare(
    const std::vector<std::string>& a, const std::vector<std::string>& b) const {
  auto cruncher = difflib::MakeSequenceMatcher(a, b);
  std::vector<std::string> results;
  for (const auto& opcode : cruncher.get_opcodes()) {
    auto [tag, alo, ahi, blo, bhi] = opcode;
    if (tag == "replace") {
      append(&results, fancy_replace(a, alo, ahi, b, blo, bhi));
    } else if (tag == "delete") {
      append(&results, dump("-", a, alo, ahi));
    } else if (tag == "insert") {
      append(&results, dump("+", b, blo, bhi));
    } else if (tag == "equal") {
      append(&results, dump(" ", a, alo, ahi));
    } else {
      throw std::runtime_error("unknown compare opcode tag");
    }
  }
  return results;
}

std::vector<std::string> Diff::dump(
    const std::string& tag,
    const std::vector<std::string>& lines,
    size_t lo, size_t hi) const {
  std::vector<std::string> results;
  for (size_t k = lo;  k < hi; k++) {
    results.push_back(fmt::format("{} {}", tag, lines[k]));
  }
  return results;
}

std::vector<std::string> Diff::plain_replace(
    const std::vector<std::string>& a, size_t alo, size_t ahi,
    const std::vector<std::string>& b, size_t blo, size_t bhi) const {
  DRAKE_ASSERT(alo < ahi && blo < bhi);
  std::vector<std::string> results;
  if (bhi - blo < ahi - alo) {
    append(&results, dump("+", b, blo, bhi));
    append(&results, dump("-", a, alo, ahi));
  } else {
    append(&results, dump("-", a, alo, ahi));
    append(&results, dump("+", b, blo, bhi));
  }
  return results;
}

std::vector<std::string> Diff::fancy_replace(
    const std::vector<std::string>& a, size_t alo, size_t ahi,
    const std::vector<std::string>& b, size_t blo, size_t bhi) const {
  std::vector<std::string> results;
  double best_ratio = 0.74;
  double cutoff = 0.75;
  auto cruncher = difflib::MakeSequenceMatcher(std::string(), std::string());
  std::optional<size_t> eqi, eqj;
  size_t best_i{}, best_j{};
  for (size_t j = blo; j < bhi; j++ ) {
    const auto& bj = b[j];
    cruncher.set_seq2(bj);
    for (size_t i = alo; i < ahi; i++) {
      const auto& ai = a[i];
      if (ai == bj) {
        if (!eqi) {
          eqi = i;
          eqj = j;
        }
        continue;
      }
      cruncher.set_seq1(ai);
      if (cruncher.ratio() > best_ratio) {
        best_ratio = cruncher.ratio();
        best_i = i;
        best_j = j;
      }
    }
  }
  if (best_ratio < cutoff) {
    if (eqi) {
      return plain_replace(a, alo, ahi, b, blo, bhi);
    }
    best_i = *eqi;
    best_j = *eqj;
    best_ratio = 1.0;
  } else {
    eqi = std::nullopt;
  }
  append(&results, fancy_helper(a, alo, best_i, b, blo, best_j));
  auto aelt = a[best_i];
  auto belt = b[best_j];
  if (!eqi) {
    // # pump out a '-', '?', '+', '?' quad for the synched lines
    std::string atags, btags;
    cruncher.set_seq(aelt, belt);
    for (const auto& opcode : cruncher.get_opcodes()) {
      auto [tag, ai1, ai2, bj1, bj2] = opcode;
      size_t la = ai2 - ai1;
      size_t lb = bj2 - bj1;
      if (tag == "replace") {
        atags += std::string(la, '^');
        btags += std::string(lb, '^');
      } else if (tag == "delete") {
        atags += std::string(la, '-');
      } else if (tag == "insert") {
        btags += std::string(lb, '+');
      } else if (tag == "equal") {
        atags += std::string(la, ' ');
        btags += std::string(lb, ' ');
      } else {
        throw std::runtime_error("unknown tag in fancy_replace");
      }
    }
    append(&results, qformat(aelt, belt, atags, btags));
  } else {
    results.push_back(std::string("  ") + aelt);
  }
  append(&results, fancy_helper(a, best_i + 1, ahi, b, best_j + 1, bhi));
  return results;
}

std::vector<std::string> Diff::fancy_helper(
    const std::vector<std::string>& a, size_t alo, size_t ahi,
    const std::vector<std::string>& b, size_t blo, size_t bhi) const {
  std::vector<std::string> results;
  if (alo < ahi) {
    if (blo < bhi) {
      append(&results, fancy_replace(a, alo, ahi, b, blo, bhi));
    } else {
      append(&results, dump("-", a, alo, ahi));
    }
  } else if (blo < bhi) {
    append(&results, dump("+", b, blo, bhi));
  }
  return results;
}

std::vector<std::string> Diff::qformat(std::string aline, std::string bline,
                                 std::string atags, std::string btags) const {
  std::vector<std::string> results;
  auto count_leading = [](const std::string& line, char c) {
    size_t i = 0;
    size_t n = line.size();
    while (i < n && line[i] == c) {
      i++;
    }
    return i;
  };
  auto rtrim = [](const std::string& s) {
    size_t last = s.find_last_not_of(" \t");
    return s.substr(0, (last + 1));
  };
  size_t common = std::min(count_leading(aline, '\t'),
                           count_leading(bline, '\t'));
  common = std::min(common, count_leading(atags.substr(0, common), ' '));
  common = std::min(common, count_leading(btags.substr(0, common), ' '));
  atags = rtrim(atags.substr(common));
  btags = rtrim(btags.substr(common));
  results.push_back(std::string("- ") + aline);
  if (!atags.empty()) {
    results.push_back(fmt::format("? {}{}\n", std::string(common, '\t'), atags));
  }
  results.push_back(std::string("- ") + bline);
  if (!btags.empty()) {
    results.push_back(fmt::format("? {}{}\n", std::string(common, '\t'), btags));
  }
  return results;
}

}
}

