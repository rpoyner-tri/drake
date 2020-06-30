#include <string>
#include <vector>


namespace drake {
namespace tmp {
// Replacement for parts of difflib.
class Diff {
 public:
  Diff() {}
  std::vector<std::string> compare(
      const std::vector<std::string>& a, const std::vector<std::string>& b) const;
 private:
  std::vector<std::string> dump(
      const std::string& tag,
      const std::vector<std::string>& lines,
      size_t lo, size_t hi) const;

  std::vector<std::string> plain_replace(
      const std::vector<std::string>& a, size_t alo, size_t ahi,
      const std::vector<std::string>& b, size_t blo, size_t bhi) const;

  std::vector<std::string> fancy_replace(
      const std::vector<std::string>& a, size_t alo, size_t ahi,
      const std::vector<std::string>& b, size_t blo, size_t bhi) const;

  std::vector<std::string> fancy_helper(
      const std::vector<std::string>& a, size_t alo, size_t ahi,
      const std::vector<std::string>& b, size_t blo, size_t bhi) const;

  std::vector<std::string> qformat(std::string aline, std::string bline,
                                   std::string atags, std::string btags) const;
};
}
}
