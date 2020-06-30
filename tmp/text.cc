#include "drake/tmp/text.h"

#include <sstream>

namespace drake {
namespace tmp {

std::vector<std::string> split( const std::string& text, char delim) {
    std::vector<std::string> result;
    std::stringstream ss(text);
    std::string to;
    while (std::getline(ss, to, delim)) {
      result.push_back(to);
    }
    return result;
}

std::string indent(const std::string& text, const std::string& prefix) {
  std::stringstream ss(text);
  std::string to;
  std::string result;

  while (std::getline(ss, to,'\n')) {
    result += prefix + to + "\n";
  }
  return result;
}

}
}
