#include <string>
#include <vector>

namespace drake {
namespace tmp {

std::vector<std::string> split(const std::string& text, char delim='\n');

// Replacement for python textwrap.indent, sorta.
std::string indent(const std::string& text, const std::string& prefix);

}
}
