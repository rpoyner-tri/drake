#pragma once

#include <string>
#include <algorithm>

/*** getCommandLineOption
 * @brief Provides a platform-independent way to parse command line options
 *(getopt is not available on msvc)
 *
 * Example usage:
 * const char * filename = getCommandLineOption(argv, argv + argc, "-f");
 * if (filename) { ... }
 *
 * See also commandLineOptionExists
 */
const char* const getCommandLineOption(const char* const * begin,
                                       const char* const * end,
                                       const std::string& option) {
  const char* const * itr = std::find(begin, end, option);
  if (itr != end && itr + 1 != end) {
    return *(itr + 1);
  }
  return 0;
}

/*** commandLineOptionExists
 * @brief Provides a platform-independent way to parse command line options
 *(getopt is not available on msvc)
 *
 * Example usage:
 * if (commandLineOptionExists(argv, argv+argc, "-h")) { ... }
 *
 * See also getCommandLineOption
 */

bool commandLineOptionExists(const char* const * begin,
                             const char* const * end,
                             const std::string& option) {
  return std::find(begin, end, option) != end;
}
