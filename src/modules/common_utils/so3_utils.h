#include <string>
#include <vector>
#include <sstream>

namespace common_utils {

/**
 * @brief Splits a string into multiple substrings based on the specified delimiter.
 *
 * @param str The string to be split.
 * @param delimiter The character used as the delimiter.
 * @return A vector containing the split substrings.
 */
inline std::vector<std::string> SplitString(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string item;
    while (std::getline(ss, item, delimiter)) {
        tokens.emplace_back(item);
    }
    return tokens;
}

}  // namespace common_utils 
