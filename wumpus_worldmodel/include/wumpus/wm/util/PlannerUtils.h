#pragma once
#include <string>
#include <vector>
#include <fstream>

namespace wumpus
{
namespace wm
{
namespace util
{
class PlannerUtils
{
public:
    /**
    * Helper method to extract the (not nested) parameters from a binary predicate
    * @param str here: result string from a query
    * @param predName name of queried predicate
    * @return vector containing the two predicates or empty vector if predName wasn't found
    */
    static std::pair<std::string, std::string> extractBinaryPredicateParameters(const std::string& str, const std::string& predName)
    {
        std::string x;
        std::string y;
        auto start = str.find(predName);
        if (start == std::string::npos) {
            return std::pair<std::string, std::string>();
        }
        auto sep = str.find(',', start);
        if (sep == std::string::npos) {
            return std::pair<std::string, std::string>();
        }

        auto end = str.find(')');
        x = str.substr(start + predName.length() + 1, sep - start - predName.length() - 1);
        y = str.substr(sep + 1, end - sep - 1);
        return std::pair<std::string, std::string>{x, y};
    }

    /**
     *Helper method to get a simple unary parameter
     */
    static std::string extractUnaryPredicateParameter(const std::string& str)
    {
        auto braceStart = str.find('(');
        auto braceEnd = str.find(')');
        return str.substr(braceStart + 1, braceEnd - braceStart - 1);
    }

    /**
     * Load additional rules to be added to terms in the form of (Extension-)Queries. These files must only contiain the rules, no comments etc.
     * @param filePath (relative to the domain config folder of the workspace)
     * @param ruleContainer
     */
    static void loadAdditionalRules(const std::string& filePath, std::vector<std::string>& ruleContainer)
    {
        auto path2etc = std::getenv("DOMAIN_CONFIG_FOLDER");
        std::ifstream input(std::string(path2etc) + '/' + filePath);
        std::string line;
        while (std::getline(input, line)) {
            ruleContainer.push_back(line);
        }
    }
};
}
}
}
