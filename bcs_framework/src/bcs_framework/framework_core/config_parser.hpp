#ifndef CONFIG_PARSER_HPP
#define CONFIG_PARSER_HPP

#include <string>
#include <vector>

// Function to parse the XML configuration file
bool parseConfigurationFile(
    const std::string& xmlFilePath,
    std::vector<std::string>& requestedFeatures,
    std::vector<std::string> allFeatures
);

#endif // CONFIG_PARSER_HPP
