#pragma once

#include <set>
#include <string>
#include <filesystem>
#include <pugixml.hpp>

class ConfigParser {
public:
    ConfigParser() = default;
    ~ConfigParser() = default;

    std::set<std::string> parseActiveFeatures(const std::string& xmlPath);

private:
    bool isFeatureSelected(const pugi::xml_node& node);
};