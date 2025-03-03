#include "ConfigParser.hpp"
#include <iostream>

std::set<std::string> ConfigParser::parseActiveFeatures(const std::string& xmlPath) {
    std::set<std::string> activeFeatures;
    pugi::xml_document doc;
    
    pugi::xml_parse_result result = doc.load_file(xmlPath.c_str());
    if (!result) {
        std::cerr << "Failed to parse XML: " << result.description() << std::endl;
        return activeFeatures;
    }
    
    pugi::xml_node config = doc.child("configuration");
    for (pugi::xml_node feature : config.children("feature")) {
        std::string name = feature.attribute("name").value();
        
        if (!name.empty() && isFeatureSelected(feature)) {
            activeFeatures.insert(name);
        }
    }
    
    return activeFeatures;
}

bool ConfigParser::isFeatureSelected(const pugi::xml_node& node) {
    // Check for both manual and automatic attributes
    if (node.attribute("manual")) {
        return std::string(node.attribute("manual").value()) == "selected";
    }
    else if (node.attribute("automatic")) {
        return std::string(node.attribute("automatic").value()) == "selected";
    }
    return false;
}