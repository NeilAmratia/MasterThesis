#include "config_parser.hpp"
#include <tinyxml2.h>
#include <iostream>

bool parseConfigurationFile(
    const std::string& xmlFilePath,
    std::vector<std::string>& requestedFeatures,
    std::vector<std::string> allFeatures
) {
    // Clear existing data
    requestedFeatures.clear();
    allFeatures.clear();

    tinyxml2::XMLDocument doc;

    // Load the XML file
    if (doc.LoadFile(xmlFilePath.c_str()) != tinyxml2::XML_SUCCESS) {
        std::cerr << "Error: Could not load XML file: " << xmlFilePath << std::endl;
        return false;
    }

    // Get the root element
    tinyxml2::XMLElement* root = doc.FirstChildElement("configuration");
    if (!root) {
        std::cerr << "Error: Missing <configuration> root element." << std::endl;
        return false;
    }

    // Iterate through all <feature> elements
    for (tinyxml2::XMLElement* feature = root->FirstChildElement("feature");
         feature != nullptr;
         feature = feature->NextSiblingElement("feature")) {
        const char* name = feature->Attribute("name");
        const char* active = feature->Attribute("manual");
        if (!active) {
            active = feature->Attribute("auto");
        }

        if (!name || !active) {
            std::cerr << "Warning: <feature> element missing 'name' or 'sampling type' attribute." << std::endl;
            continue;
        }

        std::string featureName(name);
        bool isActive = (std::string(active) == "selected");

        // Add to all features list
        allFeatures.push_back(featureName);

        // Add to the appropriate list
        if (isActive) {
            requestedFeatures.push_back(featureName);
        }
    }

    // Add debug output for requested features
    std::cout << "\nAll Features (" << allFeatures.size() << " total):" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    for (const auto& feature : allFeatures) {
        std::cout << "  - " << feature << std::endl;
    }
    std::cout << "\nRequested Features (" << requestedFeatures.size() << " total):" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    for (const auto& feature : requestedFeatures) {
        std::cout << "  - " << feature << std::endl;
    }
    std::cout << "----------------------------------------" << std::endl;

    return true;
}
