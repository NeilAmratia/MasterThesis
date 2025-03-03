#include "feature_config.hpp"
#include <iostream>
#include <algorithm>

bool FeatureConfiguration::parseConfig() {

    // Check if file exists
    if (!std::filesystem::exists(m_configFile)) {
        std::cerr << "Config file does not exist: " << m_configFile << std::endl;
        return false;
    }

    try {
        toml::table config = toml::parse_file(m_configFile);
        
        // Parse feature delta files section
        if (auto deltaFiles = config["feature_delta_files"].as_table()) {
            for (const auto& [feature, filePath] : *deltaFiles) {
                if (auto path = filePath.as_string()) {
                    m_featureDeltaFiles[std::string(feature)] = path->get();
                }
            }
        }

        // // Parse feature dependencies
        // if (auto deps = config["feature_dependencies"].as_array()) {
        //     for (const auto& depNode : *deps) {
        //         if (auto depTable = depNode.as_table()) {
        //             if (auto feature = depTable->get_as<std::string>("feature")) {
        //                 std::vector<std::string> requires;
        //                 if (auto reqArray = depTable->get_as<toml::array>("requires")) {
        //                     for (const auto& reqNode : *reqArray) {
        //                         if (auto reqStr = reqNode.value<std::string>()) {
        //                             requires.push_back(*reqStr);
        //                         }
        //                     }
        //                 }
        //                 m_featureDependencies[feature->get()] = requires;
        //             }
        //         }
        //     }
        // }
        
        return true;
    }
    catch (const toml::parse_error& err) {
        std::cerr << "Failed to parse feature config TOML: " << err.description() << "\n"
                  << "Error at " << err.source().begin << std::endl;
        return false;
    }
    catch (const std::exception& err) {
        std::cerr << "Error processing feature config: " << err.what() << std::endl;
        return false;
    }
}

bool FeatureConfiguration::canActivateFeature(
    const std::string& feature,
    const std::vector<std::string>& activeFeatures) const {
    
    // Find dependencies for this feature
    auto depIt = m_featureDependencies.find(feature);
    
    // If no dependencies defined, feature can be activated
    if (depIt == m_featureDependencies.end()) {
        return true;
    }
    
    // Check if all required features are active
    for (const auto& requiredFeature : depIt->second) {
        if (std::find(activeFeatures.begin(),
                      activeFeatures.end(),
                      requiredFeature) == activeFeatures.end()) {
            return false;
        }
    }
    
    return true;
}

std::string FeatureConfiguration::getFeatureDeltaFile(const std::string& feature) const {
    auto it = m_featureDeltaFiles.find(feature);
    if (it != m_featureDeltaFiles.end()) {
        // Get relative path from config
        std::string relativePath = it->second;
        
        // Remove leading slash if present
        if (!relativePath.empty() && relativePath[0] == '/') {
            relativePath = relativePath.substr(1);
        }
        
        // Construct full path relative to project root
        std::filesystem::path deltaPath = m_projectRoot / relativePath;

        // Verify file exists
        if (std::filesystem::exists(deltaPath)) {
            return deltaPath.string();
        } else {
            std::cerr << "Delta file not found: " << deltaPath << std::endl;
        }
    } else {
        std::cerr << "No delta file configured for feature: " << feature << std::endl;
    }
    return "";
}

std::vector<FeatureDependency> FeatureConfiguration::getAllDependencies() const {
    std::vector<FeatureDependency> result;
    for (const auto& [feature, requires] : m_featureDependencies) {
        result.push_back({feature, requires});
    }
    return result;
}