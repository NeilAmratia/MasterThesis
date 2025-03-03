#ifndef FEATURE_CONFIG_HPP
#define FEATURE_CONFIG_HPP

#include <string>
#include <vector>
#include <filesystem>
#include <unordered_map>
#include <toml++/toml.h>
namespace fs = std::filesystem;

struct FeatureDependency {
    std::string feature;
    std::vector<std::string> requiredFeatures;
};

class FeatureConfiguration {
public:
    FeatureConfiguration(const fs::path& projectRoot, const std::string& configFile)
        : m_projectRoot(projectRoot), m_configFile(configFile){}

    bool parseConfig();
    
    // Check if a feature can be activated based on its dependencies
    bool canActivateFeature(const std::string& feature,
                           const std::vector<std::string>& activeFeatures) const;
    
    // Get the delta file path for a specific feature
    std::string getFeatureDeltaFile(const std::string& feature) const;
    
    // Get all feature dependencies
    std::vector<FeatureDependency> getAllDependencies() const;

private:
    std::string m_configFile;
    std::unordered_map<std::string, std::string> m_featureDeltaFiles;
    std::unordered_map<std::string, std::vector<std::string>> m_featureDependencies;
    fs::path m_projectRoot;
};

#endif