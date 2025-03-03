#pragma once

#include <string>
#include <vector>
#include <filesystem>
#include <clang/Tooling/CommonOptionsParser.h>
#include <clang/Tooling/Tooling.h>

namespace fs = std::filesystem;

class FrameworkManager {
public:
    // Constructor
    FrameworkManager(const std::string& xml_config_path);
    ~FrameworkManager() = default;

    // Main processing function
    int processFeatures();

    // Make getProjectRoot non-static and const
    fs::path getProjectRoot() const;
    fs::path getOutputPath() const { return output_path_; }

private:
    struct FeatureFiles {
        std::string source_file;
        std::string config_file;
    };

    FeatureFiles getFeatureFiles(const fs::path& project_root, const std::string& feature_name) const;
    int processFeature(
    const FeatureFiles& files, 
    const std::vector<std::string>& requestedFeatures,
    const std::vector<std::string>& allFeatures) const;
    fs::path project_root;
    fs::path output_path_; 
    std::string xml_config_path_;
    std::vector<std::string> all_features_;
    std::vector<std::string> requested_features_;
};