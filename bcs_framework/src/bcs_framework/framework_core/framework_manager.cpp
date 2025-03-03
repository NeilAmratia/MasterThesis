#include "framework_manager.hpp"
#include "config_parser.hpp"
#include "delta_processor.hpp"
#include "feature_config.hpp"
#include <iostream>
#include <filesystem>

FrameworkManager::FrameworkManager(const std::string& xml_config_path) {
    project_root = getProjectRoot();
    xml_config_path_ = fs::absolute(project_root / xml_config_path).string();
    
    if (!fs::exists(xml_config_path_)) {
        throw std::runtime_error("XML config file not found at: " + xml_config_path_);
    }

    output_path_ = fs::absolute(xml_config_path_);
}

fs::path FrameworkManager::getProjectRoot() const {
    fs::path current_path = fs::current_path();
    fs::path framework_path = current_path / "src" / "bcs_framework";
    
    if (!fs::exists(framework_path)) {
        throw std::runtime_error("Framework directory not found at: " + framework_path.string());
    }
    
    bool has_base_features = fs::exists(framework_path / "base_features");
    bool has_feature_configs = fs::exists(framework_path / "feature_configs");
    
    if (!has_base_features || !has_feature_configs) {
        throw std::runtime_error("Required directories not found in framework directory");
    }

    return framework_path;
}

FrameworkManager::FeatureFiles FrameworkManager::getFeatureFiles(
    const fs::path& project_root, 
    const std::string& feature_name) const {
    
    FeatureFiles files;
    std::string cpp_path = fs::absolute(project_root / "base_features" / (feature_name + ".cpp")).string();
    std::string hpp_path = fs::absolute(project_root / "base_features" / (feature_name + ".hpp")).string();

    if (!fs::exists(cpp_path)) {
        throw std::runtime_error("CPP file not found for feature: " + feature_name);
    }
    if (!fs::exists(hpp_path)) {
        throw std::runtime_error("HPP file not found for feature: " + feature_name);
    }

    files.source_file = cpp_path + ";" + hpp_path;
    files.config_file = fs::absolute(project_root / "feature_configs" / (feature_name + "_config.toml")).string();

    if (!fs::exists(files.config_file)) {
        throw std::runtime_error("Config file not found for feature: " + feature_name);
    }
    
    return files;
}

int FrameworkManager::processFeature(
    const FeatureFiles& files,
    const std::vector<std::string>& requestedFeatures,
    const std::vector<std::string>& allFeatures) const {


    // Process single source file
    std::vector<std::string> source_paths = {files.source_file};

    clang::tooling::FixedCompilationDatabase Compilations(".", {});
    clang::tooling::ClangTool Tool(Compilations, source_paths);
    
    auto ActionFactory = std::make_unique<DeltaProcessingActionFactory>(
        project_root,
        files.source_file,
        files.config_file,
        requestedFeatures,
        allFeatures 
    );
    
    return Tool.run(ActionFactory.get());
}

int FrameworkManager::processFeatures() {
    try {
        // Parse configuration to get active features
        parseConfigurationFile(xml_config_path_, requested_features_,all_features_);
        project_root = getProjectRoot();

        // Print project root path
        std::cout << "Project root path: " << project_root << std::endl;
        
        // Map to store feature files: <feature_name, <source_files, config_file>>
        std::map<std::string, std::pair<std::vector<std::string>, std::string>> feature_map;
        
        // Scan base_features directory
        fs::path base_features_dir = project_root / "base_features";
        fs::path config_dir = project_root / "feature_configs";
        
        // First, collect all files for requested features
        for (const auto& entry : fs::directory_iterator(base_features_dir)) {
            if (entry.path().extension() == ".hpp" || entry.path().extension() == ".cpp") {
                std::string feature_name = entry.path().stem().string();
                
                // Check if this feature was requested
                if (std::find(requested_features_.begin(), requested_features_.end(), 
                    feature_name) != requested_features_.end()) {
                    
                    // Get or create the feature entry
                    auto& feature_entry = feature_map[feature_name];
                    
                    // Add source file to the vector
                    feature_entry.first.push_back(entry.path().string());
                    
                    // Set config file path if not already set
                    if (feature_entry.second.empty()) {
                        std::string config_path = (config_dir / (feature_name + ".toml")).string();
                        if (fs::exists(config_path)) {
                            feature_entry.second = config_path;
                        } else {
                            std::cerr << "Warning: Missing config file for feature " 
                                     << feature_name << ": " << config_path << std::endl;
                            // Remove this feature from the map as it's invalid without config
                            feature_map.erase(feature_name);
                            continue;
                        }
                    }
                }
            }
        }

        // Process each feature's files one by one
        int result = 0;
        for (const auto& [feature_name, files] : feature_map) {
            const auto& [source_files, config_file] = files;
            
            // Process each source file individually
            for (const auto& source_file : source_files) {
                try {
                    FeatureFiles current_files{
                        .source_file = source_file,
                        .config_file = config_file
                    };
                    
                    int fileResult = processFeature(current_files, requested_features_, all_features_);
                    if (fileResult != 0) {
                        result = fileResult;
                        std::cerr << "Error processing file: " << source_file << std::endl;
                    }
                }
                catch (const std::exception& e) {
                    std::cerr << "Error processing file " << source_file 
                             << ": " << e.what() << std::endl;
                    result = 1;
                }
            }
        }
        
        return result;
    }
    catch (const std::exception& e) {
        std::cerr << "Fatal error in processFeatures: " << e.what() << std::endl;
        return 1;
    }
}