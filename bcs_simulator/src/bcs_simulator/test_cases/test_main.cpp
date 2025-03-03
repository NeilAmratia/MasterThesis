#include <iostream>
#include <string>
#include <filesystem>
#include "TestManager.hpp"

namespace fs = std::filesystem;

fs::path resolveConfigPath(const std::string& inputPath) {
    fs::path currentPath = fs::current_path();
    fs::path frameworkPath = currentPath / "src" / "bcs_simulator";
    fs::path configPath;

    try {
        if (inputPath.empty()) {
            configPath = frameworkPath / "input_config" / "input_configuration.xml";
        } else {
            configPath = frameworkPath / inputPath;
        }
        
        if (!fs::exists(configPath)) {
            throw std::runtime_error("Configuration file not found: " + configPath.string());
        }
        
        return configPath;
    } catch (const fs::filesystem_error& e) {
        throw std::runtime_error("Path error: " + std::string(e.what()));
    }
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::cout << "=== BCS Test Suite v1.0 ===" << std::endl;
    
    std::string inputPath;
    
    if (argc > 1) {
        inputPath = argv[1];
    } else {
        std::cout << "Enter configuration path (or press Enter for default): ";
        std::getline(std::cin, inputPath);
    }

    try {
        fs::path configPath = resolveConfigPath(inputPath);
        auto testManager = std::make_unique<TestManager>();
        
        if (!testManager->loadConfiguration(configPath.string())) {
            throw std::runtime_error("Failed to load configuration");
        }
        
        testManager->startConfiguredTests();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
    

}