#include <string>
#include <iostream>
#include <filesystem>
#include "framework_core/framework_manager.hpp"

int main(int argc, const char** argv) {
    try {
        // Get configuration file path from command line or use default
        std::string configPath = "input_config/input_configuration.xml";
        
        if (argc > 1) {
            configPath = argv[1];
        } else {
            std::cout << "No configuration file specified. Using default: " << configPath << std::endl;
            std::cout << "Usage: ./bcs_framework/input_config/input_configuration.xml" << std::endl;
        }


        // Create framework manager with provided config path
        FrameworkManager framework(configPath);
        
        // Process all features
        return framework.processFeatures();
    }

    catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    }
}