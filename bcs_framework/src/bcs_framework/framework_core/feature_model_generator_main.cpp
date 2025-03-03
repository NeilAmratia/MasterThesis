#include "feature_model_generator.hpp"
#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;

int main() {
    try {
        // Get current path and construct relative paths
        fs::path current_path = fs::current_path();
        fs::path framework_path = current_path / "src" / "bcs_framework";
        fs::path source_dir = framework_path / "base_features";
        fs::path output_dir = framework_path / "generated_feature_model";
        fs::path output_file = output_dir / "feature_model.xml";

        // Create output directory if it doesn't exist
        if (!fs::exists(output_dir)) {
            fs::create_directories(output_dir);
        }

        FeatureModelGenerator generator(source_dir);
        if (generator.generateFeatureModel(output_file.string())) {
            std::cout << "Feature model generated at: " << output_file << std::endl;
            return 0;
        }
        return 1;
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Filesystem error: " << e.what() << std::endl;
        return 1;
    }
}