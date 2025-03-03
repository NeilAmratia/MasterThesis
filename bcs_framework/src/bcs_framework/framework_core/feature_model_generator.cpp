// feature_model_generator.cpp
#include "feature_model_generator.hpp"
#include <fstream>
#include <iostream>
#include <regex>

FeatureModelGenerator::FeatureModelGenerator(const std::filesystem::path& root) 
    : projectRoot(root) {}

bool FeatureModelGenerator::generateFeatureModel(const std::string& outputPath) {
    try {
        for (const auto& entry : std::filesystem::recursive_directory_iterator(projectRoot)) {
            if (entry.is_regular_file() && 
                (entry.path().extension() == ".hpp" || entry.path().extension() == ".cpp")) {
                parseSourceFile(entry.path());
            }
        }
        writeSimpleXmlOutput(outputPath);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return false;
    }
}

void FeatureModelGenerator::parseSourceFile(const std::filesystem::path& filePath) {
    std::ifstream file(filePath);
    std::string line;
    
    while (std::getline(file, line)) {
        // Extract class names
        if (line.find("class") != std::string::npos && line.find("enum class") == std::string::npos) {
            std::regex classPattern(R"(class\s+(\w+))");
            std::smatch match;
            if (std::regex_search(line, match, classPattern)) {
                featureNames.insert(match[1].str());
            }
        }
        
        // Extract DELTA markers
        if (line.find("DELTA:") != std::string::npos) {
            std::regex deltaPattern(R"(DELTA:\w+:\d+:(\w+))");
            std::string::const_iterator searchStart(line.cbegin());
            std::smatch match;
            while(std::regex_search(searchStart, line.cend(), match, deltaPattern)) {
                if (!match[1].str().empty()) {
                    featureNames.insert(match[1].str());
                }
                searchStart = match.suffix().first;
            }
        }
    }
}

void FeatureModelGenerator::writeSimpleXmlOutput(const std::string& outputPath) {
        std::ofstream output(outputPath);
        output << "<?xml version=\"1.0\" ?>\n";
        output << "<configuration>\n";
        
        for (const auto& name : featureNames) {
            output << "  <feature manual=\"selected\" name=\"" << name << "\"/>\n";
        }
        
        output << "</configuration>\n";
    }

void FeatureModelGenerator::extractFeatures(const std::string& line, const std::string& currentClass) {
    std::vector<std::string> featureNames = extractFeaturesFromDeltaMarker(line);
    
    for(const auto& featureName : featureNames) {
        if (!isValidFeatureName(featureName)) continue;

        // Add or update feature
        if (features.find(featureName) == features.end()) {
            features[featureName] = Feature{featureName};
        }

        auto& feature = features[featureName];

        // Set relationships based on DELTA type
        if (line.find("DELTA:add") != std::string::npos) {
            feature.type = FeatureType::OPTIONAL;
            if (!currentClass.empty()) {
                feature.parent = currentClass;
                features[currentClass].children.push_back(featureName);
            }
        }
        else if (line.find("DELTA:change") != std::string::npos || 
                 line.find("DELTA:delete") != std::string::npos) {
            feature.type = FeatureType::MANDATORY;
            if (!currentClass.empty() && 
                std::find(feature.requires.begin(), feature.requires.end(), currentClass) 
                == feature.requires.end()) {
                feature.requires.push_back(currentClass);
            }
        }
    }
}

std::vector<std::string> FeatureModelGenerator::extractFeaturesFromDeltaMarker(const std::string& line) {
    std::vector<std::string> features;
    std::regex deltaPattern(R"(DELTA:\w+:\d+:(\w+))");
    std::string::const_iterator searchStart(line.cbegin());
    std::smatch match;
    
    while(std::regex_search(searchStart, line.cend(), match, deltaPattern)) {
        features.push_back(match[1].str());
        searchStart = match.suffix().first;
    }
    return features;
}

bool FeatureModelGenerator::isValidFeatureName(const std::string& name) {
    if (name.empty()) return false;
    if (!std::isalpha(name[0])) return false;
    return std::all_of(name.begin(), name.end(), 
        [](char c) { return std::isalnum(c) || c == '_'; });
}

void FeatureModelGenerator::determineFeatureRelationships() {
    // Set DoorSystem as base feature
    if (features.find("DoorSystem") != features.end()) {
        features["DoorSystem"].isBase = true;
        features["DoorSystem"].type = FeatureType::BASE;
    }

    // Determine mandatory features based on dependencies
    for (auto& [name, feature] : features) {
        if (!feature.requires.empty()) {
            feature.type = FeatureType::MANDATORY;
        }
    }
}

void FeatureModelGenerator::writeFeatureHierarchy(std::ofstream& output, const Feature& feature, int indent) {
    std::string spacing(indent * 4, ' ');
    
    // Write feature opening tag with appropriate type
    output << spacing << "<" << getFeatureNodeType(feature.type) << " ";
    output << "name=\"" << feature.name << "\" ";
    output << "mandatory=\"" << (feature.type == FeatureType::MANDATORY ? "true" : "false") << "\">\n";
    
    // Write children
    for (const auto& childName : feature.children) {
        if (features.find(childName) != features.end()) {
            writeFeatureHierarchy(output, features[childName], indent + 1);
        }
    }
    
    output << spacing << "</" << getFeatureNodeType(feature.type) << ">\n";
}

void FeatureModelGenerator::writeConstraints(std::ofstream& output) {
    output << "    <constraints>\n";
    
    // Write requires constraints
    for (const auto& [name, feature] : features) {
        for (const auto& req : feature.requires) {
            output << "        <rule>\n";
            output << "            <imp>\n";
            output << "                <var>" << name << "</var>\n";
            output << "                <var>" << req << "</var>\n";
            output << "            </imp>\n";
            output << "        </rule>\n";
        }
    }
    
    output << "    </constraints>\n";
}

std::string FeatureModelGenerator::getFeatureNodeType(FeatureType type) {
    switch(type) {
        case FeatureType::BASE: return "and";
        case FeatureType::ALTERNATIVE: return "alt";
        case FeatureType::OR: return "or";
        default: return "feature";
    }
}


void FeatureModelGenerator::writeXmlOutput(const std::string& outputPath) {
    determineFeatureRelationships();
    
    std::ofstream output(outputPath);
    output << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    output << "<featureModel>\n";
    output << "    <struct>\n";
    
    // Write base feature and hierarchy
    for (const auto& [name, feature] : features) {
        if (feature.isBase) {
            writeFeatureHierarchy(output, feature, 2);
            break;
        }
    }
    
    output << "    </struct>\n";
    writeConstraints(output);
    output << "</featureModel>\n";
}