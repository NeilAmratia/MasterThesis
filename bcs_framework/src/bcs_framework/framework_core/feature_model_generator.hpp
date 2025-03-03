#pragma once
#include <string>
#include <vector>
#include <filesystem>
#include <map>
#include <set>

enum class FeatureType {
    BASE,
    MANDATORY,
    OPTIONAL,
    ALTERNATIVE,
    OR
};

struct Feature {
    std::string name;
    FeatureType type = FeatureType::OPTIONAL;
    std::vector<std::string> requires;
    std::vector<std::string> excludes;
    std::vector<std::string> children;
    std::string parent;
    bool isBase = false;
};

class FeatureModelGenerator {
public:
    explicit FeatureModelGenerator(const std::filesystem::path& root);
    bool generateFeatureModel(const std::string& outputPath);

private:
    void parseSourceFile(const std::filesystem::path& filePath);
    void extractFeatures(const std::string& line, const std::string& currentClass);
    std::vector<std::string> extractFeaturesFromDeltaMarker(const std::string& line);
    bool isValidFeatureName(const std::string& name);
    void determineFeatureRelationships();
    void writeFeatureHierarchy(std::ofstream& output, const Feature& feature, int indent);
    void writeConstraints(std::ofstream& output);
    std::string getFeatureNodeType(FeatureType type);
    void writeXmlOutput(const std::string& outputPath);
    void writeSimpleXmlOutput(const std::string& outputPath);
    
    std::filesystem::path projectRoot;
    std::map<std::string, Feature> features;
    std::set<std::string> featureNames;
};