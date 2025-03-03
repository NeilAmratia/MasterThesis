#include <clang/Frontend/FrontendAction.h>
#include <clang/Frontend/CompilerInstance.h>
#include <clang/Tooling/Tooling.h>
#include <clang/Tooling/CommonOptionsParser.h>
#include <clang/Tooling/CompilationDatabase.h>
#include <clang/AST/RecursiveASTVisitor.h>
#include <clang/AST/ASTContext.h>
#include <clang/Basic/SourceManager.h>
#include <fstream>
#include <map>
#include <set>
#include <vector>
#include <regex>
#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;

using namespace clang;

// Forward declarations
class BaseFeatureVisitor;
class FeatureASTConsumer;
class FeatureAnalyzerAction;

// Basic structures
struct Feature {
    std::string name;
    bool isMandatory = true;
    std::vector<std::string> subFeatures;
    std::vector<std::string> dependencies;
    std::set<std::string> deltaOperations;
    std::string parentClass;
};

struct ClassNode {
    std::string name;
    std::vector<std::string> methods;
    std::vector<std::string> subClasses;
    std::string parentClass;
};

struct DeltaMarker {
    std::string type;
    std::string featureName;
    int lineNumber;
    std::string affectedCode;
};

// AST Visitor class
class BaseFeatureVisitor : public RecursiveASTVisitor<BaseFeatureVisitor> {
public:
    explicit BaseFeatureVisitor(ASTContext *Context) : Context(Context) {}

    bool VisitCXXRecordDecl(CXXRecordDecl *Declaration) {
        if (Declaration->isClass() || Declaration->isStruct()) {
            std::string className = Declaration->getNameAsString();
            ClassNode classNode;
            classNode.name = className;

            if (Declaration->hasDefinition()) {
                for (const auto& base : Declaration->bases()) {
                    if (base.getAccessSpecifier() == AS_public) {
                        classNode.parentClass = base.getType().getAsString();
                        break;
                    }
                }
            }
            
            classHierarchy[className] = classNode;
            
            if (classNode.parentClass.empty()) {
                rootClasses.push_back(className);
            } else {
                classHierarchy[classNode.parentClass].subClasses.push_back(className);
            }
        }
        return true;
    }

    bool VisitCXXMethodDecl(CXXMethodDecl *Declaration) {
        if (Declaration->isUserProvided()) {
            std::string methodName = Declaration->getNameAsString();
            std::string className = Declaration->getParent()->getNameAsString();
            classHierarchy[className].methods.push_back(methodName);

            if (methodName.find("process") != std::string::npos ||
                methodName.find("handle") != std::string::npos ||
                methodName.find("initialize") != std::string::npos) {
                Feature feature;
                feature.name = methodName;
                feature.parentClass = className;
                features[methodName] = feature;
            }
        }
        return true;
    }

    std::map<std::string, ClassNode> getClassHierarchy() const { return classHierarchy; }
    std::vector<std::string> getRootClasses() const { return rootClasses; }
    std::map<std::string, Feature> getFeatures() const { return features; }

private:
    ASTContext *Context;
    std::map<std::string, ClassNode> classHierarchy;
    std::vector<std::string> rootClasses;
    std::map<std::string, Feature> features;
};

// AST Consumer class
class FeatureASTConsumer : public ASTConsumer {
public:
    explicit FeatureASTConsumer(ASTContext* Context, 
                               std::map<std::string, ClassNode>& hierarchy,
                               std::vector<std::string>& rootClasses,
                               std::map<std::string, Feature>& features) 
        : Visitor(Context) {
        Visitor.TraverseDecl(Context->getTranslationUnitDecl());
        hierarchy = Visitor.getClassHierarchy();
        rootClasses = Visitor.getRootClasses();
        features = Visitor.getFeatures();
    }

private:
    BaseFeatureVisitor Visitor;
};

// Frontend Action class
class FeatureAnalyzerAction : public ASTFrontendAction {
public:
    FeatureAnalyzerAction(std::map<std::string, ClassNode>& hierarchy,
                         std::vector<std::string>& rootClasses,
                         std::map<std::string, Feature>& features)
        : hierarchy_(hierarchy), rootClasses_(rootClasses), features_(features) {}

    std::unique_ptr<ASTConsumer> CreateASTConsumer(CompilerInstance& CI, 
                                                  StringRef file) override {
        return std::make_unique<FeatureASTConsumer>(&CI.getASTContext(),
                                                   hierarchy_,
                                                   rootClasses_,
                                                   features_);
    }

private:
    std::map<std::string, ClassNode>& hierarchy_;
    std::vector<std::string>& rootClasses_;
    std::map<std::string, Feature>& features_;
};

// Frontend Action Factory class
class FeatureAnalyzerActionFactory : public clang::tooling::FrontendActionFactory {
public:
    FeatureAnalyzerActionFactory(std::map<std::string, ClassNode>& hierarchy,
                                std::vector<std::string>& rootClasses,
                                std::map<std::string, Feature>& features)
        : hierarchy_(hierarchy), rootClasses_(rootClasses), features_(features) {}

    std::unique_ptr<FrontendAction> create() override {
        return std::make_unique<FeatureAnalyzerAction>(hierarchy_, rootClasses_, features_);
    }

private:
    std::map<std::string, ClassNode>& hierarchy_;
    std::vector<std::string>& rootClasses_;
    std::map<std::string, Feature>& features_;
};

// Main ROS2 Node class
class FeatureAnalyzer {
public:
    void analyze_features(const std::string& sourceFile) {
        if (sourceFile.empty()) {
            std::cerr << "No source file specified!" << std::endl;
            return;
        }

        std::map<std::string, ClassNode> classHierarchy;
        std::vector<std::string> rootClasses;
        std::map<std::string, Feature> features;
        
        analyze_ast(sourceFile, classHierarchy, rootClasses, features);
        std::vector<DeltaMarker> deltaMarkers = parse_delta_markers(sourceFile);
        process_delta_markers(deltaMarkers, features);
        generate_feature_tree(classHierarchy, rootClasses, features, deltaMarkers);
    }

    std::vector<DeltaMarker> parse_delta_markers(const std::string& filename) {
        std::vector<DeltaMarker> markers;
        std::ifstream file(filename);
        std::string line;
        std::regex deltaPattern(R"(//\s*DELTA:(\w+):(\d+):(\w+))");
        
        while (std::getline(file, line)) {
            std::smatch matches;
            if (std::regex_search(line, matches, deltaPattern)) {
                DeltaMarker marker{
                    .type = matches[1],
                    .featureName = matches[3],
                    .lineNumber = std::stoi(matches[2]),
                    .affectedCode = line
                };
                markers.push_back(marker);
            }
        }
        return markers;
    }

    void process_delta_markers(const std::vector<DeltaMarker>& markers, 
                             std::map<std::string, Feature>& features) {
        // Group delta markers by feature
        std::map<std::string, std::vector<DeltaMarker>> featureDeltas;
        for (const auto& marker : markers) {
            featureDeltas[marker.featureName].push_back(marker);
        }

        // Analyze each feature's delta markers
        for (const auto& [featureName, deltas] : featureDeltas) {
            Feature& feature = features[featureName];
            feature.isMandatory = false;  // If it has delta markers, it's optional

            for (const auto& delta : deltas) {
                feature.deltaOperations.insert(delta.type);
                
                // Analyze dependencies based on delta types
                if (delta.type == "change" || delta.type == "delete") {
                    // Look for affected features in the same region
                    for (const auto& otherDelta : markers) {
                        if (otherDelta.lineNumber < delta.lineNumber + 5 && 
                            otherDelta.lineNumber > delta.lineNumber - 5 &&
                            otherDelta.featureName != featureName) {
                            feature.dependencies.push_back(otherDelta.featureName);
                        }
                    }
                }
            }
        }
    }

    void generate_feature_tree(const std::map<std::string, ClassNode>& classHierarchy,
                             const std::vector<std::string>& rootClasses,
                             const std::map<std::string, Feature>& features,
                             const std::vector<DeltaMarker>& deltaMarkers) {
        std::ofstream outFile("feature_model.txt");
        
        outFile << "Feature Model Analysis\n";
        outFile << "=====================\n\n";

        // Print class hierarchy and base features
        outFile << "Class Hierarchy and Base Features:\n";
        outFile << "--------------------------------\n";
        for (const auto& rootClass : rootClasses) {
            print_class_hierarchy(rootClass, classHierarchy, features, outFile, 0);
        }

        // Print optional features and their dependencies
        outFile << "\nOptional Features:\n";
        outFile << "-----------------\n";
        for (const auto& [name, feature] : features) {
            if (!feature.isMandatory) {
                outFile << "- " << name << "\n";
                if (!feature.dependencies.empty()) {
                    outFile << "  Dependencies:\n";
                    for (const auto& dep : feature.dependencies) {
                        outFile << "    - " << dep << "\n";
                    }
                }
                outFile << "  Delta Operations: ";
                for (const auto& op : feature.deltaOperations) {
                    outFile << op << " ";
                }
                outFile << "\n";
            }
        }

        outFile.close();
    }

    void print_class_hierarchy(const std::string& className,
                             const std::map<std::string, ClassNode>& hierarchy,
                             const std::map<std::string, Feature>& features,
                             std::ofstream& outFile,
                             int indent) {
        std::string indentation(indent * 2, ' ');
        const auto& classNode = hierarchy.at(className);
        
        outFile << indentation << "- Class: " << className << "\n";
        
        // Print methods that are identified as features
        for (const auto& method : classNode.methods) {
            if (features.count(method) > 0 && features.at(method).isMandatory) {
                outFile << indentation << "  - Feature: " << method << "\n";
            }
        }
        
        // Print subclasses
        for (const auto& subClass : classNode.subClasses) {
            print_class_hierarchy(subClass, hierarchy, features, outFile, indent + 1);
        }
    }

    void analyze_ast(const std::string& filename,
                    std::map<std::string, ClassNode>& classHierarchy,
                    std::vector<std::string>& rootClasses,
                    std::map<std::string, Feature>& features) {
        // Add compile flags to ignore missing includes
        std::vector<std::string> args{
        "-std=c++17",
        "-fsyntax-only",
        "-w",                                     // Disable all warnings
        "-I/opt/ros/iron/include",               // ROS2 include path
        "-I/usr/include/c++/11",                 // System includes
        "-I/usr/include/x86_64-linux-gnu/c++/11",
        "-I/usr/include",
        "-I/usr/local/include",
        "-nostdinc++",                           // Don't use standard include paths
        "-nostdinc",
        "-D__CODE_GENERATOR__"                   // Define to skip problematic includes
    };
        std::vector<std::string> sources{filename};
        
        auto compilations = clang::tooling::FixedCompilationDatabase(".", args);
        clang::tooling::ClangTool tool(compilations, sources);

        FeatureAnalyzerActionFactory factory(classHierarchy, rootClasses, features);
        tool.run(&factory);
    } 

};

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <source_file>" << std::endl;
        return 1;
    }

    fs::path currentPath = fs::current_path();
    std::cerr << currentPath<< std::endl;
    fs::path inputPath = argv[1];
    fs::path fullPath = currentPath /  inputPath;
    std::cerr << fullPath<< std::endl;

    if (!fs::exists(fullPath)) {
        std::cerr << "Error: File not found at " << fullPath << std::endl;
        return 1;
    }

    FeatureAnalyzer analyzer;
    analyzer.analyze_features(fullPath.string());
    return 0;
}