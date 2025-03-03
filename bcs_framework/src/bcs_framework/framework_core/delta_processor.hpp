#ifndef DELTA_PROCESSOR_HPP
#define DELTA_PROCESSOR_HPP

#include <clang/AST/ASTConsumer.h>
#include <clang/AST/ASTContext.h>
#include <clang/Frontend/FrontendAction.h>
#include <clang/Frontend/CompilerInstance.h>
#include <clang/Rewrite/Core/Rewriter.h>
#include <clang/Tooling/Tooling.h>
#include <clang/Tooling/CommonOptionsParser.h>
#include <fstream>
#include <vector>
#include <string>
#include "feature_config.hpp"

class DeltaProcessor {
public:
    enum class DeltaType {
        ADD,
        DELETE,
        CHANGE
    };

    struct MarkerInfo {
        std::string type;
        std::string id;
        std::string feature;
    };

    struct Delta {
        std::string feature;
        DeltaType type;
        std::string id;
        std::vector<std::string> content;
    };

    DeltaProcessor(const fs::path& projectRoot,
                   const std::string& sourceFile, 
                   const std::string& featureConfigFile,
                   const std::vector<std::string>& requestedFeatures,
                   const std::vector<std::string>& allFeatures);

    bool processDeltas(clang::ASTContext& context);
    void setSourceManager(clang::SourceManager& sourceMgr, 
                          const clang::LangOptions& langOpts);
    bool writeModifiedFile();
    fs::path writeIntermediateFile();

private:
    fs::path projectRoot;
    fs::path m_outputPath;
    std::string m_sourceFile;
    std::string m_featureConfigFile;
    std::vector<std::string> m_requestedFeatures;
    std::vector<std::string> m_activeFeatures;
    std::vector<std::string> m_allFeatures;
    std::vector<Delta> m_deltas;
    clang::Rewriter m_rewriter;

    bool parseDeltaFile(const std::string& deltaFile, const MarkerInfo& marker);
    DeltaType stringToDeltaType(const std::string& type);
    bool parseContent(const toml::table* deltaTable, Delta& delta);
    bool checkSingleCondition(const toml::node& conditionNode);
};

// Frontend Action for Delta Processing
class DeltaProcessingAction : public clang::ASTFrontendAction {
public:
    DeltaProcessingAction(const fs::path& projectRoot,
                          const std::string& sourceFile, 
                          const std::string& deltaFile,
                          const std::vector<std::string>& activeFeatures,
                          const std::vector<std::string>& allFeatures);

    std::unique_ptr<clang::ASTConsumer> CreateASTConsumer(
        clang::CompilerInstance& compiler, 
        llvm::StringRef file) override;

    void EndSourceFileAction() override;

private:
    DeltaProcessor m_processor;

    // Custom AST Consumer
    class DeltaASTConsumer : public clang::ASTConsumer {
    public:
        DeltaASTConsumer(clang::ASTContext& context, DeltaProcessor& processor);

        void HandleTranslationUnit(clang::ASTContext& context) override;

    private:
        clang::ASTContext& m_context;
        DeltaProcessor& m_processor;
    };
};

// Factory for creating DeltaProcessingAction
class DeltaProcessingActionFactory : public clang::tooling::FrontendActionFactory {
public:
    DeltaProcessingActionFactory(
        const fs::path& projectRoot,
        const std::string& sourceFile,
        const std::string& deltaFile,
        const std::vector<std::string>& activeFeatures,
        const std::vector<std::string>& allFeatures);

    std::unique_ptr<clang::FrontendAction> create() override;

private:
    fs::path projectRoot;
    std::string m_sourceFile;
    std::string m_deltaFile;
    std::vector<std::string> m_activeFeatures;
    std::vector<std::string> m_allFeatures;
};

#endif // DELTA_PROCESSOR_HPP