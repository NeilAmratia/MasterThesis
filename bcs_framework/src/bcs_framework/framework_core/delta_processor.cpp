#include "delta_processor.hpp"
#include <clang/AST/RecursiveASTVisitor.h>
#include <clang/Basic/SourceManager.h>
#include <clang/Lex/Lexer.h>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <sstream>
#include <regex>
#include "feature_config.hpp"
#include <toml++/toml.hpp>


// Custom AST Visitor to find and modify delta locations
class DeltaASTVisitor : public clang::RecursiveASTVisitor<DeltaASTVisitor> {
public:
    DeltaASTVisitor(clang::ASTContext& context, 
                    clang::Rewriter& rewriter,
                    std::vector<DeltaProcessor::Delta>& deltas,
                    const std::vector<std::string>& requestedFeatures,
                    const std::vector<std::string>& allFeatures)
        : m_context(context), 
          m_rewriter(rewriter), 
          m_deltas(deltas),
          m_requestedFeatures(requestedFeatures),
          m_allFeatures(allFeatures) {}

    bool ProcessFile() { 

        clang::SourceManager& sourceMgr = m_context.getSourceManager();
        clang::SourceLocation startLoc = sourceMgr.getLocForStartOfFile(sourceMgr.getMainFileID());

        // Focus only on main file content, ignore includes
        bool invalid = false;
        llvm::StringRef content = sourceMgr.getBufferData(sourceMgr.getMainFileID(), &invalid);
        if (invalid) {
            std::cerr << "Error: Could not read main file content" << std::endl;
            return false;
        }

        // Process all deltas once for the whole file
        for (auto it = m_deltas.begin(); it != m_deltas.end();) {
            std::string markerPrefix, markerSuffix;
            bool hasId = !it->id.empty();
            bool processed = false;

            switch (it->type) {
                case DeltaProcessor::DeltaType::ADD:
                    if (hasId) {
                        markerPrefix = "// DELTA:add:" + it->id + ":" + it->feature;
                    } else {
                        markerPrefix = "// DELTA:add:" + it->feature;
                    }
                    processed = insertContentAfterMarker(startLoc, markerPrefix, it->content);
                    break;
                
                case DeltaProcessor::DeltaType::DELETE:
                    if (hasId) {
                        markerPrefix = "// DELTA:delete:" + it->id + ":" + it->feature;
                        markerSuffix = "// DELTA:delete:" + it->id + ":" + it->feature + ":END";
                    } else {
                        markerPrefix = "// DELTA:delete:" + it->feature;
                        markerSuffix = "// DELTA:delete:" + it->feature + ":END";
                    }
                    processed = removeContentBetweenMarkers(startLoc, markerPrefix, markerSuffix);
                    break;
                
                case DeltaProcessor::DeltaType::CHANGE:
                    if (hasId) {
                        markerPrefix = "// DELTA:change:" + it->id + ":" + it->feature;
                        markerSuffix = "// DELTA:change:" + it->id + ":" + it->feature + ":END";
                    } else {
                        markerPrefix = "// DELTA:change:" + it->feature;
                        markerSuffix = "// DELTA:change:" + it->feature + ":END";
                    }
                    processed = replaceContentBetweenMarkers(startLoc, markerPrefix, markerSuffix, it->content);
                    break;
            }
            // Add iterator advancement
            if (processed) {
                it = m_deltas.erase(it);  // Remove processed delta and advance iterator
            } else {
                ++it;  // Skip unprocessed delta
            }
        }
        return true;
    }

    void removeAllDeltaMarkers(clang::SourceLocation startLoc, clang::Rewriter& rewriter) {
        clang::SourceManager& sourceMgr = m_context.getSourceManager();
        bool fileModified = false; 

        // Debug logging
        std::cout << "Debug: Processing file location: " 
                << sourceMgr.getFilename(startLoc).str() << std::endl;

        // Get main file buffer
        clang::FileID fileID = sourceMgr.getFileID(startLoc);
        if (fileID.isInvalid()) {
            std::cerr << "Invalid file ID" << std::endl;
            return;
        }

        // Get file entry with additional verification
        const clang::FileEntry* fileEntry = sourceMgr.getFileEntryForID(fileID);
        if (!fileEntry) {
            // Try getting file entry directly from source location
            fileEntry = sourceMgr.getFileEntryForID(sourceMgr.getMainFileID());
            if (!fileEntry) {
                std::cerr << "Unable to get file entry for any valid ID" << std::endl;
                return;
            }
        }

        // Get file contents
        bool invalid = false;
        const llvm::StringRef fileContents = sourceMgr.getBufferData(fileID, &invalid);
        if (invalid || fileContents.empty()) {
            std::cerr << "Unable to retrieve file contents" << std::endl;
            return;
        }

        struct MarkerInfo {
            clang::SourceLocation location;
            unsigned length;
            std::string text;

            MarkerInfo(clang::SourceLocation loc, unsigned len, std::string txt) 
                : location(loc), length(len), text(std::move(txt)) {}
        };
        std::vector<MarkerInfo> markers;

        std::string fileStr = fileContents.str();
        size_t searchPos = 0;

        // First collect all markers
        while (searchPos < fileStr.length()) {
            size_t markerPos = fileStr.find("// DELTA:", searchPos);
            if (markerPos == std::string::npos) break;

            size_t lineEnd = fileStr.find('\n', markerPos);
            if (lineEnd == std::string::npos) lineEnd = fileStr.length();

            std::string marker = fileStr.substr(markerPos, lineEnd - markerPos);
            
            unsigned lineNum = sourceMgr.getLineNumber(fileID, markerPos);
            unsigned colNum = sourceMgr.getColumnNumber(fileID, markerPos);
            
            clang::SourceLocation markerLoc = sourceMgr.translateFileLineCol(
                fileEntry, lineNum, colNum);

            if (markerLoc.isValid()) {
                markers.push_back(MarkerInfo(markerLoc, marker.length(), marker));
            }

            searchPos = lineEnd + 1;
        }

        // Sort markers in reverse order of position
        std::sort(markers.begin(), markers.end(), 
            [](const MarkerInfo& a, const MarkerInfo& b) {
                return a.location.getRawEncoding() > b.location.getRawEncoding();
            });

        // Remove markers from end to start
        for (const auto& marker : markers) {
            clang::SourceRange markerRange(
                marker.location,
                marker.location.getLocWithOffset(marker.length)
            );
            
            // Use passed rewriter
            if (!rewriter.RemoveText(markerRange)) {
                std::cerr << "Failed to remove marker: " << marker.text << std::endl;
                continue;
            }
            fileModified = true;
        }

        // Verify changes
        if (fileModified) {
            const clang::RewriteBuffer* buffer = rewriter.getRewriteBufferFor(fileID);
            if (!buffer) {
                std::cerr << "Failed to get rewrite buffer after marker removal" << std::endl;
                return;
            }
            std::cout << "All DELTA markers removed successfully." << std::endl;
        }
    }

private:
    clang::ASTContext& m_context;
    clang::Rewriter& m_rewriter;
    std::vector<DeltaProcessor::Delta>& m_deltas;
    std::vector<std::string> m_requestedFeatures;
    std::vector<std::string> m_allFeatures;
    std::set<std::pair<clang::SourceLocation, clang::SourceLocation>> deletedRegions;
    

    // Helper method to find source location of a marker
    clang::SourceLocation findMarkerLocation(clang::SourceLocation start, 
                                            const std::string& marker, 
                                            bool isEndMarker = false) {
        clang::SourceManager& sourceMgr = m_context.getSourceManager();
        clang::FileID fileID = sourceMgr.getFileID(start);

        // Get the FileEntry associated with the FileID
        const clang::FileEntry* fileEntry = sourceMgr.getFileEntryForID(fileID);
        if (!fileEntry) {
            std::cerr << "Unable to retrieve FileEntry for FileID" << std::endl;
            return start;
        }

        // Get the file contents
        bool invalidTemp;
        llvm::StringRef fileContents = sourceMgr.getBufferData(fileID, &invalidTemp);
        if (invalidTemp) {
            std::cerr << "Unable to retrieve file contents" << std::endl;
            return start;
        }

        if (fileContents.empty()) {
            std::cerr << "File content is empty!" << std::endl;
            return start;
        }

        // Find the marker in the file contents
        size_t markerPos = fileContents.find(marker);
        if (markerPos == std::string::npos) {
            std::cerr << "Marker not found using full search" << std::endl;
        }

        // Calculate line and column numbers
        unsigned line = sourceMgr.getLineNumber(fileID, markerPos);
        unsigned column = sourceMgr.getColumnNumber(fileID, markerPos);

        // Translate line and column numbers to a SourceLocation
        return sourceMgr.translateFileLineCol(fileEntry, line, column);
    }


    bool isLocationDeleted(clang::SourceLocation loc) {
        for (const auto& region : deletedRegions) {
            if (m_context.getSourceManager().isBeforeInTranslationUnit(region.first, loc) &&
                !m_context.getSourceManager().isBeforeInTranslationUnit(region.second, loc)) {
                return true;
            }
        }
        return false;
    }

    bool insertContentAfterMarker(clang::SourceLocation funcStart, 
                                const std::string& markerPrefix, 
                                const std::vector<std::string>& content) {
        clang::SourceManager& sourceMgr = m_context.getSourceManager();
        clang::SourceLocation markerLoc = findMarkerLocation(funcStart, markerPrefix);

        // Skip if marker not found or in deleted region
        if (markerLoc.isInvalid() || isLocationDeleted(markerLoc)) {
            std::cout << "  Skipping insert - Invalid or deleted start marker: " << markerPrefix << std::endl;
            return false;
        }

        // Get the indentation of the marker's line
        unsigned markerColumn = sourceMgr.getSpellingColumnNumber(markerLoc) - 1;
        std::string indentation(markerColumn, ' ');
        
        // Prepare inserted content with proper indentation
        std::stringstream insertContent;
        bool isFirst = true;
        for (const auto& line : content) {
            if (!isFirst) {
                insertContent << indentation;
            }
            insertContent << line << "\n";
            isFirst = false;
        }

        // Insert content directly after the marker
        m_rewriter.InsertTextAfter(markerLoc, insertContent.str());

        return true;
    }

    // Remove content between markers
    bool removeContentBetweenMarkers(clang::SourceLocation funcStart, 
                                      const std::string& markerPrefix, 
                                      const std::string& markerSuffix) {
        clang::SourceLocation startLoc = findMarkerLocation(funcStart, markerPrefix);
        clang::SourceLocation endLoc = findMarkerLocation(startLoc, markerSuffix);

        if (startLoc.isInvalid()) {
            return false;
        }
        if (endLoc.isInvalid()) {
            return false;
        }

        deletedRegions.insert({startLoc, endLoc});

        // Remove the content between markers, including the markers themselves
        m_rewriter.ReplaceText(
            clang::SourceRange(startLoc, endLoc.getLocWithOffset(markerSuffix.length())), 
            ""
        );

        return true;
    }

    // Replace content between markers
    bool replaceContentBetweenMarkers(clang::SourceLocation startLoc, 
                                    const std::string& markerPrefix, 
                                    const std::string& markerSuffix,
                                    const std::vector<std::string>& content) {
        clang::SourceManager& sourceMgr = m_context.getSourceManager();
        clang::SourceLocation markerStartLoc = findMarkerLocation(startLoc, markerPrefix);
        
        if (markerStartLoc.isInvalid()  || isLocationDeleted(markerStartLoc) ) {
            std::cout << "  Unable to find start marker: " << markerPrefix << std::endl;
            return false;
        }

        clang::SourceLocation markerEndLoc = findMarkerLocation(markerStartLoc, markerSuffix);
        if (markerEndLoc.isInvalid()) {
            std::cout << "  Unable to find end marker: " << markerSuffix << std::endl;
            return false;
        }

        unsigned startColumn = sourceMgr.getSpellingColumnNumber(markerStartLoc) - 1;
        std::string indentation(startColumn, ' ');

        std::stringstream replaceContent;
        bool isFirst = true;
        for (const auto& line : content) {
            if (!isFirst) {
                replaceContent << indentation;
            }
            replaceContent << line << "\n";
            isFirst = false;
        }

        m_rewriter.ReplaceText(
            clang::SourceRange(markerStartLoc, markerEndLoc.getLocWithOffset(markerSuffix.length())), 
            replaceContent.str()
        );
        return true;
    }

};

class DeltaMarkerScanner {
public:
    DeltaMarkerScanner(clang::ASTContext& context, std::vector<DeltaProcessor::MarkerInfo>& markers)
        : m_context(context), m_foundMarkers(markers) {}

    void scanFile() {
        clang::SourceManager& sourceMgr = m_context.getSourceManager();
        clang::FileID mainFileID = sourceMgr.getMainFileID();
        
        bool invalid = false;
        const char* start = sourceMgr.getBufferData(mainFileID, &invalid).data();
        if (invalid) return;

        llvm::StringRef content(start, sourceMgr.getFileIDSize(mainFileID));
        size_t pos = 0;
        std::set<std::string> processedMarkers;

        // Track unique markers by type+id+feature
        std::set<std::string> uniqueMarkers;

        while ((pos = content.find("// DELTA:", pos)) != llvm::StringRef::npos) {
            size_t lineEnd = content.find('\n', pos);
            if (lineEnd == llvm::StringRef::npos) {
                lineEnd = content.size(); 
            }

            llvm::StringRef line = content.slice(pos, lineEnd);
            std::string markerText = line.str();

            // Skip if already processed
            if (processedMarkers.count(markerText) > 0) {
                pos = lineEnd + 1;
                continue;
            }
            processedMarkers.insert(markerText);
            
            // Skip END markers
            if (line.find(":END") != llvm::StringRef::npos) {
                pos = lineEnd + 1;
                continue;
            }

            if (auto marker = parseMarker(line)) {
                m_foundMarkers.push_back(*marker);
            }
            
            pos = (lineEnd == content.size()) ? lineEnd : lineEnd + 1;
        }
    }

private:
    clang::ASTContext& m_context;
    std::vector<DeltaProcessor::MarkerInfo>& m_foundMarkers;

    std::optional<DeltaProcessor::MarkerInfo> parseMarker(llvm::StringRef line) {
        // Define regex pattern for marker format
        static const std::regex markerPattern(R"(\/\/\s*DELTA:([A-Za-z]+):(\d+):([A-Za-z_]+)(?:\s*$|\s*:\s*END)?)");
        
        std::string lineStr = line.str();
        std::smatch matches;

        // Try to match the pattern
        if (!std::regex_search(lineStr, matches, markerPattern)) {
            return std::nullopt;
        }

        // Validate we have all parts (full match + 3 capture groups)
        if (matches.size() != 4) {
            return std::nullopt;
        }

        DeltaProcessor::MarkerInfo marker;
        marker.type = matches[1].str();     // First capture group: Type
        marker.id = matches[2].str();       // Second capture group: ID
        marker.feature = matches[3].str();   // Third capture group: Feature

        return isValidMarker(marker) ? std::make_optional(marker) : std::nullopt;
    }

    bool isValidMarker(const DeltaProcessor::MarkerInfo& marker) {
        return (marker.type == "add" || marker.type == "delete" || marker.type == "change") 
               && !marker.id.empty() && !marker.feature.empty();
    }
};

// Implementations for DeltaProcessor methods
DeltaProcessor::DeltaProcessor(const fs::path& projectRoot,
                               const std::string& sourceFile, 
                               const std::string& featureConfigFile,
                               const std::vector<std::string>& requestedFeatures,
                            const std::vector<std::string>& allFeatures)
    : projectRoot(projectRoot),
      m_sourceFile(sourceFile), 
      m_featureConfigFile(featureConfigFile),
      m_requestedFeatures(requestedFeatures),
      m_allFeatures(allFeatures) {}


bool DeltaProcessor::checkSingleCondition(const toml::node& conditionNode) {
    if (!conditionNode.is_table()) return true;
    if (auto condTable = conditionNode.as_table()) {
        if (auto featureNode = condTable->get("feature")) {
            if (auto featureName = featureNode->value<std::string>()) {
                if (auto stateNode = condTable->get("state")) {
                    if (auto state = stateNode->value<std::string>()) {
                        bool featureActive = std::find(m_requestedFeatures.begin(), 
                                                        m_requestedFeatures.end(), 
                                                        *featureName) != m_requestedFeatures.end();
                        
                        return !(*state == "active" && !featureActive) && 
                                !(*state == "inactive" && featureActive);
                    }
                }
            }
        }
    }
    return true;
}

bool DeltaProcessor::parseDeltaFile(const std::string& deltaFile, const MarkerInfo& marker) {

    if (!fs::exists(deltaFile)) {
        std::cerr << "Delta file not found: " << deltaFile << std::endl;
        return false;
    }
    try {
        toml::table deltaToml = toml::parse_file(deltaFile);
        const toml::array* deltasArray = deltaToml.get("deltas")->as_array();

        if (!deltasArray) {
            std::cerr << "No 'deltas' array found in file: " << deltaFile << std::endl;
            return false;
        }

        // Find the specific delta we're looking for
        for (const auto& deltaNode : *deltasArray) {
            const toml::table* deltaTable = deltaNode.as_table();
            if (!deltaTable) continue;

            // Quick check for type and ID first
            auto typeNode = deltaTable->get("type");
            auto idNode = deltaTable->get("id");
            
            if (!typeNode || !idNode) continue;

            // Convert and compare ID first (as it's likely to be more unique)
            int64_t currentId;
            try {
                currentId = *idNode->value<int64_t>();
                if (std::to_string(currentId) != marker.id) continue;
            } catch (...) {
                continue;
            }

            // Then check type
            std::string currentType;
            try {
                currentType = *typeNode->value<std::string>();
                if (currentType != marker.type) continue;
            } catch (...) {
                continue;
            }

            // Check conditions if present
            if (auto conditionNode = deltaTable->get("condition")) {
                bool allConditionsMet = true;

                if (auto condArray = conditionNode->as_array()) {
                    for (const auto& item : *condArray) {
                        if (!checkSingleCondition(item)) {
                            allConditionsMet = false;
                            break;
                        }
                    }
                } else if (auto condTable = conditionNode->as_table()) {
                    allConditionsMet = checkSingleCondition(*conditionNode);
                }

                if (!allConditionsMet) {
                    return false;
                }
            }

            // Create and populate delta object
            Delta d;
            try {
                d.type = stringToDeltaType(currentType);  // Convert string to DeltaType enum
            } catch (const std::exception& e) {
                std::cerr << "Failed to convert delta type '" << currentType << "': " << e.what() << std::endl;
                return false;
            }
            d.id = std::to_string(currentId);
            d.feature = marker.feature;

            // Parse content for non-DELETE type
            if (d.type != DeltaType::DELETE) {
                if (!parseContent(deltaTable, d)) {
                    std::cerr << "Failed to parse content for delta ID: " << currentId << std::endl;
                    return false;
                }
            }

            // Add valid delta and return success
            m_deltas.push_back(d);
            return true;
        }

        // If we get here, we didn't find the delta
        std::cerr << "No matching delta found (Type: " << marker.type 
                  << ", ID: " << marker.id << ") in file: " << deltaFile << std::endl;
        return false;

    } catch (const std::exception& err) {
        std::cerr << "Error processing TOML file: " << err.what() << std::endl;
        return false;
    }
}


DeltaProcessor::DeltaType DeltaProcessor::stringToDeltaType(const std::string& type) {
    if (type == "add") return DeltaType::ADD;
    if (type == "delete") return DeltaType::DELETE;
    if (type == "change") return DeltaType::CHANGE;
    throw std::runtime_error("Invalid delta type: " + type);
}

bool DeltaProcessor::parseContent(const toml::table* deltaTable, Delta& delta) {
    if (auto contentNode = deltaTable->get("content")) {
        // Handle single string content
        if (auto contentStr = contentNode->value<std::string>()) {
            // Split multi-line string into vector
            std::istringstream stream(*contentStr);
            std::string line;
            while (std::getline(stream, line)) {
                delta.content.push_back(line);
            }
            return true;
        }
        // Handle array of strings
        else if (auto contentArray = contentNode->as_array()) {
            for (const auto& line : *contentArray) {
                if (auto str = line.value<std::string>()) {
                    delta.content.push_back(*str);
                }
            }
            return !delta.content.empty();
        }
        std::cerr << "Invalid content format in delta " << delta.id << std::endl;
        return false;
    }
    std::cerr << "Missing content for non-DELETE delta " << delta.id << std::endl;
    return false;
}

bool DeltaProcessor::processDeltas(clang::ASTContext& context) {
    // Vector to store found markers with their details
    std::vector<MarkerInfo> foundMarkers;
    bool success = true;

    // First scan for markers in source file
    DeltaMarkerScanner scanner(context, foundMarkers);
    scanner.scanFile();

    if (foundMarkers.empty()) {
        std::cout << "No delta markers found in source file." << std::endl;
        return true;
    }

    // Group markers by feature
    std::map<std::string, std::vector<MarkerInfo>> markersByFeature;
    for (const auto& marker : foundMarkers) {
        markersByFeature[marker.feature].push_back(marker);
    }

    FeatureConfiguration featureConfig(projectRoot, m_featureConfigFile);
    if (!featureConfig.parseConfig()) {
        std::cout << "Cannot parse the configuration file" << m_featureConfigFile << std::endl;
        return false;
    }

    bool hasModifications = false;
    
    // Process one feature at a time
    for (const auto& [feature, markers] : markersByFeature) {
        std::vector<Delta> featureDeltas;
        
        // Parse all deltas for this feature
        for (const auto& marker : markers) {
            std::string deltaFile = featureConfig.getFeatureDeltaFile(marker.feature);
            if (deltaFile.empty()) continue;

            if (parseDeltaFile(deltaFile, marker)) {
                featureDeltas.insert(featureDeltas.end(), m_deltas.begin(), m_deltas.end());
            }
            m_deltas.clear();
        }

        // Apply all deltas for this feature at once
        if (!featureDeltas.empty()) {
            m_deltas = featureDeltas;
            DeltaASTVisitor visitor(context, m_rewriter, m_deltas, m_requestedFeatures, m_allFeatures);
            if (visitor.ProcessFile()) {
                hasModifications = true;
            }
        }
    }

    // Write modified file first
    // if (!writeModifiedFile()) return false;

    // // Load the modified file and remove markers
    // fs::path outputFile = projectRoot / "active_configuration_files" / std::filesystem::path(m_sourceFile).filename();
    // std::ifstream modifiedFile(outputFile);
    // if (!modifiedFile) return false;

    // std::string content((std::istreambuf_iterator<char>(modifiedFile)), std::istreambuf_iterator<char>());
    // modifiedFile.close();

    // // Create new rewriter for marker removal
    // clang::FileID mainFileID = context.getSourceManager().createFileID(
    //     llvm::MemoryBuffer::getMemBuffer(content),
    //     clang::SrcMgr::C_User
    // );

    // clang::Rewriter markerRewriter;
    // markerRewriter.setSourceMgr(context.getSourceManager(), context.getLangOpts());

    // std::vector<Delta> emptyDeltas;
    // DeltaASTVisitor cleanupVisitor(context, markerRewriter, emptyDeltas, m_requestedFeatures, m_allFeatures);
    // cleanupVisitor.removeAllDeltaMarkers(
    //     context.getSourceManager().getLocForStartOfFile(mainFileID),
    //     markerRewriter
    // );

    // // Overwrite the file with markers removed
    // std::ofstream finalFile(outputFile);
    // if (!finalFile) return false;

    // bool invalid = false;
    // llvm::StringRef finalContent = context.getSourceManager().getBufferData(mainFileID, &invalid);
    // if (invalid) return false;

    // finalFile << finalContent.str();
    // finalFile.close();

    return true;
}

fs::path DeltaProcessor::writeIntermediateFile() {
    // Get workspace directory from ROS2 environment
    const char* workspace_dir = std::getenv("COLCON_PREFIX_PATH");
    if (!workspace_dir) {
        std::cerr << "COLCON_PREFIX_PATH not set. Please source your workspace." << std::endl;
        return "";
    }
    
    // Go up two levels from COLCON_PREFIX_PATH to reach workspace root
    fs::path workspacePath = fs::path(workspace_dir).parent_path().parent_path();
    fs::path outputDir = workspacePath / "src" / "active_configuration_files" / "src";
    
    std::filesystem::create_directories(outputDir);
    
    if (!fs::exists(outputDir)) {
        std::cerr << "Output directory does not exist: " << outputDir << std::endl;
        return "";
    }
    
    fs::path intermediateFile = outputDir / "intermediate.txt";
    std::error_code EC;
    
    llvm::raw_fd_ostream outFile(intermediateFile.string(), EC);
    if (EC) return "";

    clang::FileID mainFileID = m_rewriter.getSourceMgr().getMainFileID();
    const auto* buffer = m_rewriter.getRewriteBufferFor(mainFileID);
    
    if (buffer) {
        buffer->write(outFile);
    } else {
        bool invalid = false;
        llvm::StringRef content = m_rewriter.getSourceMgr().getBufferData(mainFileID, &invalid);
        if (!invalid) outFile << content;
        else return "";
    }
    
    outFile.close();
    return intermediateFile;
}

bool DeltaProcessor::writeModifiedFile() {
    fs::path outputDir =  projectRoot.parent_path() / "active_configuration_files/src";

    // Build the output file path by combining the directory and the source file name
    fs::path outputFile = outputDir / std::filesystem::path(m_sourceFile).filename();

    // Delete existing file if it exists
    if (fs::exists(outputFile)) {
        fs::remove(outputFile);
    }

    std::cout << "Output stored at" << outputFile << std::endl;
    std::error_code EC;
    
    llvm::raw_fd_ostream outFile(outputFile.string(), EC);
    if (EC) return false;

    clang::FileID mainFileID = m_rewriter.getSourceMgr().getMainFileID();
    const auto* buffer = m_rewriter.getRewriteBufferFor(mainFileID);
    
    if (buffer) {
        buffer->write(outFile);
    } else {
        bool invalid = false;
        llvm::StringRef content = m_rewriter.getSourceMgr().getBufferData(mainFileID, &invalid);
        if (!invalid) outFile << content;
        else return false;
    }
    
    outFile.close();
    return true;
}

void DeltaProcessor::setSourceManager(clang::SourceManager& sourceMgr, 
                                    const clang::LangOptions& langOpts) {
    m_rewriter.setSourceMgr(sourceMgr, langOpts);
}

DeltaProcessingAction::DeltaProcessingAction(
    const fs::path& projectRoot,
    const std::string& sourceFile, 
    const std::string& deltaFile,
    const std::vector<std::string>& activeFeatures,
    const std::vector<std::string>& allFeatures)
    : m_processor(projectRoot, sourceFile, deltaFile, activeFeatures, allFeatures) {}

std::unique_ptr<clang::ASTConsumer> DeltaProcessingAction::CreateASTConsumer(
    clang::CompilerInstance& compiler, 
    llvm::StringRef file) {
    
    // Configure the compiler to ignore missing includes
    compiler.getDiagnostics().setIgnoreAllWarnings(true);
    compiler.getDiagnostics().setSuppressAllDiagnostics(true);

    // Add dummy include paths to prevent errors
    compiler.getHeaderSearchOpts().AddPath("/usr/include", 
        clang::frontend::IncludeDirGroup::System, false, false);

    // Set source manager for the processor
    m_processor.setSourceManager(
        compiler.getSourceManager(), 
        compiler.getLangOpts()
    );
    
    // Return a new DeltaASTConsumer
    return std::make_unique<DeltaASTConsumer>(
        compiler.getASTContext(), 
        m_processor
    );
}

void DeltaProcessingAction::EndSourceFileAction() {
    // Write modified file after processing
    m_processor.writeModifiedFile();
}

// Implementation for DeltaProcessingAction::DeltaASTConsumer
DeltaProcessingAction::DeltaASTConsumer::DeltaASTConsumer(
    clang::ASTContext& context, 
    DeltaProcessor& processor)
    : m_context(context), m_processor(processor) {}

void DeltaProcessingAction::DeltaASTConsumer::HandleTranslationUnit(
    clang::ASTContext& context) {
    // Process deltas on the entire translation unit
    m_processor.processDeltas(context);
}

// Implementation for DeltaProcessingActionFactory
DeltaProcessingActionFactory::DeltaProcessingActionFactory(
    const fs::path& projectRoot,
    const std::string& sourceFile, 
    const std::string& deltaFile,
    const std::vector<std::string>& activeFeatures,
    const std::vector<std::string>& allFeatures)
    : projectRoot(projectRoot),
      m_sourceFile(sourceFile), 
      m_deltaFile(deltaFile), 
      m_activeFeatures(activeFeatures),
      m_allFeatures(allFeatures) {}

std::unique_ptr<clang::FrontendAction> DeltaProcessingActionFactory::create() {
    return std::make_unique<DeltaProcessingAction>(
        projectRoot,
        m_sourceFile, 
        m_deltaFile, 
        m_activeFeatures,
        m_allFeatures
    );
}

