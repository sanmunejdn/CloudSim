/**
 * @file ImportStepCommand.h
 * @brief Undoable command for STEP import
 */
#pragma once

#include "Command.h"
#include "../../io/step/StepImporter.h"

#include <string>
#include <unordered_map>
#include <array>
#include <vector>

namespace onecad::app {
class Document;
}

namespace onecad::app::commands {

class ImportStepCommand : public Command {
public:
    ImportStepCommand(Document* document, std::vector<io::ImportedBody> bodies);

    bool execute() override;
    bool undo() override;
    std::string label() const override { return "Import STEP"; }

private:
    Document* document_;
    std::vector<io::ImportedBody> bodies_;
    std::vector<std::string> createdBodyIds_;
    std::unordered_map<std::string, std::unordered_map<std::string, std::array<float, 4>>> bodyFaceColors_;
};

} // namespace onecad::app::commands
