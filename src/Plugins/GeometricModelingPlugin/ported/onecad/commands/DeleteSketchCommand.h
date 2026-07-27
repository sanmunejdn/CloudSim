/**
 * @file DeleteSketchCommand.h
 * @brief Command that deletes a sketch from the document with undo support.
 */
#ifndef ONECAD_APP_COMMANDS_DELETESKETCHCOMMAND_H
#define ONECAD_APP_COMMANDS_DELETESKETCHCOMMAND_H

#include "Command.h"

#include <string>
#include <memory>

namespace onecad::core::sketch {
class Sketch;
}

namespace onecad::app {
class Document;
}

namespace onecad::app::commands {

class DeleteSketchCommand : public Command {
public:
    DeleteSketchCommand(Document* document, const std::string& sketchId);

    bool execute() override;
    bool undo() override;
    std::string label() const override { return "Delete Sketch"; }

private:
    Document* document_ = nullptr;
    std::string sketchId_;
    std::string savedName_;
    std::string savedJson_;  // Sketch serialized to JSON for undo
    bool savedVisible_ = true;
};

} // namespace onecad::app::commands

#endif // ONECAD_APP_COMMANDS_DELETESKETCHCOMMAND_H
