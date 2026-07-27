/**
 * @file AddSketchCommand.h
 * @brief Command that adds a new (empty) sketch to the document.
 */
#ifndef ONECAD_APP_COMMANDS_ADDSKETCHCOMMAND_H
#define ONECAD_APP_COMMANDS_ADDSKETCHCOMMAND_H

#include "Command.h"
#include "../../core/sketch/Sketch.h"

#include <optional>
#include <string>
#include <utility>

namespace onecad::app {
class Document;
}

namespace onecad::app::commands {

/**
 * Adds an empty sketch on a given plane, optionally host-face attached.
 * The sketch id is generated once and reused across undo/redo cycles so
 * references (navigator selection, follow-up commands in the same
 * transaction) stay stable.
 */
class AddSketchCommand : public Command {
public:
    AddSketchCommand(Document* document,
                     const core::sketch::SketchPlane& plane,
                     std::string name = {});

    /// Optional host-face attachment applied to the created sketch.
    void setHostFaceAttachment(std::string bodyId, std::string faceId);

    bool execute() override;
    bool undo() override;
    std::string label() const override { return "Add Sketch"; }

    const std::string& sketchId() const { return sketchId_; }

private:
    Document* document_ = nullptr;
    core::sketch::SketchPlane plane_;
    std::string name_;
    std::string sketchId_;
    std::optional<std::pair<std::string, std::string>> hostAttachment_;
};

} // namespace onecad::app::commands

#endif // ONECAD_APP_COMMANDS_ADDSKETCHCOMMAND_H
