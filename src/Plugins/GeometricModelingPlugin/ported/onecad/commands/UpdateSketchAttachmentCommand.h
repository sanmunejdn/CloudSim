/**
 * @file UpdateSketchAttachmentCommand.h
 * @brief Re-sync a host-attached sketch's (frozen) plane to its host face's current position.
 *
 * Sketch-on-face attachment is frozen at creation. This command lets the user explicitly
 * re-derive the sketch plane from the host face's current geometry (after upstream edits
 * moved it) and regenerate downstream features.
 */
#ifndef ONECAD_APP_COMMANDS_UPDATESKETCHATTACHMENTCOMMAND_H
#define ONECAD_APP_COMMANDS_UPDATESKETCHATTACHMENTCOMMAND_H

#include "Command.h"
#include "../../core/sketch/Sketch.h"  // core::sketch::SketchPlane

#include <string>

namespace onecad::app {
class Document;
}

namespace onecad::app::commands {

class UpdateSketchAttachmentCommand : public Command {
public:
    UpdateSketchAttachmentCommand(Document* document, std::string sketchId);

    bool execute() override;
    bool undo() override;
    std::string label() const override { return "Update Sketch Attachment"; }

private:
    Document* document_ = nullptr;
    std::string sketchId_;
    core::sketch::SketchPlane oldPlane_;
    std::string oldBodyId_;
    std::string oldFaceId_;
    bool hasOldState_ = false;
};

} // namespace onecad::app::commands

#endif // ONECAD_APP_COMMANDS_UPDATESKETCHATTACHMENTCOMMAND_H
