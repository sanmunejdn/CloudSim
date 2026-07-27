/**
 * @file RenameSketchCommand.cpp
 */
#include "RenameSketchCommand.h"
#include "../document/Document.h"

namespace onecad::app::commands {

RenameSketchCommand::RenameSketchCommand(Document* document,
                                         const std::string& sketchId,
                                         const std::string& newName)
    : document_(document), sketchId_(sketchId), newName_(newName) {
}

bool RenameSketchCommand::execute() {
    if (!document_ || sketchId_.empty()) {
        return false;
    }
    if (!document_->getSketch(sketchId_)) {
        return false;
    }

    oldName_ = document_->getSketchName(sketchId_);
    document_->setSketchName(sketchId_, newName_);
    return true;
}

bool RenameSketchCommand::undo() {
    if (!document_ || sketchId_.empty()) {
        return false;
    }
    document_->setSketchName(sketchId_, oldName_);
    return true;
}

} // namespace onecad::app::commands
