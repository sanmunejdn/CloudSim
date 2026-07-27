/**
 * @file RenameBodyCommand.cpp
 */
#include "RenameBodyCommand.h"
#include "../document/Document.h"

namespace onecad::app::commands {

RenameBodyCommand::RenameBodyCommand(Document* document,
                                     const std::string& bodyId,
                                     const std::string& newName)
    : document_(document), bodyId_(bodyId), newName_(newName) {
}

bool RenameBodyCommand::execute() {
    if (!document_ || bodyId_.empty()) {
        return false;
    }
    if (!document_->getBodyShape(bodyId_)) {
        return false;
    }

    oldName_ = document_->getBodyName(bodyId_);
    document_->setBodyName(bodyId_, newName_);
    return true;
}

bool RenameBodyCommand::undo() {
    if (!document_ || bodyId_.empty()) {
        return false;
    }
    document_->setBodyName(bodyId_, oldName_);
    return true;
}

} // namespace onecad::app::commands
