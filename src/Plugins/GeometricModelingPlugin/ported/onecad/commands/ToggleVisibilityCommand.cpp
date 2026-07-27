/**
 * @file ToggleVisibilityCommand.cpp
 */
#include "ToggleVisibilityCommand.h"
#include "../document/Document.h"

namespace onecad::app::commands {

ToggleVisibilityCommand::ToggleVisibilityCommand(Document* document,
                                                 const std::string& itemId,
                                                 ItemType type,
                                                 bool visible)
    : document_(document), itemId_(itemId), type_(type), newVisible_(visible) {
}

bool ToggleVisibilityCommand::execute() {
    if (!document_ || itemId_.empty()) {
        return false;
    }

    if (type_ == ItemType::Body) {
        if (!document_->getBodyShape(itemId_)) {
            return false;
        }
        oldVisible_ = document_->isBodyVisible(itemId_);
        document_->setBodyVisible(itemId_, newVisible_);
    } else {
        if (!document_->getSketch(itemId_)) {
            return false;
        }
        oldVisible_ = document_->isSketchVisible(itemId_);
        document_->setSketchVisible(itemId_, newVisible_);
    }

    return true;
}

bool ToggleVisibilityCommand::undo() {
    if (!document_ || itemId_.empty()) {
        return false;
    }

    if (type_ == ItemType::Body) {
        document_->setBodyVisible(itemId_, oldVisible_);
    } else {
        document_->setSketchVisible(itemId_, oldVisible_);
    }

    return true;
}

} // namespace onecad::app::commands
