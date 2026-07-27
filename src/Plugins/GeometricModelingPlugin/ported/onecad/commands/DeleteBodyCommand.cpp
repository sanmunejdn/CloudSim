/**
 * @file DeleteBodyCommand.cpp
 */
#include "DeleteBodyCommand.h"
#include "../document/Document.h"

namespace onecad::app::commands {

DeleteBodyCommand::DeleteBodyCommand(Document* document, const std::string& bodyId)
    : document_(document), bodyId_(bodyId) {
}

bool DeleteBodyCommand::execute() {
    if (!document_ || bodyId_.empty()) {
        return false;
    }

    // Save state for undo
    const TopoDS_Shape* shape = document_->getBodyShape(bodyId_);
    if (!shape) {
        return false;
    }
    savedShape_ = *shape;
    savedName_ = document_->getBodyName(bodyId_);
    savedVisible_ = document_->isBodyVisible(bodyId_);

    return document_->removeBody(bodyId_);
}

bool DeleteBodyCommand::undo() {
    if (!document_ || bodyId_.empty() || savedShape_.IsNull()) {
        return false;
    }

    if (!document_->addBodyWithId(bodyId_, savedShape_, savedName_)) {
        return false;
    }

    // Restore visibility
    document_->setBodyVisible(bodyId_, savedVisible_);
    return true;
}

} // namespace onecad::app::commands
