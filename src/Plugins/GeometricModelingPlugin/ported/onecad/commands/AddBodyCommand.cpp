/**
 * @file AddBodyCommand.cpp
 */
#include "AddBodyCommand.h"
#include "../document/Document.h"

#include <QUuid>

namespace onecad::app::commands {

AddBodyCommand::AddBodyCommand(Document* document,
                               const TopoDS_Shape& shape,
                               const std::string& bodyId,
                               const std::string& bodyName)
    : document_(document),
      shape_(shape),
      bodyId_(bodyId),
      bodyName_(bodyName) {
}

bool AddBodyCommand::execute() {
    if (!document_ || shape_.IsNull()) {
        return false;
    }
    if (bodyId_.empty()) {
        bodyId_ = QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString();
    }
    if (!document_->addBodyWithId(bodyId_, shape_, bodyName_)) {
        return false;
    }
    if (bodyName_.empty()) {
        bodyName_ = document_->getBodyName(bodyId_);
    }
    return true;
}

bool AddBodyCommand::undo() {
    if (!document_ || bodyId_.empty()) {
        return false;
    }
    return document_->removeBody(bodyId_);
}

} // namespace onecad::app::commands
