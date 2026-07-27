/**
 * @file AddSketchCommand.cpp
 * @brief Implementation of AddSketchCommand.
 */
#include "AddSketchCommand.h"
#include "../document/Document.h"

#include <QUuid>

#include <memory>

namespace onecad::app::commands {

AddSketchCommand::AddSketchCommand(Document* document,
                                   const core::sketch::SketchPlane& plane,
                                   std::string name)
    : document_(document)
    , plane_(plane)
    , name_(std::move(name))
    , sketchId_(QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString()) {
}

void AddSketchCommand::setHostFaceAttachment(std::string bodyId, std::string faceId) {
    hostAttachment_ = std::make_pair(std::move(bodyId), std::move(faceId));
}

bool AddSketchCommand::execute() {
    if (!document_) {
        return false;
    }
    auto sketch = std::make_unique<core::sketch::Sketch>(plane_);
    if (hostAttachment_) {
        sketch->setHostFaceAttachment(hostAttachment_->first, hostAttachment_->second);
    }
    return document_->addSketchWithId(sketchId_, std::move(sketch), name_);
}

bool AddSketchCommand::undo() {
    if (!document_) {
        return false;
    }
    return document_->removeSketch(sketchId_);
}

} // namespace onecad::app::commands
