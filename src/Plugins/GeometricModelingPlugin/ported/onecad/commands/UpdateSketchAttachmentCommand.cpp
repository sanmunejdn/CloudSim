/**
 * @file UpdateSketchAttachmentCommand.cpp
 * @brief Implementation of UpdateSketchAttachmentCommand.
 */
#include "UpdateSketchAttachmentCommand.h"
#include "OperationCommandUtils.h"
#include "../document/Document.h"

namespace onecad::app::commands {

UpdateSketchAttachmentCommand::UpdateSketchAttachmentCommand(Document* document,
                                                             std::string sketchId)
    : document_(document)
    , sketchId_(std::move(sketchId)) {
}

bool UpdateSketchAttachmentCommand::execute() {
    if (!document_) {
        return false;
    }
    core::sketch::Sketch* sketch = document_->getSketch(sketchId_);
    if (!sketch) {
        return false;
    }
    const auto& host = sketch->hostFaceAttachment();
    if (!host || !host->isValid()) {
        return false;
    }
    const std::string bodyId = host->bodyId;

    // Re-derive the plane from the host face's CURRENT geometry (with id re-match fallback).
    auto resolved = document_->resolveHostFaceResync(sketchId_);
    if (!resolved) {
        return false;
    }

    oldPlane_ = sketch->getPlane();
    oldBodyId_ = bodyId;
    oldFaceId_ = host->faceId;
    hasOldState_ = true;

    sketch->setPlane(resolved->first);
    if (resolved->second != oldFaceId_) {
        sketch->setHostFaceAttachment(bodyId, resolved->second);  // re-matched id
    }
    // Strict, like the other document-mutating commands: any op failure after
    // the re-sync means the command failed.
    if (regenerateDocumentStrict(document_)) {
        return true;
    }

    // Restore the captured state so the failed command leaves zero net
    // mutation (it never reaches the undo stack).
    sketch->setPlane(oldPlane_);
    sketch->setHostFaceAttachment(oldBodyId_, oldFaceId_);
    hasOldState_ = false;
    regenerateDocument(document_);  // best-effort return to the pre-command state
    return false;
}

bool UpdateSketchAttachmentCommand::undo() {
    if (!document_ || !hasOldState_) {
        return false;
    }
    core::sketch::Sketch* sketch = document_->getSketch(sketchId_);
    if (!sketch) {
        return false;
    }
    sketch->setPlane(oldPlane_);
    sketch->setHostFaceAttachment(oldBodyId_, oldFaceId_);
    return regenerateDocument(document_);
}

} // namespace onecad::app::commands
