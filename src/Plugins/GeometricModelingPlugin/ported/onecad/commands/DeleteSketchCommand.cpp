/**
 * @file DeleteSketchCommand.cpp
 */
#include "DeleteSketchCommand.h"
#include "../document/Document.h"
#include "../../core/sketch/Sketch.h"

namespace onecad::app::commands {

DeleteSketchCommand::DeleteSketchCommand(Document* document, const std::string& sketchId)
    : document_(document), sketchId_(sketchId) {
}

bool DeleteSketchCommand::execute() {
    if (!document_ || sketchId_.empty()) {
        return false;
    }

    // Save state for undo
    const core::sketch::Sketch* sketch = document_->getSketch(sketchId_);
    if (!sketch) {
        return false;
    }
    savedJson_ = sketch->toJson();
    savedName_ = document_->getSketchName(sketchId_);
    savedVisible_ = document_->isSketchVisible(sketchId_);

    return document_->removeSketch(sketchId_);
}

bool DeleteSketchCommand::undo() {
    if (!document_ || sketchId_.empty() || savedJson_.empty()) {
        return false;
    }

    // Recreate sketch from JSON
    auto sketch = core::sketch::Sketch::fromJson(savedJson_);
    if (!sketch) {
        return false;
    }

    // Re-add with original ID
    if (!document_->addSketchWithId(sketchId_, std::move(sketch), savedName_)) {
        return false;
    }

    document_->setSketchVisible(sketchId_, savedVisible_);
    return true;
}

} // namespace onecad::app::commands
