#include "SketchDragGestureCommand.h"

#include "../document/Document.h"
#include "../../core/sketch/Sketch.h"

#include <memory>
#include <utility>

namespace onecad::app::commands {

SketchDragGestureCommand::SketchDragGestureCommand(Document* document, std::string sketchId,
                                                   std::string label)
    : document_(document),
      sketchId_(std::move(sketchId)),
      label_(std::move(label)) {
}

bool SketchDragGestureCommand::beginGesture() {
    if (!document_ || sketchId_.empty()) {
        return false;
    }

    began_ = false;
    beforeJson_.clear();
    beforeName_.clear();
    beforeVisible_ = true;
    afterJson_.clear();
    afterName_.clear();
    afterVisible_ = true;
    finalized_ = false;
    changed_ = false;

    began_ = snapshotSketchState(&beforeJson_, &beforeName_, &beforeVisible_);
    return began_;
}

bool SketchDragGestureCommand::finalizeGesture() {
    if (!began_) {
        return false;
    }

    finalized_ = false;
    changed_ = false;

    afterJson_.clear();
    afterName_.clear();
    afterVisible_ = true;
    if (!snapshotSketchState(&afterJson_, &afterName_, &afterVisible_)) {
        return false;
    }

    changed_ = (beforeJson_ != afterJson_) ||
               (beforeName_ != afterName_) ||
               (beforeVisible_ != afterVisible_);
    finalized_ = true;
    return true;
}

bool SketchDragGestureCommand::restoreBeginState() {
    if (!began_ || beforeJson_.empty()) {
        return false;
    }
    return restoreSketchState(beforeJson_, beforeName_, beforeVisible_);
}

void SketchDragGestureCommand::cancelGesture() {
    began_ = false;
    finalized_ = false;
    changed_ = false;
    beforeJson_.clear();
    beforeName_.clear();
    afterJson_.clear();
    afterName_.clear();
    beforeVisible_ = true;
    afterVisible_ = true;
}

bool SketchDragGestureCommand::execute() {
    if (!finalized_ || !changed_) {
        return false;
    }
    return restoreSketchState(afterJson_, afterName_, afterVisible_);
}

bool SketchDragGestureCommand::undo() {
    if (!finalized_ || !changed_) {
        return false;
    }
    return restoreSketchState(beforeJson_, beforeName_, beforeVisible_);
}

bool SketchDragGestureCommand::snapshotSketchState(std::string* outJson,
                                                   std::string* outName,
                                                   bool* outVisible) const {
    if (!document_ || sketchId_.empty() || !outJson || !outName || !outVisible) {
        return false;
    }
    const core::sketch::Sketch* sketch = document_->getSketch(sketchId_);
    if (!sketch) {
        return false;
    }

    *outJson = sketch->toJson();
    *outName = document_->getSketchName(sketchId_);
    *outVisible = document_->isSketchVisible(sketchId_);
    return !outJson->empty();
}

bool SketchDragGestureCommand::restoreSketchState(const std::string& json,
                                                  const std::string& name,
                                                  bool visible) {
    if (!document_ || sketchId_.empty() || json.empty()) {
        return false;
    }
    auto sketch = core::sketch::Sketch::fromJson(json);
    if (!sketch) {
        return false;
    }
    if (!document_->removeSketch(sketchId_)) {
        return false;
    }
    if (!document_->addSketchWithId(sketchId_, std::move(sketch), name)) {
        return false;
    }
    document_->setSketchVisible(sketchId_, visible);
    return true;
}

}
