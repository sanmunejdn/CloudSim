/**
 * @file AddDatumPlaneCommand.cpp
 * @brief Implementation of AddDatumPlaneCommand.
 */
#include "AddDatumPlaneCommand.h"
#include "../document/Document.h"

namespace onecad::app::commands {

AddDatumPlaneCommand::AddDatumPlaneCommand(Document* document, DatumPlane datum)
    : document_(document)
    , datum_(std::move(datum)) {
}

bool AddDatumPlaneCommand::execute() {
    if (!document_) {
        return false;
    }
    // addDatumPlane assigns an id on first run; reuse it on redo for stability.
    const std::string id = document_->addDatumPlane(datum_, /*recompute=*/true);
    if (id.empty()) {
        return false;
    }
    datum_.id = id;
    return true;
}

bool AddDatumPlaneCommand::undo() {
    if (!document_ || datum_.id.empty()) {
        return false;
    }
    return document_->removeDatumPlane(datum_.id);
}

} // namespace onecad::app::commands
