/**
 * @file EditOperationInputCommand.cpp
 * @brief Implementation of EditOperationInputCommand.
 */
#include "EditOperationInputCommand.h"
#include "OperationCommandUtils.h"
#include "../document/Document.h"
#include "../document/OperationValidation.h"

#include <string>

namespace onecad::app::commands {

EditOperationInputCommand::EditOperationInputCommand(Document* document, std::string opId,
                                                     OperationInput newInput)
    : document_(document)
    , opId_(std::move(opId))
    , newInput_(std::move(newInput)) {
}

bool EditOperationInputCommand::execute() {
    if (!document_) {
        return false;
    }
    const OperationRecord* op = document_->findOperation(opId_);
    if (!op) {
        return false;
    }
    // Enforce the input-type contract (e.g. an extrude cannot be re-profiled to a FaceRef).
    std::string whyNot;
    if (!validateOperationInputType(op->type, newInput_, &whyNot)) {
        return false;
    }

    oldInput_ = op->input;
    hasOldInput_ = true;
    if (!document_->updateOperationInput(opId_, newInput_)) {
        return false;
    }
    // Input changed -> dependency edges shift; regenerateDocument rebuilds the graph.
    if (!regenerateDocumentStrict(document_)) {
        document_->updateOperationInput(opId_, oldInput_);
        regenerateDocument(document_);
        return false;
    }
    return true;
}

bool EditOperationInputCommand::undo() {
    if (!document_ || !hasOldInput_) {
        return false;
    }
    if (!document_->updateOperationInput(opId_, oldInput_)) {
        return false;
    }
    return regenerateDocument(document_);
}

} // namespace onecad::app::commands
