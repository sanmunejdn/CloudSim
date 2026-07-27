/**
 * @file SetOperationSuppressionCommand.cpp
 * @brief Implementation of SetOperationSuppressionCommand.
 */
#include "SetOperationSuppressionCommand.h"
#include "OperationCommandUtils.h"
#include "../document/Document.h"

namespace onecad::app::commands {

SetOperationSuppressionCommand::SetOperationSuppressionCommand(Document* document,
                                                               std::string opId,
                                                               bool suppressed)
    : document_(document)
    , opId_(std::move(opId))
    , newSuppressed_(suppressed) {
}

bool SetOperationSuppressionCommand::execute() {
    if (!document_) {
        return false;
    }
    oldSuppressed_ = document_->isOperationSuppressed(opId_);
    hasOldState_ = true;

    if (!document_->setOperationSuppressed(opId_, newSuppressed_)) {
        return false;
    }
    if (!regenerateDocument(document_)) {
        document_->setOperationSuppressed(opId_, oldSuppressed_);
        regenerateDocument(document_);
        return false;
    }
    return true;
}

bool SetOperationSuppressionCommand::undo() {
    if (!document_ || !hasOldState_) {
        return false;
    }
    if (!document_->setOperationSuppressed(opId_, oldSuppressed_)) {
        return false;
    }
    if (!regenerateDocument(document_)) {
        document_->setOperationSuppressed(opId_, newSuppressed_);
        regenerateDocument(document_);
        return false;
    }
    return true;
}

} // namespace onecad::app::commands
