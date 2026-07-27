/**
 * @file RemoveOperationCommand.cpp
 * @brief Implementation of RemoveOperationCommand.
 */
#include "RemoveOperationCommand.h"
#include "OperationCommandUtils.h"
#include "../document/Document.h"

namespace onecad::app::commands {

RemoveOperationCommand::RemoveOperationCommand(Document* document, std::string opId)
    : document_(document)
    , opId_(std::move(opId)) {
}

bool RemoveOperationCommand::execute() {
    if (!document_) {
        return false;
    }
    removedIndex_ = document_->operationIndex(opId_);
    if (removedIndex_ < 0) {
        return false;
    }
    const auto* op = document_->findOperation(opId_);
    if (!op) {
        return false;
    }

    removedRecord_ = *op;
    wasSuppressed_ = document_->isOperationSuppressed(opId_);
    hasRecord_ = true;

    if (!document_->removeOperation(opId_)) {
        return false;
    }

    if (!regenerateDocument(document_)) {
        document_->insertOperation(static_cast<std::size_t>(removedIndex_), removedRecord_);
        document_->setOperationSuppressed(opId_, wasSuppressed_);
        regenerateDocument(document_);
        return false;
    }

    return true;
}

bool RemoveOperationCommand::undo() {
    if (!document_ || !hasRecord_ || removedIndex_ < 0) {
        return false;
    }

    if (!document_->insertOperation(static_cast<std::size_t>(removedIndex_), removedRecord_)) {
        return false;
    }
    document_->setOperationSuppressed(opId_, wasSuppressed_);

    if (!regenerateDocument(document_)) {
        document_->removeOperation(opId_);
        regenerateDocument(document_);
        return false;
    }

    return true;
}

} // namespace onecad::app::commands
