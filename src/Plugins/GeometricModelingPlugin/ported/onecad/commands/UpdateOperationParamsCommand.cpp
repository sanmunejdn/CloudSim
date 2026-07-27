/**
 * @file UpdateOperationParamsCommand.cpp
 * @brief Implementation of UpdateOperationParamsCommand.
 */
#include "UpdateOperationParamsCommand.h"
#include "OperationCommandUtils.h"
#include "../document/Document.h"

namespace onecad::app::commands {

UpdateOperationParamsCommand::UpdateOperationParamsCommand(Document* document,
                                                           std::string opId,
                                                           OperationParams params)
    : document_(document)
    , opId_(std::move(opId))
    , newParams_(std::move(params)) {
}

bool UpdateOperationParamsCommand::execute() {
    if (!document_) {
        return false;
    }
    auto* op = document_->findOperation(opId_);
    if (!op) {
        return false;
    }
    oldParams_ = op->params;
    hasOldParams_ = true;

    if (!document_->updateOperationParams(opId_, newParams_)) {
        return false;
    }
    if (!regenerateDocumentStrict(document_)) {
        document_->updateOperationParams(opId_, oldParams_);
        if (!regenerateDocumentStrict(document_)) {
            regenerateDocument(document_);
        }
        return false;
    }
    return true;
}

bool UpdateOperationParamsCommand::undo() {
    if (!document_ || !hasOldParams_) {
        return false;
    }
    if (!document_->updateOperationParams(opId_, oldParams_)) {
        return false;
    }
    if (!regenerateDocumentStrict(document_)) {
        document_->updateOperationParams(opId_, newParams_);
        if (!regenerateDocumentStrict(document_)) {
            regenerateDocument(document_);
        }
        return false;
    }
    return true;
}

} // namespace onecad::app::commands
