/**
 * @file RollbackCommand.cpp
 * @brief Implementation of RollbackCommand.
 */
#include "RollbackCommand.h"
#include "../document/Document.h"
#include "../history/DependencyGraph.h"
#include "../history/RegenerationEngine.h"

namespace onecad::app::commands {

RollbackCommand::RollbackCommand(Document* document, const std::string& targetOpId)
    : document_(document)
    , targetOpId_(targetOpId) {
}

bool RollbackCommand::execute() {
    if (!document_) return false;

    const int targetIndex = document_->operationIndex(targetOpId_);
    if (targetIndex < 0) {
        return false;
    }

    previousSuppression_.clear();
    previousAppliedOpCount_ = document_->appliedOpCount();
    targetAppliedOpCount_ = static_cast<std::size_t>(targetIndex) + 1;

    // Build dependency graph
    history::DependencyGraph graph;
    graph.rebuildFromOperations(document_->operations());

    // Get all downstream operations from target
    auto downstream = graph.getDownstream(targetOpId_);

    // Suppress downstream operations and capture previous states
    for (const auto& opId : downstream) {
        const bool wasSuppressed = document_->isOperationSuppressed(opId);
        previousSuppression_[opId] = wasSuppressed;
        document_->setOperationSuppressed(opId, true);
    }

    document_->setAppliedOpCount(targetAppliedOpCount_);

    history::RegenerationEngine engine(document_);
    engine.regenerateToAppliedCount(document_->appliedOpCount());

    document_->setModified(true);
    return true;
}

bool RollbackCommand::undo() {
    if (!document_) return false;

    // Restore suppression states
    for (const auto& [opId, wasSuppressed] : previousSuppression_) {
        document_->setOperationSuppressed(opId, wasSuppressed);
    }

    document_->setAppliedOpCount(previousAppliedOpCount_);

    history::RegenerationEngine engine(document_);
    engine.regenerateToAppliedCount(document_->appliedOpCount());

    document_->setModified(true);
    return true;
}

} // namespace onecad::app::commands
