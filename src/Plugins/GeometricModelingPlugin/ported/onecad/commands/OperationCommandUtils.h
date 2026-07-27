/**
 * @file OperationCommandUtils.h
 * @brief Helpers for history commands.
 */
#ifndef ONECAD_APP_COMMANDS_OPERATIONCOMMANDUTILS_H
#define ONECAD_APP_COMMANDS_OPERATIONCOMMANDUTILS_H

#include "../history/RegenerationEngine.h"
#include "../document/Document.h"

namespace onecad::app::commands {

inline bool regenerateDocument(Document* document) {
    if (!document) {
        return false;
    }
    history::RegenerationEngine engine(document);
    auto result = engine.regenerateToAppliedCount(document->appliedOpCount());
    return result.status != history::RegenStatus::CriticalFailure;
}

inline bool regenerateDocumentStrict(Document* document) {
    if (!document) {
        return false;
    }
    history::RegenerationEngine engine(document);
    auto result = engine.regenerateToAppliedCount(document->appliedOpCount());
    return result.status == history::RegenStatus::Success;
}

} // namespace onecad::app::commands

#endif // ONECAD_APP_COMMANDS_OPERATIONCOMMANDUTILS_H
