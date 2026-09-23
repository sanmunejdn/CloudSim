/**
 * @file OperationCommandUtils.h
 * @brief Helpers for history commands.
 */
#ifndef GEOMETRICMODELINGPLUGIN_OPERATIONCOMMANDUTILS_H
#define GEOMETRICMODELINGPLUGIN_OPERATIONCOMMANDUTILS_H

#include "../document/Document.h"
#include "../history/RegenerationEngine.h"

namespace onecad::app::commands
{
inline bool regenerateDocument(Document* document)
{
	if (!document)
	{
		return false;
	}
	history::RegenerationEngine engine(document);
	auto result = engine.regenerateToAppliedCount(document->appliedOpCount());
	return result.status != history::RegenStatus::CriticalFailure;
}

inline bool regenerateDocumentStrict(Document* document)
{
	if (!document)
	{
		return false;
	}
	history::RegenerationEngine engine(document);
	auto result = engine.regenerateToAppliedCount(document->appliedOpCount());
	return result.status == history::RegenStatus::Success;
}

} // namespace onecad::app::commands

#endif // GEOMETRICMODELINGPLUGIN_OPERATIONCOMMANDUTILS_H
