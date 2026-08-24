/// @file BackendVisualEnsure.cpp
/// @brief ensureVisual 实现：委托 sceneFacade + SyncEngine

#include "visual/BackendVisualEnsure.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BackendSceneDocumentFacade.h"
#include "DocumentHost.h"
#include "visual/VisualAspect.h"

namespace cloudsim::host
{
EnsureVisualResult ensureVisual(DocumentHost& host, const std::string& backendId, const EnsureVisualPolicy policy,
								const EnsureVisualOptions& opts)
{
	EnsureVisualResult result;
	const auto obj = host.backend().getData(backendId);
	if (!obj)
	{
		result.error = QStringLiteral("backend not found");
		return result;
	}
	BackendSceneDocumentFacade facade = host.sceneFacade();
	const bool hasBranch = facade.entity(backendId).hasSceneBranch();
	if (policy == EnsureVisualPolicy::TransformOnly)
	{
		if (!hasBranch)
		{
			result.error = QStringLiteral("no visual branch");
			return result;
		}
		host.markVisualDirty(backendId, VisualAspect::Transform | VisualAspect::Selection);
		result.ok = host.flushVisualSync();
		return result;
	}
	if (hasBranch && policy != EnsureVisualPolicy::FullRebuild)
	{
		host.markVisualDirty(backendId, VisualAspect::Transform | VisualAspect::Selection);
		result.ok = host.flushVisualSync();
		result.createdBranch = false;
		return result;
	}
	host.ensureSelectionVisualForBackend(backendId, opts.urdfLinkMesh);
	result.createdBranch = facade.entity(backendId).hasSceneBranch();
	host.markVisualDirty(backendId, VisualAspect::Transform | VisualAspect::Selection);
	result.ok = host.flushVisualSync();
	return result;
}

} // namespace cloudsim::host
