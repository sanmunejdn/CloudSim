#include "BackendFollowSolve.h"

#include "BackendFollowTransformSolver.h"
#include "DocumentHost.h"
#include "FollowAttachmentComponent.h"
#include "OsgWidget.h"
#include "OsgWidgetSceneBridge.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"

#include <osg/Matrixd>

#include <unordered_set>

namespace cloudsim::host
{

void runBackendFollowSolveAndSync(DocumentHost& page, OsgWidget& osg, const FollowSolveContext* ctx,
	const std::string* manualPoseAuthorityBackendId)
{
	if (osg.isTcpDragTeachActive()) // TCP 示教与 Follow 求解互斥
	{
		return;
	}
	if (ctx && ctx->skipAll && ctx->skipAll())
	{
		return;
	}

	BackendDataManager& mgr = page.backend();
	const bool forced = page.takeFollowSolveForced();
	auto& dirty = page.followDirtyBackendIds();
	const bool gizmoDrag = osg.isTransformGizmoDragging();
	if (!forced && dirty.empty() && !gizmoDrag)
	{
		return;
	}

	std::string skipId;
	std::string gizmoDragSelectedId;
	std::string manualAuthorityId;
	if (manualPoseAuthorityBackendId && !manualPoseAuthorityBackendId->empty())
	{
		manualAuthorityId = *manualPoseAuthorityBackendId;
		skipId = manualAuthorityId;
	}
	if (gizmoDrag && ctx && ctx->fillGizmoSelectedId && ctx->fillGizmoSelectedId(gizmoDragSelectedId))
	{
		skipId = gizmoDragSelectedId;
	}

	const BackendFollowTransformSolver::WorldMatQuery worldQuery = [&osg](const std::string& bid, BackendMat4& out) -> bool {
		osg::Matrixd om;
		if (!osg.getBackendRootWorldMatrix(bid, om))
		{
			return false;
		}
		for (int c = 0; c < 4; ++c)
		{
			for (int r = 0; r < 4; ++r)
			{
				out.v[c * 4 + r] = om(r, c);
			}
		}
		return true;
	};

	// 脏集模式：求解仍走全拓扑，仅对脏 id 写回 pose，避免链式 follower 矩阵陈旧
	const bool usePoseLimit = !forced && !gizmoDrag && !dirty.empty();
	const std::unordered_set<std::string>* limitPtr = usePoseLimit ? &dirty : nullptr;
	BackendFollowTransformSolver::solve(mgr, worldQuery, skipId, limitPtr);

	for (const auto& d : mgr.listData())
	{
		if (!d)
		{
			continue;
		}
		auto comp = std::dynamic_pointer_cast<FollowAttachmentComponent>(d->getComponent(FollowAttachmentComponent::typeKeyStatic()));
		if (!comp || !comp->enabled() || comp->targetBackendId().empty())
		{
			continue;
		}
		const std::string fid = d->id();
		if (!gizmoDragSelectedId.empty() && fid == gizmoDragSelectedId) // 避免与 gizmo 抢写
		{
			continue;
		}
		if (!manualAuthorityId.empty() && fid == manualAuthorityId)
		{
			continue;
		}
		if (usePoseLimit && !dirty.count(fid))
		{
			continue;
		}
		page.sceneBridge().syncOuterPatFromBackend(*d);
	}
	dirty.clear();
}

} // namespace cloudsim::host
