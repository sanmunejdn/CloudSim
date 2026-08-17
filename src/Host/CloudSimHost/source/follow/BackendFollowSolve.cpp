/// @file BackendFollowSolve.cpp
/// @brief Follow 脏集求解

#include "BackendFollowSolve.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BackendFollowTransformSolver.h"
#include "CoreTypes.h"
#include "DocumentHost.h"
#include "FollowAttachmentComponent.h"
#include "IRenderView.h"
#include "OsgWidget.h"
#include "OsgWidgetSceneBridge.h"

#include <QString>
#include <unordered_set>

#include <osg/Matrixd>

namespace cloudsim::host
{
void runBackendFollowSolveAndSync(DocumentHost& page, OsgWidget* osg, const FollowSolveContext* ctx,
								  const std::string* manualPoseAuthorityBackendId)
{
	if (ctx && ctx->skipAll && ctx->skipAll())
	{
		return;
	}

	// FK 与 Follow 写同一批连杆会拆散装配；先卸再解
	page.stripKinematicsOwnedFollowAttachments();

	BackendDataManager& mgr = page.backend();
	const bool forced = page.takeFollowSolveForced();
	auto& dirty = page.followDirtyBackendIds();
	const bool gizmoDrag = osg && osg->isTransformGizmoDragging();
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

	// 无 OSG 时 query 失败 → solver 回落 BackendData pose（Web FK 已写连杆矩阵）
	const BackendFollowTransformSolver::WorldMatQuery worldQuery = [osg](const std::string& bid,
																		 BackendMat4& out) -> bool
	{
		if (!osg)
		{
			return false;
		}
		osg::Matrixd om;
		if (!osg->getBackendRootWorldMatrix(bid, om))
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
		if (page.isKinematicsOwnedBackend(d->id()))
		{
			continue;
		}
		auto comp = std::dynamic_pointer_cast<FollowAttachmentComponent>(
			d->getComponent(FollowAttachmentComponent::typeKeyStatic()));
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

void afterFollowPropertyEdited(DocumentHost& host, const QString& backendId, const QString& propertyKey,
							   const QString& valueText)
{
	const std::string id = backendId.toStdString();
	const auto data = host.backend().getData(id);
	if (!data)
	{
		return;
	}
	if (host.isKinematicsOwnedBackend(id))
	{
		if (data->hasComponent(FollowAttachmentComponent::typeKeyStatic()))
		{
			data->removeComponent(FollowAttachmentComponent::typeKeyStatic());
			host.invalidateFollowReverseIndex();
		}
		return;
	}
	const BackendFollowTransformSolver::WorldMatQuery worldQuery = [&host](const std::string& bid,
																		   BackendMat4& out) -> bool
	{
		cloudsim::core::Mat4 mat;
		if (!host.render().getWorldMatrix(QString::fromStdString(bid), mat))
		{
			return false;
		}
		for (int i = 0; i < 16; ++i)
		{
			out.v[i] = mat[static_cast<size_t>(i)];
		}
		return true;
	};
	if (propertyKey == QStringLiteral("follow.targetId") || propertyKey == QStringLiteral("follow.targetName") ||
		(propertyKey == QStringLiteral("follow.enabled") &&
		 (valueText == QStringLiteral("1") || valueText.compare(QStringLiteral("true"), Qt::CaseInsensitive) == 0)))
	{
		(void)FollowAttachmentComponent::recomputeLocalFromCurrentWorld(host.backend(), worldQuery, *data, nullptr);
		// 只脏本对象及下游 follower；全量 forced 会误伤其它链（旧工程连杆 Follow）
	}
	host.markFollowAttachmentDirtyFromBackendMove(id);
	host.invalidateFollowReverseIndex();
}

} // namespace cloudsim::host
