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
	// gizmo 拖动不能单独当全量解：否则无关件每帧 syncOuterPat（日志里 A 被反复刷）
	if (!forced && dirty.empty())
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
	// 拖 follower 时同步烘焙 local，松手求解才不会用旧偏移拽回
	if (!skipId.empty())
	{
		bakeFollowLocalAfterManualPoseEdit(page, skipId);
	}

	// OSG 有件时优先场景矩阵（与 FK/gizmo 一致）；无 OSG 才回落 backend worldMatrix
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

	// 有脏集时只写脏 follower；forced 仍全量
	const bool usePoseLimit = !forced && !dirty.empty();
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
	if (!data->hasComponent(FollowAttachmentComponent::typeKeyStatic()))
	{
		host.invalidateFollowReverseIndex();
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

void bakeFollowLocalAfterManualPoseEdit(DocumentHost& host, const std::string& backendId)
{
	if (backendId.empty() || host.isKinematicsOwnedBackend(backendId))
	{
		return;
	}
	const auto data = host.backend().getData(backendId);
	if (!data)
	{
		return;
	}
	const auto comp = std::dynamic_pointer_cast<FollowAttachmentComponent>(
		data->getComponent(FollowAttachmentComponent::typeKeyStatic()));
	if (!comp || !comp->enabled() || comp->targetBackendId().empty())
	{
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
	(void)FollowAttachmentComponent::recomputeLocalFromCurrentWorld(host.backend(), worldQuery, *data, nullptr);
}

} // namespace cloudsim::host
