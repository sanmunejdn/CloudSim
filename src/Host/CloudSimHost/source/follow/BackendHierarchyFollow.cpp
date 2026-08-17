/// @file BackendHierarchyFollow.cpp
/// @brief 层级 Follow 绑定

#include "BackendHierarchyFollow.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BackendFollowTransformSolver.h"
#include "DocumentHost.h"
#include "DocumentHostAccess.h"
#include "FollowAttachmentComponent.h"
#include "OsgWidget.h"

namespace cloudsim::host
{
void applyHierarchyFollowBinding(DocumentHost& host, const std::string& childId, const std::string& parentId)
{
	if (childId.empty())
	{
		return;
	}
	OsgWidget* osg = osgWidgetFrom(host);
	const std::shared_ptr<BackendDataBase> child = host.backend().getData(childId);
	if (!child || !child->hasPoseProperty())
	{
		return;
	}
	// 机器人连杆由 FK 写位姿；层级边只保留 parent 镜像，不装 Follow
	if (host.isKinematicsOwnedBackend(childId))
	{
		if (child->hasComponent(FollowAttachmentComponent::typeKeyStatic()))
		{
			child->removeComponent(FollowAttachmentComponent::typeKeyStatic());
			host.invalidateFollowReverseIndex();
		}
		return;
	}
	if (parentId.empty())
	{
		if (const auto follow = std::dynamic_pointer_cast<FollowAttachmentComponent>(
				child->getComponent(FollowAttachmentComponent::typeKeyStatic())))
		{
			if (follow->hierarchyDriven())
			{
				child->removeComponent(FollowAttachmentComponent::typeKeyStatic());
			}
		}
		host.markFollowAttachmentDirtyFromBackendMove(childId);
		host.invalidateFollowReverseIndex();
		return;
	}
	if (!host.backend().contains(parentId))
	{
		return;
	}
	if (const auto existing = std::dynamic_pointer_cast<FollowAttachmentComponent>(
			child->getComponent(FollowAttachmentComponent::typeKeyStatic())))
	{
		// 保留用户手工配置的 Follow，勿被工程 edges 覆盖
		if (existing->enabled() && !existing->hierarchyDriven() && !existing->targetBackendId().empty())
		{
			return;
		}
	}
	if (!child->getComponent(FollowAttachmentComponent::typeKeyStatic()))
	{
		child->addComponent(std::make_shared<FollowAttachmentComponent>());
	}
	const auto follow = std::dynamic_pointer_cast<FollowAttachmentComponent>(
		child->getComponent(FollowAttachmentComponent::typeKeyStatic()));
	follow->setHierarchyDriven(true);
	follow->setEnabled(true);
	follow->setTargetBackendId(parentId);
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
	// 绑定瞬间用当前世界位姿反算 local，再交给 Follow 求解
	(void)FollowAttachmentComponent::recomputeLocalFromCurrentWorld(host.backend(), worldQuery, *child, nullptr);
	host.markFollowAttachmentDirtyFromBackendMove(childId);
	host.invalidateFollowReverseIndex();
}

} // namespace cloudsim::host
