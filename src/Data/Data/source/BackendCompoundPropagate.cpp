/// @file BackendCompoundPropagate.cpp
/// @brief 同部件 compound 刚体 Δ

#include "BackendCompoundPropagate.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "FollowAttachmentComponent.h"

#include <queue>

namespace backend_compound
{
namespace
{
bool hasEnabledFollow(const BackendDataBase& data)
{
	const auto follow = std::dynamic_pointer_cast<FollowAttachmentComponent>(
		data.getComponent(FollowAttachmentComponent::typeKeyStatic()));
	return follow && follow->enabled() && !follow->targetBackendId().empty();
}
} // namespace

std::unordered_set<std::string> propagateRigidDelta(BackendDataManager& mgr, const std::string& rootId,
													const BackendMat4& delta,
													const std::unordered_set<std::string>* skipIds,
													const WorldWriteFn& writeWorld)
{
	std::unordered_set<std::string> touched;
	if (rootId.empty())
	{
		return touched;
	}

	std::queue<std::string> queue;
	std::unordered_set<std::string> visited;
	visited.insert(rootId);
	queue.push(rootId);
	while (!queue.empty())
	{
		const std::string cur = queue.front();
		queue.pop();
		for (const std::string& childId : mgr.childrenOf(cur))
		{
			if (!visited.insert(childId).second)
			{
				continue;
			}
			queue.push(childId);
			if (skipIds && skipIds->count(childId) != 0)
			{
				continue;
			}
			const auto child = mgr.getData(childId);
			if (!child || !child->hasPoseProperty())
			{
				continue;
			}
			// 跨部件 follower 位姿由 Follow 独占，勿被 compound 抢写
			if (hasEnabledFollow(*child))
			{
				continue;
			}
			BackendMat4 childNew{};
			(void)backend_mat4_multiply(delta, child->worldMatrix(), childNew);
			child->setWorldMatrix(childNew);
			if (writeWorld)
			{
				writeWorld(childId, childNew);
			}
			touched.insert(childId);
		}
	}
	return touched;
}

std::unordered_set<std::string> propagateFromWorldChange(BackendDataManager& mgr, const std::string& rootId,
														 const BackendMat4& wOld, const BackendMat4& wNew,
														 const std::unordered_set<std::string>* skipIds,
														 const WorldWriteFn& writeWorld)
{
	BackendMat4 invOld{};
	BackendMat4 delta{};
	(void)backend_mat4_invert_rigid(wOld, invOld);
	(void)backend_mat4_multiply(wNew, invOld, delta);
	return propagateRigidDelta(mgr, rootId, delta, skipIds, writeWorld);
}

} // namespace backend_compound
