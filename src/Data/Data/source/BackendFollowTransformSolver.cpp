/// @file BackendFollowTransformSolver.cpp
/// @brief Follow 变换求解

#include "BackendFollowTransformSolver.h"

#include "BackendDataManager.h"
#include "FollowAttachmentComponent.h"
#include "RunLogger.h"

#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace
{
bool tryWorldMatForData(const BackendDataBase& data, const BackendFollowTransformSolver::WorldMatQuery& worldQuery,
						BackendMat4& outWorld)
{
	if (worldQuery && worldQuery(data.id(), outWorld))
	{
		return true;
	}
	if (!data.hasPoseProperty())
	{
		return false;
	}
	outWorld = data.worldMatrix();
	return true;
}

} // namespace

void BackendFollowTransformSolver::solve(BackendDataManager& mgr, const WorldMatQuery& worldQuery,
										 const std::string& skipUpdatingFollowerId,
										 const std::unordered_set<std::string>* limitPoseUpdateToFollowerIds)
{
	const auto all = mgr.listData();
	std::unordered_map<std::string, std::string> followerToTarget;
	std::unordered_map<std::string, std::vector<std::string>> targetToFollowers;
	std::unordered_set<std::string> followers;
	std::unordered_set<std::string> nodes;
	for (const auto& d : all)
	{
		if (!d)
		{
			continue;
		}
		auto comp = std::dynamic_pointer_cast<FollowAttachmentComponent>(
			d->getComponent(FollowAttachmentComponent::typeKeyStatic()));
		if (!comp)
		{
			continue;
		}
		const auto snap = comp->snapshot();
		if (!snap.enabled)
		{
			continue;
		}
		const std::string& tid = snap.targetId;
		if (tid.empty() || tid == d->id())
		{
			continue;
		}
		if (!mgr.contains(tid))
		{
			continue;
		}
		const std::string fid = d->id();
		followerToTarget[fid] = tid;
		targetToFollowers[tid].push_back(fid);
		followers.insert(fid);
		nodes.insert(fid);
		nodes.insert(tid);
	}
	if (followers.empty())
	{
		return;
	}

	std::unordered_map<std::string, int> indegree;
	for (const std::string& n : nodes)
	{
		indegree[n] = 0;
	}
	for (const std::string& f : followers)
	{
		(void)indegree[followerToTarget[f]]; // ensure target key exists
		indegree[f] = 1;
	}

	std::vector<std::string> topo; // 拓扑序
	std::vector<std::string> queue;
	queue.reserve(nodes.size());
	for (const std::string& n : nodes)
	{
		if (indegree[n] == 0)
		{
			queue.push_back(n);
		}
	}
	std::size_t qh = 0;
	while (qh < queue.size())
	{
		const std::string u = queue[qh++];
		topo.push_back(u);
		const auto it = targetToFollowers.find(u);
		if (it == targetToFollowers.end())
		{
			continue;
		}
		for (const std::string& f : it->second)
		{
			if (indegree[f] > 0)
			{
				--indegree[f];
				if (indegree[f] == 0)
				{
					queue.push_back(f);
				}
			}
		}
	}
	// 环只影响所在强连通分量：未入 topo 的节点即环上节点，单独告警并跳过，
	// 不连累场景里其余正常 Follow 关系
	if (topo.size() != nodes.size())
	{
		std::string cyclicIds;
		for (const std::string& n : nodes)
		{
			if (indegree[n] > 0)
			{
				if (!cyclicIds.empty())
				{
					cyclicIds += ", ";
				}
				cyclicIds += n;
			}
		}
		RunLogger::warn("[FollowSolver] cycle detected, skip cyclic followers only: " + cyclicIds);
	}

	std::unordered_map<std::string, BackendMat4> worldCache;
	auto getWorld = [&](const std::string& bid, BackendMat4& w) -> bool
	{
		const auto itc = worldCache.find(bid);
		if (itc != worldCache.end())
		{
			w = itc->second;
			return true;
		}
		const auto obj = mgr.getData(bid);
		if (!obj)
		{
			return false;
		}
		if (!tryWorldMatForData(*obj, worldQuery, w))
		{
			return false;
		}
		worldCache[bid] = w;
		return true;
	};

	for (const std::string& fid : topo)
	{
		if (!followers.count(fid))
		{
			continue;
		}
		if (fid == skipUpdatingFollowerId)
		{
			continue;
		}
		auto follower = mgr.getData(fid);
		auto comp = follower ? std::dynamic_pointer_cast<FollowAttachmentComponent>(
								   follower->getComponent(FollowAttachmentComponent::typeKeyStatic()))
							 : nullptr;
		if (!follower || !comp)
		{
			continue;
		}
		const auto snap = comp->snapshot();
		if (snap.solverPaused)
		{
			continue;
		}
		const std::string& tid = snap.targetId;
		BackendMat4 wT{};
		if (!getWorld(tid, wT))
		{
			continue;
		}
		const BackendVec3& lp = snap.localPos;
		const BackendVec3& le = snap.localEulerDeg;
		const BackendMat4 lMat = backend_world_mat_from_pose(lp, le);
		BackendMat4 wF{};
		backend_mat4_multiply(wT, lMat, wF);

		// 跳过检查全部前置：被跳过的 follower 不写入 worldCache，
		// 避免下游 Follow 命中缓存拿到从未生效的位姿（mount/limit 场景）
		if (!follower->hasPoseProperty())
		{
			continue;
		}
		if (limitPoseUpdateToFollowerIds && !limitPoseUpdateToFollowerIds->empty() &&
			!limitPoseUpdateToFollowerIds->count(fid))
		{
			continue;
		}
		// P3-1: 与 setter 侧 no-op 阈值 1e-5 对齐（BackendDataBase.cpp:459/475），
		// 避免 1e-7~1e-5 量级的"无变化"更新白白 bump poseRevision 触发下游同步
		if (backend_mat4_nearly_equal(follower->worldMatrix(), wF, 1e-5))
		{
			continue;
		}
		// 通用扩展点：组件可声明「位姿由外部驱动」（如 mount），求解器不覆写
		if (follower->isPoseExternallyDriven())
		{
			continue;
		}
		follower->setWorldMatrix(wF);
		worldCache[fid] = wF;
	}
}
