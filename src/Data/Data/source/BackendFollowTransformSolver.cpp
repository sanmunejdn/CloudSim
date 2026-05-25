#include "BackendFollowTransformSolver.h"

#include "BackendDataManager.h"
#include "FollowAttachmentComponent.h"
#include "MeshBackendData.h"
#include "PointCloudBackendData.h"

#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace
{
BackendVec3 modelCenterForData(const BackendDataBase& data)
{
	if (const auto* pc = dynamic_cast<const PointCloudBackendData*>(&data))
	{
		const auto& xyz = pc->pointPositionsXyz();
		if (xyz.size() < 3U || (xyz.size() % 3U) != 0U)
		{
			return BackendVec3{};
		}
		float minx = xyz[0], maxx = xyz[0], miny = xyz[1], maxy = xyz[1], minz = xyz[2], maxz = xyz[2];
		for (std::size_t i = 0; i + 2 < xyz.size(); i += 3U)
		{
			const float x = xyz[i], y = xyz[i + 1], z = xyz[i + 2];
			minx = std::min(minx, x);
			maxx = std::max(maxx, x);
			miny = std::min(miny, y);
			maxy = std::max(maxy, y);
			minz = std::min(minz, z);
			maxz = std::max(maxz, z);
		}
		return BackendVec3{ 0.5 * (static_cast<double>(minx) + static_cast<double>(maxx)),
			0.5 * (static_cast<double>(miny) + static_cast<double>(maxy)),
			0.5 * (static_cast<double>(minz) + static_cast<double>(maxz)) };
	}
	if (const auto* mesh = dynamic_cast<const MeshBackendData*>(&data))
	{
		const auto& soup = mesh->triangleSoup();
		if (soup.size() < 3U || (soup.size() % 3U) != 0U)
		{
			return BackendVec3{};
		}
		float minx = soup[0], maxx = soup[0], miny = soup[1], maxy = soup[1], minz = soup[2], maxz = soup[2];
		for (std::size_t i = 0; i + 2 < soup.size(); i += 3U)
		{
			const float x = soup[i], y = soup[i + 1], z = soup[i + 2];
			minx = std::min(minx, x);
			maxx = std::max(maxx, x);
			miny = std::min(miny, y);
			maxy = std::max(maxy, y);
			minz = std::min(minz, z);
			maxz = std::max(maxz, z);
		}
		return BackendVec3{ 0.5 * (static_cast<double>(minx) + static_cast<double>(maxx)),
			0.5 * (static_cast<double>(miny) + static_cast<double>(maxy)),
			0.5 * (static_cast<double>(minz) + static_cast<double>(maxz)) };
	}
	return BackendVec3{};
}

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

void BackendFollowTransformSolver::solve(
	BackendDataManager& mgr, const WorldMatQuery& worldQuery, const std::string& skipUpdatingFollowerId,
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
		auto comp = std::dynamic_pointer_cast<FollowAttachmentComponent>(d->getComponent(FollowAttachmentComponent::typeKeyStatic()));
		if (!comp || !comp->enabled())
		{
			continue;
		}
		const std::string tid = comp->targetBackendId();
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

	std::vector<std::string> topo;  // 拓扑序
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
	if (topo.size() != nodes.size())
	{
		// 环或不一致图：跳过自动解
		return;
	}

	std::unordered_map<std::string, BackendMat4> worldCache;
	auto getWorld = [&](const std::string& bid, BackendMat4& w) -> bool {
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
		if (!follower || !comp || comp->solverPaused())
		{
			continue;
		}
		const std::string tid = comp->targetBackendId();
		BackendMat4 wT{};
		if (!getWorld(tid, wT))
		{
			continue;
		}
		const BackendVec3 lp = comp->localPosition();
		const BackendVec3 le = comp->localEulerDeg();
		const BackendMat4 lMat = [&]() {
			const BackendMat4 t = BackendMat4::translate(lp.x, lp.y, lp.z);
			const BackendMat4 r = BackendMat4::rotateEulerDeg(le.x, le.y, le.z);
			BackendMat4 o{};
			backend_mat4_multiply(t, r, o);
			return o;
		}();
		BackendMat4 wF{};
		backend_mat4_multiply(wT, lMat, wF);
		worldCache[fid] = wF;

		if (!follower->hasPoseProperty())
		{
			continue;
		}
		if (limitPoseUpdateToFollowerIds && !limitPoseUpdateToFollowerIds->empty()
			&& !limitPoseUpdateToFollowerIds->count(fid))
		{
			continue;
		}
		const BackendVec3 c = modelCenterForData(*follower);
		BackendVec3 newPose{};
		BackendVec3 newEuler{};
		backend_pose_euler_from_world_mat(wF, c, newPose, newEuler);
		follower->setPose(newPose);
		follower->setRotation(newEuler);
	}
}
