#pragma once

#include "BackendFollowMath.h"
#include "data_global.h"

#include <functional>
#include <string>
#include <unordered_set>

class BackendDataBase;
class BackendDataManager;

/// Resolves follower poses from \ref FollowAttachmentComponent chains (DAG). Uses optional OSG world query
/// when provided so robot / live gizmo targets stay consistent with the scene.
class DATA_EXPORT BackendFollowTransformSolver
{
public:
	using WorldMatQuery = std::function<bool(const std::string& backendId, BackendMat4& outWorld)>;

	/// Updates followers in topological order. Skips \a skipUpdatingFollowerId if non-empty (e.g. user dragging that follower).
	/// When \a limitPoseUpdateToFollowerIds is non-null, only followers whose id appears in the set receive pose writes
	/// (full graph is still traversed for cycle checks and world cache). When null, all followers are updated.
	static void solve(BackendDataManager& mgr, const WorldMatQuery& worldQuery, const std::string& skipUpdatingFollowerId,
		const std::unordered_set<std::string>* limitPoseUpdateToFollowerIds);
};
