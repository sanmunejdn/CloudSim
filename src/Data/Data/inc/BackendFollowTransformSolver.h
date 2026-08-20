#ifndef DATA_BACKENDFOLLOWTRANSFORMSOLVER_H
#define DATA_BACKENDFOLLOWTRANSFORMSOLVER_H

/// @file BackendFollowTransformSolver.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 沿 FollowAttachment DAG 解 follower 位姿；可选 OSG 世界查询与场景/机器人对齐

#include "data_global.h"

#include "BackendFollowMath.h"

#include <functional>
#include <string>
#include <unordered_set>

class BackendDataBase;
class BackendDataManager;

/// 沿 FollowAttachment DAG 解 follower 位姿；可选 OSG 世界查询与场景/机器人对齐
class DATA_EXPORT BackendFollowTransformSolver
{
public:
	using WorldMatQuery = std::function<bool(const std::string& backendId, BackendMat4& outWorld)>;

	/// 拓扑序更新 follower；skipUpdatingFollowerId 非空则跳过（如拖拽中）
	/// limitPoseUpdateToFollowerIds 非空时仅写集合内 id（仍全图遍历做环检测）
	static void solve(BackendDataManager& mgr, const WorldMatQuery& worldQuery,
					  const std::string& skipUpdatingFollowerId,
					  const std::unordered_set<std::string>* limitPoseUpdateToFollowerIds);
};

#endif // DATA_BACKENDFOLLOWTRANSFORMSOLVER_H
