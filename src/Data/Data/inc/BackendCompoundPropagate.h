#ifndef DATA_BACKENDCOMPOUNDPROPAGATE_H
#define DATA_BACKENDCOMPOUNDPROPAGATE_H

/// @file BackendCompoundPropagate.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 同部件 Data 子树刚体 Δ 传播（非跨部件 Follow）

#include "data_global.h"

#include "BackendFollowMath.h"

#include <functional>
#include <string>
#include <unordered_set>

class BackendDataManager;

namespace backend_compound
{
/// 可选：把新世界位姿写到 OSG / pose sink
using WorldWriteFn = std::function<void(const std::string& backendId, const BackendMat4& world)>;

/// 对 root 的 Data 子孙施加 Δ；跳过 skipIds 与自身已启用 Follow 的节点（位姿由 Follow 独占）
/// @return 被改写世界位姿的 id（不含 root）
DATA_EXPORT std::unordered_set<std::string>
propagateRigidDelta(BackendDataManager& mgr, const std::string& rootId, const BackendMat4& delta,
					const std::unordered_set<std::string>* skipIds = nullptr,
					const WorldWriteFn& writeWorld = nullptr);

/// Δ = W_new · inv(W_old)，再传播
DATA_EXPORT std::unordered_set<std::string>
propagateFromWorldChange(BackendDataManager& mgr, const std::string& rootId, const BackendMat4& wOld,
						 const BackendMat4& wNew, const std::unordered_set<std::string>* skipIds = nullptr,
						 const WorldWriteFn& writeWorld = nullptr);

} // namespace backend_compound

#endif // DATA_BACKENDCOMPOUNDPROPAGATE_H
