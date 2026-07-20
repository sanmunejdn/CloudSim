#ifndef DATA_BACKENDHIERARCHYCHANGE_H
#define DATA_BACKENDHIERARCHYCHANGE_H

/// @file BackendHierarchyChange.h
/// @brief 后端对象图结构变更（非属性/位姿编辑）

#include "data_global.h"

#include <functional>
#include <string>

/// 后端对象图结构变更（非属性/位姿编辑）
enum class BackendHierarchyChangeKind
{
	DataRegistered,
	DataUnregistered,
	EdgeAttached,
	EdgeDetached,
	AllCleared
};

/// parentId/childId 含义随 kind 变化
struct DATA_EXPORT BackendHierarchyChangeEvent
{
	BackendHierarchyChangeKind kind = BackendHierarchyChangeKind::AllCleared;
	std::string parentId;
	std::string childId;
};

using BackendHierarchyObserver = std::function<void(const BackendHierarchyChangeEvent&)>;

#endif // DATA_BACKENDHIERARCHYCHANGE_H
