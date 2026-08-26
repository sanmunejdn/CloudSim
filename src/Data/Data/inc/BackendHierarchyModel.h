#ifndef DATA_BACKENDHIERARCHYMODEL_H
#define DATA_BACKENDHIERARCHYMODEL_H

/// @file BackendHierarchyModel.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief BackendDataManager 层级变更的增量镜像

#include "data_global.h"

#include "BackendHierarchyChange.h"

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class BackendDataManager;

/// BackendDataManager 层级变更的增量镜像（**UI 线程专属**：构造、resync、subtreeIds、观察者回调均须在 UI 线程调用）
/// 缓存 subtreeIds（根+可达后代，DAG 去重）
class DATA_EXPORT BackendHierarchyModel
{
public:
	explicit BackendHierarchyModel(BackendDataManager& manager);
	~BackendHierarchyModel();

	BackendHierarchyModel(const BackendHierarchyModel&) = delete;
	BackendHierarchyModel& operator=(const BackendHierarchyModel&) = delete;

	/// 从 manager 重建镜像（调用方勿持 manager 写锁）
	void resyncFrom(const BackendDataManager& manager);

	/// 根优先 BFS；按值返回，UI 线程同步使用
	std::vector<std::string> subtreeIds(const std::string& rootId) const;

private:
	void onHierarchyChange(const BackendHierarchyChangeEvent& event);
	void invalidateSubtreeCachesForSeeds(const std::vector<std::string>& seeds);
	void eraseNodeFromMirror(const std::string& id);
	std::vector<std::string> buildSubtreeBfs(const std::string& rootId) const;

	BackendDataManager& m_manager;
	std::unordered_map<std::string, std::unordered_set<std::string>> m_children;
	std::unordered_map<std::string, std::unordered_set<std::string>> m_parents;
	std::unordered_set<std::string> m_nodes;
	mutable std::unordered_map<std::string, std::vector<std::string>> m_subtreeCache;
};

#endif // DATA_BACKENDHIERARCHYMODEL_H
