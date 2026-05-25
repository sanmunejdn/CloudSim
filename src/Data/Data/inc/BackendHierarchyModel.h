#pragma once

#include "BackendHierarchyChange.h"
#include "data_global.h"

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class BackendDataManager;

/// BackendDataManager 层级变更的增量镜像
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

	/// 根优先 BFS；下次结构变更前有效
	/// 或仅在 UI 线程同步使用
	const std::vector<std::string>& subtreeIds(const std::string& rootId);
	const std::vector<std::string>& subtreeIds(const std::string& rootId) const;

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
	mutable std::vector<std::string> m_emptySubtree;
};
