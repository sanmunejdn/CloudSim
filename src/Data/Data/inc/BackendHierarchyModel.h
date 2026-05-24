#pragma once

#include "BackendHierarchyChange.h"
#include "data_global.h"

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class BackendDataManager;

/// Incremental mirror of \ref BackendHierarchyChangeKind events from a \ref BackendDataManager,
/// with cached \c subtreeIds (root + reachable descendants, unique in DAG).
class DATA_EXPORT BackendHierarchyModel
{
public:
	explicit BackendHierarchyModel(BackendDataManager& manager);
	~BackendHierarchyModel();

	BackendHierarchyModel(const BackendHierarchyModel&) = delete;
	BackendHierarchyModel& operator=(const BackendHierarchyModel&) = delete;

	/// Rebuild mirror from manager (caller must not hold the manager's write lock).
	void resyncFrom(const BackendDataManager& manager);

	/// Root first, then BFS over \c m_children. Reference valid until the next structural change
	/// invalidates this root's cache entry (or use only synchronously on the UI thread).
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
