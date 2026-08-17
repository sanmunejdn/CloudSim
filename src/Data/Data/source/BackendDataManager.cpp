/// @file BackendDataManager.cpp
/// @brief BackendData 管理

#include "BackendDataManager.h"

#include "BackendRegistryBuiltins.h"

#include <algorithm>
#include <chrono>
#include <mutex>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace
{
std::vector<std::string> sortedKeys(const std::unordered_set<std::string>& ids)
{
	std::vector<std::string> out;
	out.reserve(ids.size());
	for (const std::string& id : ids)
	{
		out.push_back(id);
	}
	std::sort(out.begin(), out.end());
	return out;
}

} // namespace

BackendDataManager& BackendDataManager::instance()
{
	ensureBackendBuiltinsRegistered();
	static BackendDataManager manager;
	return manager;
}

bool BackendDataManager::registerData(const std::shared_ptr<BackendDataBase>& data)
{
	if (!data || data->id().empty())
	{
		return false;
	}

	std::unique_lock<std::shared_mutex> lock(m_mutex);
	const auto result = m_records.emplace(data->id(), data);
	if (result.second)
	{
		m_subtreeCache.clear();
		notifyHierarchyObserversLocked(
			BackendHierarchyChangeEvent{BackendHierarchyChangeKind::DataRegistered, {}, data->id()});
	}
	return result.second;
}

bool BackendDataManager::unregisterData(const std::string& id)
{
	if (id.empty())
	{
		return false;
	}

	std::unique_lock<std::shared_mutex> lock(m_mutex);
	if (m_records.erase(id) == 0)
	{
		return false;
	}

	auto parentsIt = m_parentsByChild.find(id);
	if (parentsIt != m_parentsByChild.end())
	{
		for (const std::string& parentId : parentsIt->second)
		{
			auto childSetIt = m_childrenByParent.find(parentId);
			if (childSetIt != m_childrenByParent.end())
			{
				childSetIt->second.erase(id);
				if (childSetIt->second.empty())
				{
					m_childrenByParent.erase(childSetIt);
				}
			}
		}
		m_parentsByChild.erase(parentsIt);
	}

	auto childrenIt = m_childrenByParent.find(id);
	if (childrenIt != m_childrenByParent.end())
	{
		for (const std::string& childId : childrenIt->second)
		{
			auto parentSetIt = m_parentsByChild.find(childId);
			if (parentSetIt != m_parentsByChild.end())
			{
				parentSetIt->second.erase(id);
				if (parentSetIt->second.empty())
				{
					m_parentsByChild.erase(parentSetIt);
				}
			}
		}
		m_childrenByParent.erase(childrenIt);
	}

	notifyHierarchyObserversLocked(BackendHierarchyChangeEvent{BackendHierarchyChangeKind::DataUnregistered, {}, id});
	m_subtreeCache.clear();
	return true;
}

bool BackendDataManager::contains(const std::string& id) const
{
	std::shared_lock<std::shared_mutex> lock(m_mutex);
	return m_records.find(id) != m_records.end();
}

std::shared_ptr<BackendDataBase> BackendDataManager::getData(const std::string& id) const
{
	std::shared_lock<std::shared_mutex> lock(m_mutex);
	const auto it = m_records.find(id);
	if (it == m_records.end())
	{
		return nullptr;
	}

	return it->second;
}

std::vector<std::shared_ptr<BackendDataBase>> BackendDataManager::listData() const
{
	std::shared_lock<std::shared_mutex> lock(m_mutex);
	std::vector<std::shared_ptr<BackendDataBase>> records;
	records.reserve(m_records.size());
	for (const auto& item : m_records)
	{
		records.push_back(item.second);
	}
	std::sort(records.begin(), records.end(),
			  [](const std::shared_ptr<BackendDataBase>& lhs, const std::shared_ptr<BackendDataBase>& rhs)
			  {
				  if (!lhs)
				  {
					  return rhs != nullptr;
				  }
				  if (!rhs)
				  {
					  return false;
				  }
				  return lhs->id() < rhs->id();
			  });
	return records;
}

std::vector<std::shared_ptr<BackendDataBase>> BackendDataManager::findByName(const std::string& name) const
{
	std::shared_lock<std::shared_mutex> lock(m_mutex);
	std::vector<std::shared_ptr<BackendDataBase>> out;
	for (const auto& item : m_records)
	{
		if (item.second && item.second->name() == name)
		{
			out.push_back(item.second);
		}
	}
	return out;
}

std::vector<std::shared_ptr<BackendDataBase>> BackendDataManager::findByClass(const std::string& className) const
{
	std::shared_lock<std::shared_mutex> lock(m_mutex);
	std::vector<std::shared_ptr<BackendDataBase>> out;
	for (const auto& item : m_records)
	{
		if (item.second && item.second->className() == className)
		{
			out.push_back(item.second);
		}
	}
	return out;
}

std::vector<std::shared_ptr<BackendDataBase>>
BackendDataManager::findByComponent(const std::string& componentType) const
{
	std::shared_lock<std::shared_mutex> lock(m_mutex);
	std::vector<std::shared_ptr<BackendDataBase>> out;
	for (const auto& item : m_records)
	{
		if (item.second && item.second->hasComponent(componentType))
		{
			out.push_back(item.second);
		}
	}
	return out;
}

bool BackendDataManager::attachChild(const std::string& parentId, const std::string& childId)
{
	if (parentId.empty() || childId.empty() || parentId == childId)
	{
		return false;
	}

	std::unique_lock<std::shared_mutex> lock(m_mutex);
	if (m_records.find(parentId) == m_records.end() || m_records.find(childId) == m_records.end())
	{
		return false;
	}

	// 环检测：parent 可达 child 则拒加边
	std::queue<std::string> queue;
	std::unordered_set<std::string> visited;
	queue.push(childId);
	visited.insert(childId);
	while (!queue.empty())
	{
		const std::string cur = queue.front();
		queue.pop();
		if (cur == parentId)
		{
			return false;
		}
		const auto nextIt = m_childrenByParent.find(cur);
		if (nextIt == m_childrenByParent.end())
		{
			continue;
		}
		for (const std::string& next : nextIt->second)
		{
			if (visited.insert(next).second)
			{
				queue.push(next);
			}
		}
	}

	m_childrenByParent[parentId].insert(childId);
	m_parentsByChild[childId].insert(parentId);
	m_subtreeCache.clear();
	notifyHierarchyObserversLocked(
		BackendHierarchyChangeEvent{BackendHierarchyChangeKind::EdgeAttached, parentId, childId});
	return true;
}

bool BackendDataManager::setParent(const std::string& childId, const std::string& parentId)
{
	if (childId.empty())
	{
		return false;
	}
	if (parentId.empty())
	{
		return detachAllParents(childId);
	}
	if (parentId == childId)
	{
		return false;
	}
	std::unique_lock<std::shared_mutex> lock(m_mutex);
	if (m_records.find(parentId) == m_records.end() || m_records.find(childId) == m_records.end())
	{
		return false;
	}
	// 仅校验新边成环
	std::queue<std::string> queue;
	std::unordered_set<std::string> visited;
	queue.push(childId);
	visited.insert(childId);
	while (!queue.empty())
	{
		const std::string cur = queue.front();
		queue.pop();
		if (cur == parentId)
		{
			return false;
		}
		const auto nextIt = m_childrenByParent.find(cur);
		if (nextIt == m_childrenByParent.end())
		{
			continue;
		}
		for (const std::string& next : nextIt->second)
		{
			if (visited.insert(next).second)
			{
				queue.push(next);
			}
		}
	}

	// 先 detach 旧父
	auto parentSetIt = m_parentsByChild.find(childId);
	if (parentSetIt != m_parentsByChild.end())
	{
		const std::vector<std::string> oldParents(parentSetIt->second.begin(), parentSetIt->second.end());
		for (const std::string& oldParent : oldParents)
		{
			auto childSetIt = m_childrenByParent.find(oldParent);
			if (childSetIt != m_childrenByParent.end())
			{
				childSetIt->second.erase(childId);
				if (childSetIt->second.empty())
				{
					m_childrenByParent.erase(childSetIt);
				}
			}
			notifyHierarchyObserversLocked(
				BackendHierarchyChangeEvent{BackendHierarchyChangeKind::EdgeDetached, oldParent, childId});
		}
		m_parentsByChild.erase(parentSetIt);
	}

	m_childrenByParent[parentId].insert(childId);
	m_parentsByChild[childId].insert(parentId);
	m_subtreeCache.clear();
	notifyHierarchyObserversLocked(
		BackendHierarchyChangeEvent{BackendHierarchyChangeKind::EdgeAttached, parentId, childId});
	return true;
}

bool BackendDataManager::detachChild(const std::string& parentId, const std::string& childId)
{
	if (parentId.empty() || childId.empty())
	{
		return false;
	}

	std::unique_lock<std::shared_mutex> lock(m_mutex);
	auto childSetIt = m_childrenByParent.find(parentId);
	if (childSetIt == m_childrenByParent.end())
	{
		return false;
	}
	const bool erased = childSetIt->second.erase(childId) > 0;
	if (!erased)
	{
		return false;
	}
	if (childSetIt->second.empty())
	{
		m_childrenByParent.erase(childSetIt);
	}
	auto parentSetIt = m_parentsByChild.find(childId);
	if (parentSetIt != m_parentsByChild.end())
	{
		parentSetIt->second.erase(parentId);
		if (parentSetIt->second.empty())
		{
			m_parentsByChild.erase(parentSetIt);
		}
	}
	m_subtreeCache.clear();
	notifyHierarchyObserversLocked(
		BackendHierarchyChangeEvent{BackendHierarchyChangeKind::EdgeDetached, parentId, childId});
	return true;
}

bool BackendDataManager::detachAllParents(const std::string& childId)
{
	if (childId.empty())
	{
		return false;
	}
	std::unique_lock<std::shared_mutex> lock(m_mutex);
	auto parentSetIt = m_parentsByChild.find(childId);
	if (parentSetIt == m_parentsByChild.end())
	{
		return false;
	}
	const std::vector<std::string> parents(parentSetIt->second.begin(), parentSetIt->second.end());
	for (const std::string& parentId : parents)
	{
		auto childSetIt = m_childrenByParent.find(parentId);
		if (childSetIt != m_childrenByParent.end())
		{
			childSetIt->second.erase(childId);
			if (childSetIt->second.empty())
			{
				m_childrenByParent.erase(childSetIt);
			}
		}
	}
	m_parentsByChild.erase(parentSetIt);
	m_subtreeCache.clear();
	for (const std::string& parentId : parents)
	{
		notifyHierarchyObserversLocked(
			BackendHierarchyChangeEvent{BackendHierarchyChangeKind::EdgeDetached, parentId, childId});
	}
	return true;
}

std::vector<std::string> BackendDataManager::parentsOf(const std::string& id) const
{
	std::shared_lock<std::shared_mutex> lock(m_mutex);
	const auto it = m_parentsByChild.find(id);
	if (it == m_parentsByChild.end())
	{
		return {};
	}
	return sortedKeys(it->second);
}

std::vector<std::string> BackendDataManager::childrenOf(const std::string& id) const
{
	std::shared_lock<std::shared_mutex> lock(m_mutex);
	const auto it = m_childrenByParent.find(id);
	if (it == m_childrenByParent.end())
	{
		return {};
	}
	return sortedKeys(it->second);
}

std::vector<std::string> BackendDataManager::ancestorsOf(const std::string& id) const
{
	if (id.empty())
	{
		return {};
	}
	std::shared_lock<std::shared_mutex> lock(m_mutex);
	std::vector<std::string> out;
	std::queue<std::string> queue;
	std::unordered_set<std::string> visited;
	queue.push(id);
	visited.insert(id);
	while (!queue.empty())
	{
		const std::string cur = queue.front();
		queue.pop();
		const auto it = m_parentsByChild.find(cur);
		if (it == m_parentsByChild.end())
		{
			continue;
		}
		for (const std::string& parentId : it->second)
		{
			if (visited.insert(parentId).second)
			{
				out.push_back(parentId);
				queue.push(parentId);
			}
		}
	}
	std::sort(out.begin(), out.end());
	return out;
}

std::vector<std::string> BackendDataManager::descendantsOf(const std::string& id) const
{
	if (id.empty())
	{
		return {};
	}
	std::shared_lock<std::shared_mutex> lock(m_mutex);
	std::vector<std::string> out;
	std::queue<std::string> queue;
	std::unordered_set<std::string> visited;
	queue.push(id);
	visited.insert(id);
	while (!queue.empty())
	{
		const std::string cur = queue.front();
		queue.pop();
		const auto it = m_childrenByParent.find(cur);
		if (it == m_childrenByParent.end())
		{
			continue;
		}
		for (const std::string& childId : it->second)
		{
			if (visited.insert(childId).second)
			{
				out.push_back(childId);
				queue.push(childId);
			}
		}
	}
	std::sort(out.begin(), out.end());
	return out;
}

const std::vector<std::string>& BackendDataManager::subtreeIds(const std::string& rootId) const
{
	if (rootId.empty())
	{
		m_emptySubtree.clear();
		return m_emptySubtree;
	}
	{
		std::shared_lock<std::shared_mutex> readLock(m_mutex);
		if (m_records.find(rootId) == m_records.end())
		{
			m_emptySubtree.clear();
			return m_emptySubtree;
		}
		const auto cached = m_subtreeCache.find(rootId);
		if (cached != m_subtreeCache.end())
		{
			return cached->second;
		}
	}

	std::unique_lock<std::shared_mutex> writeLock(m_mutex);
	if (m_records.find(rootId) == m_records.end())
	{
		m_emptySubtree.clear();
		return m_emptySubtree;
	}
	const auto cached = m_subtreeCache.find(rootId);
	if (cached != m_subtreeCache.end())
	{
		return cached->second;
	}
	std::vector<std::string> out;
	std::queue<std::string> queue;
	std::unordered_set<std::string> visited;
	queue.push(rootId);
	visited.insert(rootId);
	while (!queue.empty())
	{
		const std::string cur = queue.front();
		queue.pop();
		out.push_back(cur);
		const auto it = m_childrenByParent.find(cur);
		if (it == m_childrenByParent.end())
		{
			continue;
		}
		for (const std::string& childId : it->second)
		{
			if (visited.insert(childId).second)
			{
				queue.push(childId);
			}
		}
	}
	const auto ins = m_subtreeCache.emplace(rootId, std::move(out));
	return ins.first->second;
}

std::vector<std::string> BackendDataManager::rootIds() const
{
	std::shared_lock<std::shared_mutex> lock(m_mutex);
	std::vector<std::string> roots;
	for (const auto& rec : m_records)
	{
		const auto parentIt = m_parentsByChild.find(rec.first);
		if (parentIt == m_parentsByChild.end() || parentIt->second.empty())
		{
			roots.push_back(rec.first);
		}
	}
	std::sort(roots.begin(), roots.end());
	return roots;
}

std::vector<std::string> BackendDataManager::topoOrder() const
{
	std::shared_lock<std::shared_mutex> lock(m_mutex);
	std::unordered_map<std::string, int> indegree;
	indegree.reserve(m_records.size());
	for (const auto& rec : m_records)
	{
		indegree[rec.first] = 0;
	}
	for (const auto& item : m_childrenByParent)
	{
		for (const std::string& childId : item.second)
		{
			auto indegreeIt = indegree.find(childId);
			if (indegreeIt != indegree.end())
			{
				++indegreeIt->second;
			}
		}
	}

	std::vector<std::string> ready;
	ready.reserve(indegree.size());
	for (const auto& item : indegree)
	{
		if (item.second == 0)
		{
			ready.push_back(item.first);
		}
	}
	std::sort(ready.begin(), ready.end());

	std::vector<std::string> order;
	order.reserve(m_records.size());
	while (!ready.empty())
	{
		const std::string cur = ready.front();
		ready.erase(ready.begin());
		order.push_back(cur);
		const auto childIt = m_childrenByParent.find(cur);
		if (childIt == m_childrenByParent.end())
		{
			continue;
		}
		std::vector<std::string> children = sortedKeys(childIt->second);
		for (const std::string& childId : children)
		{
			auto indegreeIt = indegree.find(childId);
			if (indegreeIt == indegree.end())
			{
				continue;
			}
			--indegreeIt->second;
			if (indegreeIt->second == 0)
			{
				ready.push_back(childId);
			}
		}
		std::sort(ready.begin(), ready.end());
	}

	// 图无效（成环）时确定性追加剩余节点
	if (order.size() != m_records.size())
	{
		std::vector<std::string> remaining;
		remaining.reserve(m_records.size() - order.size());
		for (const auto& rec : m_records)
		{
			if (std::find(order.begin(), order.end(), rec.first) == order.end())
			{
				remaining.push_back(rec.first);
			}
		}
		std::sort(remaining.begin(), remaining.end());
		order.insert(order.end(), remaining.begin(), remaining.end());
	}
	return order;
}

std::vector<std::pair<std::string, std::string>> BackendDataManager::listEdges() const
{
	std::shared_lock<std::shared_mutex> lock(m_mutex);
	std::vector<std::pair<std::string, std::string>> edges;
	for (const auto& item : m_childrenByParent)
	{
		for (const std::string& childId : item.second)
		{
			edges.emplace_back(item.first, childId);
		}
	}
	std::sort(edges.begin(), edges.end());
	return edges;
}

bool BackendDataManager::wouldCreateCycle(const std::string& parentId, const std::string& childId) const
{
	if (parentId.empty() || childId.empty() || parentId == childId)
	{
		return true;
	}

	std::shared_lock<std::shared_mutex> lock(m_mutex);
	if (m_records.find(parentId) == m_records.end() || m_records.find(childId) == m_records.end())
	{
		return true;
	}
	std::queue<std::string> queue;
	std::unordered_set<std::string> visited;
	queue.push(childId);
	visited.insert(childId);
	while (!queue.empty())
	{
		const std::string cur = queue.front();
		queue.pop();
		if (cur == parentId)
		{
			return true;
		}
		const auto nextIt = m_childrenByParent.find(cur);
		if (nextIt == m_childrenByParent.end())
		{
			continue;
		}
		for (const std::string& next : nextIt->second)
		{
			if (visited.insert(next).second)
			{
				queue.push(next);
			}
		}
	}
	return false;
}

bool BackendDataManager::validateGraph(std::string* errMsg) const
{
	std::shared_lock<std::shared_mutex> lock(m_mutex);
	for (const auto& item : m_childrenByParent)
	{
		if (m_records.find(item.first) == m_records.end())
		{
			if (errMsg)
			{
				*errMsg = "Graph has dangling parent id: " + item.first;
			}
			return false;
		}
		for (const std::string& childId : item.second)
		{
			if (m_records.find(childId) == m_records.end())
			{
				if (errMsg)
				{
					*errMsg = "Graph has dangling child id: " + childId;
				}
				return false;
			}
		}
	}

	std::unordered_map<std::string, int> indegree;
	for (const auto& rec : m_records)
	{
		indegree[rec.first] = 0;
	}
	for (const auto& item : m_childrenByParent)
	{
		for (const std::string& childId : item.second)
		{
			++indegree[childId];
		}
	}

	std::queue<std::string> queue;
	for (const auto& item : indegree)
	{
		if (item.second == 0)
		{
			queue.push(item.first);
		}
	}

	std::size_t visitedCount = 0;
	while (!queue.empty())
	{
		const std::string cur = queue.front();
		queue.pop();
		++visitedCount;
		const auto it = m_childrenByParent.find(cur);
		if (it == m_childrenByParent.end())
		{
			continue;
		}
		for (const std::string& childId : it->second)
		{
			const auto indegreeIt = indegree.find(childId);
			if (indegreeIt == indegree.end())
			{
				continue;
			}
			--indegreeIt->second;
			if (indegreeIt->second == 0)
			{
				queue.push(childId);
			}
		}
	}

	if (visitedCount != m_records.size())
	{
		if (errMsg)
		{
			*errMsg = "Graph contains cycle.";
		}
		return false;
	}
	return true;
}

std::vector<BackendSnapshot> BackendDataManager::takeSnapshot() const
{
	std::shared_lock<std::shared_mutex> lock(m_mutex);
	std::vector<BackendSnapshot> snapshots;
	snapshots.reserve(m_records.size());
	for (const auto& item : m_records)
	{
		const std::shared_ptr<BackendDataBase>& data = item.second;
		if (!data)
		{
			continue;
		}
		BackendSnapshot snap;
		snap.id = data->id();
		snap.name = data->name();
		snap.className = data->className();
		snap.pose = data->pose();
		snap.rotation = data->rotation();
		snap.color = data->color();
		snap.worldMatrix = data->worldMatrix(this);
		if (const auto pit = m_parentsByChild.find(item.first); pit != m_parentsByChild.end())
		{
			snap.parents.assign(pit->second.begin(), pit->second.end());
			std::sort(snap.parents.begin(), snap.parents.end());
		}
		if (const auto cit = m_childrenByParent.find(item.first); cit != m_childrenByParent.end())
		{
			snap.children.assign(cit->second.begin(), cit->second.end());
			std::sort(snap.children.begin(), snap.children.end());
		}
		snapshots.push_back(std::move(snap));
	}
	std::sort(snapshots.begin(), snapshots.end(),
			  [](const BackendSnapshot& lhs, const BackendSnapshot& rhs) { return lhs.id < rhs.id; });
	return snapshots;
}

BackendBaselineMetrics BackendDataManager::collectBaselineMetrics(const std::string& sampleRootId) const
{
	const auto t0 = std::chrono::steady_clock::now();
	const std::vector<std::shared_ptr<BackendDataBase>> all = listData();
	const auto t1 = std::chrono::steady_clock::now();
	const std::vector<std::string> descendants =
		descendantsOf(sampleRootId.empty() ? (all.empty() ? std::string() : all.front()->id()) : sampleRootId);
	const auto t2 = std::chrono::steady_clock::now();
	const std::vector<std::shared_ptr<BackendDataBase>> follow = findByComponent("FollowAttachment");
	const auto t3 = std::chrono::steady_clock::now();
	const std::vector<BackendSnapshot> snapshots = takeSnapshot();
	const auto t4 = std::chrono::steady_clock::now();

	BackendBaselineMetrics metrics;
	metrics.objectCount = all.size();
	metrics.edgeCount = listEdges().size();
	metrics.listDataMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
	metrics.descendantsMs = std::chrono::duration<double, std::milli>(t2 - t1).count();
	metrics.followLookupMs = std::chrono::duration<double, std::milli>(t3 - t2).count();
	metrics.snapshotMs = std::chrono::duration<double, std::milli>(t4 - t3).count();
	(void)descendants;
	(void)follow;
	(void)snapshots;
	return metrics;
}

void BackendDataManager::clear()
{
	std::unique_lock<std::shared_mutex> lock(m_mutex);
	m_records.clear();
	m_childrenByParent.clear();
	m_parentsByChild.clear();
	m_subtreeCache.clear();
	notifyHierarchyObserversLocked(BackendHierarchyChangeEvent{BackendHierarchyChangeKind::AllCleared, {}, {}});
}

void BackendDataManager::addHierarchyObserver(void* key, BackendHierarchyObserver observer)
{
	std::unique_lock<std::shared_mutex> lock(m_mutex);
	m_hierarchyObservers.erase(std::remove_if(m_hierarchyObservers.begin(), m_hierarchyObservers.end(),
											  [key](const std::pair<void*, BackendHierarchyObserver>& p)
											  { return p.first == key; }),
							   m_hierarchyObservers.end());
	m_hierarchyObservers.emplace_back(key, std::move(observer));
}

void BackendDataManager::removeHierarchyObserver(void* key)
{
	std::unique_lock<std::shared_mutex> lock(m_mutex);
	m_hierarchyObservers.erase(std::remove_if(m_hierarchyObservers.begin(), m_hierarchyObservers.end(),
											  [key](const std::pair<void*, BackendHierarchyObserver>& p)
											  { return p.first == key; }),
							   m_hierarchyObservers.end());
}

void BackendDataManager::notifyHierarchyObserversLocked(const BackendHierarchyChangeEvent& event)
{
	for (const auto& kv : m_hierarchyObservers)
	{
		if (kv.second)
		{
			kv.second(event);
		}
	}
}
