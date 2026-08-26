/// @file BackendDataManager.cpp
/// @brief BackendData 管理

#include "BackendDataManager.h"

#include "BackendRegistryBuiltins.h"
#include "RunLogger.h"

#include <algorithm>
#include <chrono>
#include <mutex>
#include <queue>
#include <set>
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

	std::vector<BackendHierarchyChangeEvent> pendingEvents;
	std::vector<std::pair<void*, BackendHierarchyObserver>> observerSnapshot;
	bool registered = false;
	{
		std::unique_lock<std::shared_mutex> lock(m_mutex);
		const auto result = m_records.emplace(data->id(), data);
		if (result.second)
		{
			data->markIdRegistered(true);
			m_subtreeCache.clear();
			pendingEvents.push_back(
				BackendHierarchyChangeEvent{BackendHierarchyChangeKind::DataRegistered, {}, data->id()});
			observerSnapshot = m_hierarchyObservers;
			registered = true;
		}
	}
	dispatchHierarchyEvents(observerSnapshot, pendingEvents);
	return registered;
}

bool BackendDataManager::unregisterData(const std::string& id)
{
	if (id.empty())
	{
		return false;
	}

	std::vector<BackendHierarchyChangeEvent> pendingEvents;
	std::vector<std::pair<void*, BackendHierarchyObserver>> observerSnapshot;
	std::vector<std::shared_ptr<BackendDataBase>> recordsSnapshot;
	bool removed = false;
	{
		std::unique_lock<std::shared_mutex> lock(m_mutex);
		const auto recIt = m_records.find(id);
		if (recIt == m_records.end())
		{
			return false;
		}
		if (recIt->second)
		{
			recIt->second->markIdRegistered(false);
		}
		m_records.erase(recIt);
		removed = true;
		m_primaryParentByChild.erase(id);

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
						m_primaryParentByChild.erase(childId);
					}
				else
				{
					const auto primIt = m_primaryParentByChild.find(childId);
					if (primIt != m_primaryParentByChild.end() && primIt->second == id)
					{
						const std::vector<std::string> remain = sortedKeys(parentSetIt->second);
						m_primaryParentByChild[childId] = remain.front();
						// P3-2: 与 primaryParentOf 回退路径的 warn 对齐，注销时不再静默
						RunLogger::warn("[BackendDataManager] unregisterData: primary parent of \"" + childId +
										"\" removed, re-pointed to \"" + remain.front() + "\".");
					}
				}
				}
			}
			m_childrenByParent.erase(childrenIt);
		}

		pendingEvents.push_back(
			BackendHierarchyChangeEvent{BackendHierarchyChangeKind::DataUnregistered, {}, id});
		m_subtreeCache.clear();
		observerSnapshot = m_hierarchyObservers;
		for (const auto& kv : m_records)
		{
			recordsSnapshot.push_back(kv.second);
		}
	}
	dispatchHierarchyEvents(observerSnapshot, pendingEvents);
	// 悬挂引用检测放锁外：组件 getter 持对象锁，持 manager 写锁调用会与 appendPropertyRows 锁序颠倒
	for (const auto& obj : recordsSnapshot)
	{
		if (!obj)
		{
			continue;
		}
		std::vector<std::string> refs;
		obj->collectReferencedBackendIds(refs);
		for (const std::string& ref : refs)
		{
			if (ref == id)
			{
				RunLogger::warn("[BackendDataManager] unregistered \"" + id + "\" still referenced by \"" +
								obj->id() + "\" (dangling backend id).");
				break;
			}
		}
	}
	return removed;
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
	if (m_primaryParentByChild.find(childId) == m_primaryParentByChild.end())
	{
		m_primaryParentByChild[childId] = parentId;
	}
	m_subtreeCache.clear();
	std::vector<BackendHierarchyChangeEvent> pendingEvents{
		BackendHierarchyChangeEvent{BackendHierarchyChangeKind::EdgeAttached, parentId, childId}};
	const std::vector<std::pair<void*, BackendHierarchyObserver>> observerSnapshot = m_hierarchyObservers;
	lock.unlock();
	dispatchHierarchyEvents(observerSnapshot, pendingEvents);
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
	std::vector<BackendHierarchyChangeEvent> pendingEvents;
	std::vector<std::pair<void*, BackendHierarchyObserver>> observerSnapshot;
	{
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
			pendingEvents.push_back(
				BackendHierarchyChangeEvent{BackendHierarchyChangeKind::EdgeDetached, oldParent, childId});
		}
		m_parentsByChild.erase(parentSetIt);
	}

	m_childrenByParent[parentId].insert(childId);
	m_parentsByChild[childId].insert(parentId);
	m_primaryParentByChild[childId] = parentId;
	m_subtreeCache.clear();
	pendingEvents.push_back(
		BackendHierarchyChangeEvent{BackendHierarchyChangeKind::EdgeAttached, parentId, childId});
	observerSnapshot = m_hierarchyObservers;
	}
	dispatchHierarchyEvents(observerSnapshot, pendingEvents);
	return true;
}

bool BackendDataManager::detachChild(const std::string& parentId, const std::string& childId)
{
	if (parentId.empty() || childId.empty())
	{
		return false;
	}

	std::vector<BackendHierarchyChangeEvent> pendingEvents;
	std::vector<std::pair<void*, BackendHierarchyObserver>> observerSnapshot;
	{
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
				m_primaryParentByChild.erase(childId);
			}
			else
			{
				const auto primIt = m_primaryParentByChild.find(childId);
				if (primIt != m_primaryParentByChild.end() && primIt->second == parentId)
				{
					const std::vector<std::string> remain = sortedKeys(parentSetIt->second);
					m_primaryParentByChild[childId] = remain.front();
					// B2: 与 unregisterData 路径对齐，detachChild 字典序重指也告警
					RunLogger::warn("[BackendDataManager] detachChild: primary parent of \"" + childId +
									"\" detached, re-pointed to \"" + remain.front() + "\".");
				}
			}
		}
		m_subtreeCache.clear();
		pendingEvents.push_back(
			BackendHierarchyChangeEvent{BackendHierarchyChangeKind::EdgeDetached, parentId, childId});
		observerSnapshot = m_hierarchyObservers;
	}
	dispatchHierarchyEvents(observerSnapshot, pendingEvents);
	return true;
}

bool BackendDataManager::detachAllParents(const std::string& childId)
{
	if (childId.empty())
	{
		return false;
	}
	std::vector<BackendHierarchyChangeEvent> pendingEvents;
	std::vector<std::pair<void*, BackendHierarchyObserver>> observerSnapshot;
	{
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
			pendingEvents.push_back(
				BackendHierarchyChangeEvent{BackendHierarchyChangeKind::EdgeDetached, parentId, childId});
		}
		m_parentsByChild.erase(parentSetIt);
		m_primaryParentByChild.erase(childId);
		m_subtreeCache.clear();
		observerSnapshot = m_hierarchyObservers;
	}
	dispatchHierarchyEvents(observerSnapshot, pendingEvents);
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

std::string BackendDataManager::primaryParentOf(const std::string& childId) const
{
	std::shared_lock<std::shared_mutex> lock(m_mutex);
	const auto parentsIt = m_parentsByChild.find(childId);
	if (parentsIt == m_parentsByChild.end() || parentsIt->second.empty())
	{
		return {};
	}
	const auto primIt = m_primaryParentByChild.find(childId);
	if (primIt != m_primaryParentByChild.end() && parentsIt->second.count(primIt->second) != 0)
	{
		return primIt->second;
	}
	const std::vector<std::string> sorted = sortedKeys(parentsIt->second);
	RunLogger::warn("[BackendDataManager] primaryParentOf: no explicit primary for \"" + childId +
					"\", fallback to lexicographic first.");
	return sorted.front();
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

std::vector<std::string> BackendDataManager::subtreeIds(const std::string& rootId) const
{
	if (rootId.empty())
	{
		return {};
	}
	{
		std::shared_lock<std::shared_mutex> readLock(m_mutex);
		if (m_records.find(rootId) == m_records.end())
		{
			return {};
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
		return {};
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
		// 子集为 unordered_set，先排序再入队，保证相同图结构下遍历序可复现
		std::vector<std::string> children(it->second.begin(), it->second.end());
		std::sort(children.begin(), children.end());
		for (const std::string& childId : children)
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

	std::set<std::string> ready;
	for (const auto& item : indegree)
	{
		if (item.second == 0)
		{
			ready.insert(item.first);
		}
	}

	std::vector<std::string> order;
	order.reserve(m_records.size());
	while (!ready.empty())
	{
		const std::string cur = *ready.begin();
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
				ready.insert(childId);
			}
		}
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
		snap.worldMatrix = data->worldMatrix();
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
	std::vector<BackendHierarchyChangeEvent> pendingEvents;
	std::vector<std::pair<void*, BackendHierarchyObserver>> observerSnapshot;
	{
		std::unique_lock<std::shared_mutex> lock(m_mutex);
		for (const auto& kv : m_records)
		{
			if (kv.second)
			{
				kv.second->markIdRegistered(false);
			}
		}
		m_records.clear();
		m_childrenByParent.clear();
		m_parentsByChild.clear();
		m_primaryParentByChild.clear();
		m_subtreeCache.clear();
		pendingEvents.push_back(BackendHierarchyChangeEvent{BackendHierarchyChangeKind::AllCleared, {}, {}});
		observerSnapshot = m_hierarchyObservers;
	}
	dispatchHierarchyEvents(observerSnapshot, pendingEvents);
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

void BackendDataManager::dispatchHierarchyEvents(
	const std::vector<std::pair<void*, BackendHierarchyObserver>>& observers,
	const std::vector<BackendHierarchyChangeEvent>& events)
{
	for (const BackendHierarchyChangeEvent& event : events)
	{
		for (const auto& kv : observers)
		{
			if (kv.second)
			{
				kv.second(event);
			}
		}
	}
}
