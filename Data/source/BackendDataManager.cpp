#include "BackendDataManager.h"
#include "BackendDataBase.h"

#include <algorithm>
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
	static BackendDataManager manager;
	return manager;
}

bool BackendDataManager::registerData(const std::shared_ptr<BackendDataBase>& data)
{
	if (!data || data->id().empty())
	{
		return false;
	}

	std::lock_guard<std::mutex> lock(m_mutex);
	const auto result = m_records.emplace(data->id(), data);
	return result.second;
}

bool BackendDataManager::unregisterData(const std::string& id)
{
	if (id.empty())
	{
		return false;
	}

	std::lock_guard<std::mutex> lock(m_mutex);
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

	return true;
}

bool BackendDataManager::contains(const std::string& id) const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_records.find(id) != m_records.end();
}

std::shared_ptr<BackendDataBase> BackendDataManager::getData(const std::string& id) const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	const auto it = m_records.find(id);
	if (it == m_records.end())
	{
		return nullptr;
	}

	return it->second;
}

std::vector<std::shared_ptr<BackendDataBase>> BackendDataManager::listData() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	std::vector<std::shared_ptr<BackendDataBase>> records;
	records.reserve(m_records.size());
	for (const auto& item : m_records)
	{
		records.push_back(item.second);
	}
	std::sort(records.begin(), records.end(),
		[](const std::shared_ptr<BackendDataBase>& lhs, const std::shared_ptr<BackendDataBase>& rhs) {
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
	std::lock_guard<std::mutex> lock(m_mutex);
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
	std::lock_guard<std::mutex> lock(m_mutex);
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

std::vector<std::shared_ptr<BackendDataBase>> BackendDataManager::findByComponent(const std::string& componentType) const
{
	std::lock_guard<std::mutex> lock(m_mutex);
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

	std::lock_guard<std::mutex> lock(m_mutex);
	if (m_records.find(parentId) == m_records.end() || m_records.find(childId) == m_records.end())
	{
		return false;
	}

	// Cycle check: if parent is reachable from child, adding edge parent->child introduces a cycle.
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
	return true;
}

bool BackendDataManager::detachChild(const std::string& parentId, const std::string& childId)
{
	if (parentId.empty() || childId.empty())
	{
		return false;
	}

	std::lock_guard<std::mutex> lock(m_mutex);
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
	return true;
}

bool BackendDataManager::detachAllParents(const std::string& childId)
{
	if (childId.empty())
	{
		return false;
	}
	std::lock_guard<std::mutex> lock(m_mutex);
	auto parentSetIt = m_parentsByChild.find(childId);
	if (parentSetIt == m_parentsByChild.end())
	{
		return false;
	}
	for (const std::string& parentId : parentSetIt->second)
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
	return true;
}

std::vector<std::string> BackendDataManager::parentsOf(const std::string& id) const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	const auto it = m_parentsByChild.find(id);
	if (it == m_parentsByChild.end())
	{
		return {};
	}
	return sortedKeys(it->second);
}

std::vector<std::string> BackendDataManager::childrenOf(const std::string& id) const
{
	std::lock_guard<std::mutex> lock(m_mutex);
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
	std::lock_guard<std::mutex> lock(m_mutex);
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
	std::lock_guard<std::mutex> lock(m_mutex);
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

std::vector<std::string> BackendDataManager::rootIds() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
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
	std::lock_guard<std::mutex> lock(m_mutex);
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

	// Fallback: if graph is invalid (cycle), append remaining nodes deterministically.
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
	std::lock_guard<std::mutex> lock(m_mutex);
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

	std::lock_guard<std::mutex> lock(m_mutex);
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
	std::lock_guard<std::mutex> lock(m_mutex);
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

void BackendDataManager::clear()
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_records.clear();
	m_childrenByParent.clear();
	m_parentsByChild.clear();
}

