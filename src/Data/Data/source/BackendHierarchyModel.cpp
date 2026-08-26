/// @file BackendHierarchyModel.cpp
/// @brief 后端层级模型

#include "BackendHierarchyModel.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"

#include <algorithm>
#include <queue>

BackendHierarchyModel::BackendHierarchyModel(BackendDataManager& manager) : m_manager(manager)
{
	m_manager.addHierarchyObserver(this, [this](const BackendHierarchyChangeEvent& e) { onHierarchyChange(e); });
}

BackendHierarchyModel::~BackendHierarchyModel()
{
	m_manager.removeHierarchyObserver(this);
}

void BackendHierarchyModel::resyncFrom(const BackendDataManager& manager)
{
	m_children.clear();
	m_parents.clear();
	m_nodes.clear();
	m_subtreeCache.clear();

	for (const std::shared_ptr<BackendDataBase>& d : manager.listData())
	{
		if (d && !d->id().empty())
		{
			m_nodes.insert(d->id());
		}
	}
	for (const std::pair<std::string, std::string>& edge : manager.listEdges())
	{
		m_children[edge.first].insert(edge.second);
		m_parents[edge.second].insert(edge.first);
		m_nodes.insert(edge.first);
		m_nodes.insert(edge.second);
	}
}

void BackendHierarchyModel::invalidateSubtreeCachesForSeeds(const std::vector<std::string>& seeds)
{
	std::unordered_set<std::string> ancestors;
	std::queue<std::string> q;
	for (const std::string& s : seeds)
	{
		if (!s.empty())
		{
			q.push(s);
		}
	}
	while (!q.empty())
	{
		const std::string cur = q.front();
		q.pop();
		if (!ancestors.insert(cur).second)
		{
			continue;
		}
		const auto pit = m_parents.find(cur);
		if (pit == m_parents.end())
		{
			continue;
		}
		for (const std::string& p : pit->second)
		{
			q.push(p);
		}
	}
	for (const std::string& a : ancestors)
	{
		m_subtreeCache.erase(a);
	}
}

void BackendHierarchyModel::eraseNodeFromMirror(const std::string& id)
{
	const auto chIt = m_children.find(id);
	if (chIt != m_children.end())
	{
		for (const std::string& c : chIt->second)
		{
			const auto pIt = m_parents.find(c);
			if (pIt != m_parents.end())
			{
				pIt->second.erase(id);
				if (pIt->second.empty())
				{
					m_parents.erase(pIt);
				}
			}
		}
		m_children.erase(chIt);
	}
	const auto paIt = m_parents.find(id);
	if (paIt != m_parents.end())
	{
		for (const std::string& p : paIt->second)
		{
			const auto cIt = m_children.find(p);
			if (cIt != m_children.end())
			{
				cIt->second.erase(id);
				if (cIt->second.empty())
				{
					m_children.erase(cIt);
				}
			}
		}
		m_parents.erase(paIt);
	}
	m_nodes.erase(id);
	m_subtreeCache.erase(id);
}

std::vector<std::string> BackendHierarchyModel::buildSubtreeBfs(const std::string& rootId) const
{
	std::vector<std::string> out;
	if (rootId.empty() || m_nodes.find(rootId) == m_nodes.end())
	{
		return out;
	}
	std::queue<std::string> queue;
	std::unordered_set<std::string> visited;
	queue.push(rootId);
	visited.insert(rootId);
	while (!queue.empty())
	{
		const std::string cur = queue.front();
		queue.pop();
		out.push_back(cur);
		const auto it = m_children.find(cur);
		if (it == m_children.end())
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
	return out;
}

std::vector<std::string> BackendHierarchyModel::subtreeIds(const std::string& rootId) const
{
	if (rootId.empty() || m_nodes.find(rootId) == m_nodes.end())
	{
		return {};
	}
	const auto cached = m_subtreeCache.find(rootId);
	if (cached != m_subtreeCache.end())
	{
		return cached->second;
	}
	std::vector<std::string> built = buildSubtreeBfs(rootId);
	const auto ins = m_subtreeCache.emplace(rootId, std::move(built));
	return ins.first->second;
}

void BackendHierarchyModel::onHierarchyChange(const BackendHierarchyChangeEvent& event)
{
	switch (event.kind)
	{
	case BackendHierarchyChangeKind::AllCleared:
		m_children.clear();
		m_parents.clear();
		m_nodes.clear();
		m_subtreeCache.clear();
		return;
	case BackendHierarchyChangeKind::DataRegistered:
		invalidateSubtreeCachesForSeeds({event.childId});
		if (!event.childId.empty())
		{
			m_nodes.insert(event.childId);
		}
		return;
	case BackendHierarchyChangeKind::DataUnregistered:
		if (!event.childId.empty())
		{
			invalidateSubtreeCachesForSeeds({event.childId});
			eraseNodeFromMirror(event.childId);
		}
		return;
	case BackendHierarchyChangeKind::EdgeAttached:
		invalidateSubtreeCachesForSeeds({event.parentId, event.childId});
		if (!event.parentId.empty() && !event.childId.empty())
		{
			m_children[event.parentId].insert(event.childId);
			m_parents[event.childId].insert(event.parentId);
			m_nodes.insert(event.parentId);
			m_nodes.insert(event.childId);
		}
		return;
	case BackendHierarchyChangeKind::EdgeDetached:
		invalidateSubtreeCachesForSeeds({event.parentId, event.childId});
		if (!event.parentId.empty() && !event.childId.empty())
		{
			const auto cit = m_children.find(event.parentId);
			if (cit != m_children.end())
			{
				cit->second.erase(event.childId);
				if (cit->second.empty())
				{
					m_children.erase(cit);
				}
			}
			const auto pit = m_parents.find(event.childId);
			if (pit != m_parents.end())
			{
				pit->second.erase(event.parentId);
				if (pit->second.empty())
				{
					m_parents.erase(pit);
				}
			}
		}
		return;
	}
}
