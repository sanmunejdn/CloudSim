/// @file BackendFollowReverseIndex.cpp
/// @brief BackendFollowReverseIndex 实现

#include "BackendFollowReverseIndex.h"

#include "IDataService.h"

#include <algorithm>

void BackendFollowReverseIndex::invalidate()
{
	m_dirty = true;
}

void BackendFollowReverseIndex::rebuild(const cloudsim::core::IDataService& data) const
{
	m_targetToFollowersSorted.clear();
	for (const cloudsim::core::ObjectId& id : data.listAll())
	{
		if (id.isEmpty())
		{
			continue;
		}
		const cloudsim::core::ObjectId tid = data.followTargetId(id);
		if (tid.isEmpty())
		{
			continue;
		}
		m_targetToFollowersSorted[tid.toStdString()].push_back(id.toStdString());
	}
	for (auto& kv : m_targetToFollowersSorted)
	{
		std::sort(kv.second.begin(), kv.second.end());
		kv.second.erase(std::unique(kv.second.begin(), kv.second.end()), kv.second.end());
	}
	m_dirty = false;
}

std::vector<std::string> BackendFollowReverseIndex::followersOf(const cloudsim::core::IDataService& data,
																const std::string& targetBackendId) const
{
	if (m_dirty)
	{
		rebuild(data);
	}
	const auto it = m_targetToFollowersSorted.find(targetBackendId);
	if (it == m_targetToFollowersSorted.end())
	{
		return {};
	}
	return it->second;
}
