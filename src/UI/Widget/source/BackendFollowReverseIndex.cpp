/// @file BackendFollowReverseIndex.cpp
/// @brief BackendFollowReverseIndex 实现

#include "BackendFollowReverseIndex.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "FollowAttachmentComponent.h"

#include <algorithm>

void BackendFollowReverseIndex::invalidate()
{
	m_dirty = true;
}

void BackendFollowReverseIndex::rebuild(const BackendDataManager& mgr) const
{
	m_targetToFollowersSorted.clear();
	for (const std::shared_ptr<BackendDataBase>& d : mgr.listData())
	{
		if (!d)
		{
			continue;
		}
		const auto f = std::dynamic_pointer_cast<FollowAttachmentComponent>(
			d->getComponent(FollowAttachmentComponent::typeKeyStatic()));
		if (!f || !f->enabled())
		{
			continue;
		}
		const std::string& tid = f->targetBackendId();
		if (tid.empty())
		{
			continue;
		}
		m_targetToFollowersSorted[tid].push_back(d->id());
	}
	for (auto& kv : m_targetToFollowersSorted)
	{
		std::sort(kv.second.begin(), kv.second.end());
		kv.second.erase(std::unique(kv.second.begin(), kv.second.end()), kv.second.end());
	}
	m_dirty = false;
}

std::vector<std::string> BackendFollowReverseIndex::followersOf(const BackendDataManager& mgr,
																const std::string& targetBackendId) const
{
	if (m_dirty)
	{
		rebuild(mgr);
	}
	const auto it = m_targetToFollowersSorted.find(targetBackendId);
	if (it == m_targetToFollowersSorted.end())
	{
		return {};
	}
	return it->second;
}
