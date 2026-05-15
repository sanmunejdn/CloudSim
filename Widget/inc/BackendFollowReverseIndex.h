#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "widget_global.h"

class BackendDataManager;

/// Reverse index: follow **target** backend id -> follower backend ids (for queries; rebuilt when invalidated).
class WIDGET_EXPORT BackendFollowReverseIndex
{
public:
	void invalidate();
	bool isDirty() const { return m_dirty; }

	/// Rebuilds from \a mgr when dirty, then returns followers of \a targetBackendId (may be empty).
	std::vector<std::string> followersOf(const BackendDataManager& mgr, const std::string& targetBackendId) const;

private:
	void rebuild(const BackendDataManager& mgr) const;

	mutable bool m_dirty = true;
	mutable std::unordered_map<std::string, std::vector<std::string>> m_targetToFollowersSorted;
};
