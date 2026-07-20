#ifndef WIDGET_BACKENDFOLLOWREVERSEINDEX_H
#define WIDGET_BACKENDFOLLOWREVERSEINDEX_H

/// @file BackendFollowReverseIndex.h
/// @brief 跟随反向索引：被跟随 backend id → 跟随者 id 列表；脏时按 BackendDataManager 重建

#include "widget_global.h"

#include <string>
#include <unordered_map>
#include <vector>

class BackendDataManager;

/// 跟随反向索引：被跟随 backend id → 跟随者 id 列表；脏时按 BackendDataManager 重建
class WIDGET_EXPORT BackendFollowReverseIndex
{
public:
	void invalidate();
	bool isDirty() const { return m_dirty; }

	/// 脏则重建索引，返回 targetBackendId 的跟随者列表（可为空）
	std::vector<std::string> followersOf(const BackendDataManager& mgr, const std::string& targetBackendId) const;

private:
	void rebuild(const BackendDataManager& mgr) const;

	mutable bool m_dirty = true;
	mutable std::unordered_map<std::string, std::vector<std::string>> m_targetToFollowersSorted;
};

#endif // WIDGET_BACKENDFOLLOWREVERSEINDEX_H
