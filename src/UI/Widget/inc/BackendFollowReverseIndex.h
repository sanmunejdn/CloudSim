#ifndef WIDGET_BACKENDFOLLOWREVERSEINDEX_H
#define WIDGET_BACKENDFOLLOWREVERSEINDEX_H

/// @file BackendFollowReverseIndex.h
/// @brief 跟随反向索引：被跟随 id → 跟随者列表；脏时经 IDataService 重建

#include "widget_global.h"

#include <string>
#include <unordered_map>
#include <vector>

namespace cloudsim::core
{
class IDataService;
}

/// 跟随反向索引：被跟随 id → 跟随者列表；脏时经 IDataService 重建
class WIDGET_EXPORT BackendFollowReverseIndex
{
public:
	void invalidate();
	bool isDirty() const { return m_dirty; }

	/// 脏则重建索引，返回 targetBackendId 的跟随者列表（可为空）
	std::vector<std::string> followersOf(const cloudsim::core::IDataService& data,
										 const std::string& targetBackendId) const;

private:
	void rebuild(const cloudsim::core::IDataService& data) const;

	mutable bool m_dirty = true;
	mutable std::unordered_map<std::string, std::vector<std::string>> m_targetToFollowersSorted;
};

#endif // WIDGET_BACKENDFOLLOWREVERSEINDEX_H
