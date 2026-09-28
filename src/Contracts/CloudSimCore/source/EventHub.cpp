/// @file EventHub.cpp
/// @brief EventHub 实现

#include "EventHub.h"

namespace cloudsim::core
{
EventHub::~EventHub() = default;

void EventHub::unsubscribe(const std::type_index& key, std::list<HandlerFn>::iterator it)
{
	const auto mapIt = m_handlers.find(key);
	if (mapIt == m_handlers.end())
		return;
	mapIt->second.erase(it);
	// 空表及时摘除，避免 publish 空转
	if (mapIt->second.empty())
		m_handlers.erase(mapIt);
}

void EventHub::clear()
{
	m_handlers.clear();
}

} // namespace cloudsim::core
