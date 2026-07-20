/// @file EventHub.cpp
/// @brief EventHub 实现

#include "EventHub.h"

namespace cloudsim::core
{
EventHub::~EventHub() = default;

void EventHub::clear()
{
	m_handlers.clear();
}

} // namespace cloudsim::core
