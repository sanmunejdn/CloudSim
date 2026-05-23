#pragma once

#include "cloudsim_core_global.h"

#include <functional>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>
#include <vector>

namespace cloudsim::core {

/// UI-thread domain event bus (typed publish/subscribe).
class CLOUDSIM_CORE_EXPORT EventHub
{
public:
	EventHub() = default;
	~EventHub();

	EventHub(const EventHub&) = delete;
	EventHub& operator=(const EventHub&) = delete;

	template<typename Event>
	void subscribe(std::function<void(const Event&)> handler)
	{
		const std::type_index key(typeid(Event));
		auto wrapper = [handler](const void* raw) {
			handler(*static_cast<const Event*>(raw));
		};
		m_handlers[key].push_back(std::move(wrapper));
	}

	template<typename Event>
	void publish(const Event& event)
	{
		const std::type_index key(typeid(Event));
		const auto it = m_handlers.find(key);
		if (it == m_handlers.end())
			return;
		for (const auto& fn : it->second)
			fn(&event);
	}

	void clear();

private:
	using HandlerFn = std::function<void(const void*)>;
	std::unordered_map<std::type_index, std::vector<HandlerFn>> m_handlers;
};

} // namespace cloudsim::core
