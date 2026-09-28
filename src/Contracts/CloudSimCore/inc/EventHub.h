#ifndef CLOUDSIMCORE_EVENTHUB_H
#define CLOUDSIMCORE_EVENTHUB_H

/// @file EventHub.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief UI 线程事件总线

#include "cloudsim_core_global.h"

#include <exception>
#include <functional>
#include <iostream>
#include <list>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>

namespace cloudsim::core
{
/// UI 线程事件总线
class CLOUDSIM_CORE_EXPORT EventHub
{
	using HandlerFn = std::function<void(const void*)>;

public:
	/// RAII 订阅句柄：析构自动退订，避免订阅方先销毁后 handler 悬挂
	class Subscription
	{
	public:
		Subscription() = default;
		~Subscription() { reset(); }

		Subscription(const Subscription&) = delete;
		Subscription& operator=(const Subscription&) = delete;

		Subscription(Subscription&& other) noexcept
			: m_hub(other.m_hub), m_key(other.m_key), m_it(other.m_it)
		{
			other.m_hub = nullptr;
		}

		Subscription& operator=(Subscription&& other) noexcept
		{
			if (this != &other)
			{
				reset();
				m_hub = other.m_hub;
				m_key = other.m_key;
				m_it = other.m_it;
				other.m_hub = nullptr;
			}
			return *this;
		}

		/// 主动退订；幂等
		void reset()
		{
			if (m_hub)
			{
				m_hub->unsubscribe(m_key, m_it);
				m_hub = nullptr;
			}
		}

	private:
		friend class EventHub;

		Subscription(EventHub* hub, std::type_index key, std::list<HandlerFn>::iterator it)
			: m_hub(hub), m_key(key), m_it(it)
		{
		}

		EventHub* m_hub = nullptr;
		std::type_index m_key{typeid(void)};
		std::list<HandlerFn>::iterator m_it;
	};

	EventHub() = default;
	~EventHub();

	EventHub(const EventHub&) = delete;
	EventHub& operator=(const EventHub&) = delete;

	template <typename Event>
	Subscription subscribe(std::function<void(const Event&)> handler)
	{
		const std::type_index key(typeid(Event));
		auto wrapper = [handler](const void* raw) { handler(*static_cast<const Event*>(raw)); };
		auto& handlers = m_handlers[key];
		handlers.push_back(std::move(wrapper));
		auto it = handlers.end();
		--it;
		return Subscription(this, key, it);
	}

	template <typename Event>
	void publish(const Event& event)
	{
		const std::type_index key(typeid(Event));
		const auto it = m_handlers.find(key);
		if (it == m_handlers.end())
			return;
		for (const auto& fn : it->second)
		{
			// 单个 handler 异常不应阻断其余 handler；CloudSimCore 不依赖 RunLogger，只能落 stderr
			try
			{
				fn(&event);
			}
			catch (const std::exception& e)
			{
				std::cerr << "[EventHub] handler exception: " << e.what() << std::endl;
			}
			catch (...)
			{
				std::cerr << "[EventHub] handler unknown exception" << std::endl;
			}
		}
	}

	void clear();

private:
	void unsubscribe(const std::type_index& key, std::list<HandlerFn>::iterator it);

	std::unordered_map<std::type_index, std::list<HandlerFn>> m_handlers;
};

} // namespace cloudsim::core

#endif // CLOUDSIMCORE_EVENTHUB_H
