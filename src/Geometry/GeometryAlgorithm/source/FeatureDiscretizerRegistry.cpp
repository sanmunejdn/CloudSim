/// @file FeatureDiscretizerRegistry.cpp
/// @brief FeatureDiscretizerRegistry 实现

#include "FeatureDiscretizerRegistry.h"

#include <algorithm>
#include <cassert>
#include <functional>
#include <mutex>
#include <vector>

namespace
{
geoalgo::FeatureDiscretizerRegistry* g_processFeatureDiscretizerRegistry = nullptr;
std::mutex g_deferredDiscretizerMutex;
std::vector<std::function<std::unique_ptr<geoalgo::IFeatureDiscretizer>()>> g_deferredDiscretizers;

void flushDeferredDiscretizers(geoalgo::FeatureDiscretizerRegistry& registry)
{
	std::vector<std::function<std::unique_ptr<geoalgo::IFeatureDiscretizer>()>> pending;
	{
		std::lock_guard<std::mutex> lock(g_deferredDiscretizerMutex);
		pending.swap(g_deferredDiscretizers);
	}
	for (const auto& factory : pending)
	{
		registry.registerDiscretizer(factory());
	}
}
} // namespace

namespace geoalgo
{
void deferFeatureDiscretizerRegistration(std::function<std::unique_ptr<IFeatureDiscretizer>()> factory)
{
	std::lock_guard<std::mutex> lock(g_deferredDiscretizerMutex);
	g_deferredDiscretizers.push_back(std::move(factory));
}

void flushDeferredFeatureDiscretizerRegistrations()
{
	if (g_processFeatureDiscretizerRegistry != nullptr)
	{
		flushDeferredDiscretizers(*g_processFeatureDiscretizerRegistry);
	}
}

void FeatureDiscretizerRegistry::setProcessInstance(FeatureDiscretizerRegistry* registry)
{
	assert(registry != nullptr);
	g_processFeatureDiscretizerRegistry = registry;
	flushDeferredDiscretizers(*registry);
}

FeatureDiscretizerRegistry& FeatureDiscretizerRegistry::instance()
{
	assert(g_processFeatureDiscretizerRegistry != nullptr);
	return *g_processFeatureDiscretizerRegistry;
}

void FeatureDiscretizerRegistry::registerDiscretizer(std::unique_ptr<IFeatureDiscretizer> discretizer)
{
	if (!discretizer)
	{
		return;
	}
	std::unique_lock<std::shared_mutex> lock(m_mutex);
	const std::string id = discretizer->strategyId();
	for (const std::unique_ptr<IFeatureDiscretizer>& existing : m_discretizers)
	{
		if (existing && existing->strategyId() == id)
		{
			return;
		}
	}
	m_discretizers.push_back(std::move(discretizer));
}

const IFeatureDiscretizer* FeatureDiscretizerRegistry::get(const std::string& strategyId) const
{
	std::shared_lock<std::shared_mutex> lock(m_mutex);
	for (const std::unique_ptr<IFeatureDiscretizer>& d : m_discretizers)
	{
		if (d && d->strategyId() == strategyId)
		{
			return d.get();
		}
	}
	return nullptr;
}

std::vector<std::string> FeatureDiscretizerRegistry::listStrategyIds() const
{
	std::shared_lock<std::shared_mutex> lock(m_mutex);
	std::vector<std::string> ids;
	ids.reserve(m_discretizers.size());
	for (const std::unique_ptr<IFeatureDiscretizer>& d : m_discretizers)
	{
		if (d)
		{
			ids.push_back(d->strategyId());
		}
	}
	std::sort(ids.begin(), ids.end());
	return ids;
}

} // namespace geoalgo
