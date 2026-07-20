/// @file FeatureDiscretizerRegistry.cpp
/// @brief FeatureDiscretizerRegistry 实现

#include "FeatureDiscretizerRegistry.h"

#include <algorithm>

namespace geoalgo
{
FeatureDiscretizerRegistry& FeatureDiscretizerRegistry::instance()
{
	static FeatureDiscretizerRegistry registry;
	return registry;
}

void FeatureDiscretizerRegistry::registerDiscretizer(std::unique_ptr<IFeatureDiscretizer> discretizer)
{
	if (!discretizer)
	{
		return;
	}
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
