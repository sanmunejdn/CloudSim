/// @file FeatureDiscretizerConfigRegistry.cpp
/// @brief FeatureDiscretizerConfigRegistry 实现

#include "FeatureDiscretizerConfigRegistry.h"

#include "FeatureDiscretizeParamUtils.h"
#include "FeatureDiscretizerRegistry.h"

#include <cassert>

namespace
{
geoalgo::FeatureDiscretizerConfigRegistry* g_processFeatureDiscretizerConfigRegistry = nullptr;

// 多文档并发时隔离加载态，避免共享成员被串改
thread_local std::string activeResourceBaseDir;
}

namespace geoalgo
{
void FeatureDiscretizerConfigRegistry::setProcessInstance(FeatureDiscretizerConfigRegistry* registry)
{
	assert(registry != nullptr);
	g_processFeatureDiscretizerConfigRegistry = registry;
}

FeatureDiscretizerConfigRegistry& FeatureDiscretizerConfigRegistry::instance()
{
	assert(g_processFeatureDiscretizerConfigRegistry != nullptr);
	return *g_processFeatureDiscretizerConfigRegistry;
}

void FeatureDiscretizerConfigRegistry::registerConfig(std::unique_ptr<IFeatureDiscretizerConfig> config)
{
	if (!config)
	{
		return;
	}
	m_configs.push_back(std::move(config));
}

bool FeatureDiscretizerConfigRegistry::ensureLoaded(const std::string& resourceBaseDir, std::string* errMsg)
{
	(void)errMsg;
	if (!resourceBaseDir.empty())
	{
		activeResourceBaseDir = resourceBaseDir;
	}
	return true;
}

const std::string& FeatureDiscretizerConfigRegistry::resourceBaseDir() const
{
	return activeResourceBaseDir;
}

const IFeatureDiscretizerConfig* FeatureDiscretizerConfigRegistry::configFor(const std::string& strategyId) const
{
	for (const std::unique_ptr<IFeatureDiscretizerConfig>& config : m_configs)
	{
		if (config && config->strategyId() == strategyId)
		{
			return config.get();
		}
	}
	return nullptr;
}

std::vector<FeatureDiscretizerParamField>
FeatureDiscretizerConfigRegistry::paramFieldsForStrategy(const std::string& strategyId) const
{
	const IFeatureDiscretizerConfig* config = configFor(strategyId);
	if (config)
	{
		return config->paramFields();
	}
	const IFeatureDiscretizer* algo = FeatureDiscretizerRegistry::instance().get(strategyId);
	if (algo)
	{
		return algo->paramFields();
	}
	return {};
}

nlohmann::json FeatureDiscretizerConfigRegistry::defaultParamsForStrategy(const std::string& strategyId) const
{
	const IFeatureDiscretizerConfig* config = configFor(strategyId);
	if (config)
	{
		return config->defaultParams();
	}
	return nlohmann::json::object();
}

} // namespace geoalgo
