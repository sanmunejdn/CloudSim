#include "FeatureDiscretizerConfigRegistry.h"

#include "FeatureDiscretizerRegistry.h"
#include "FeatureDiscretizeParamUtils.h"

namespace geoalgo
{

FeatureDiscretizerConfigRegistry& FeatureDiscretizerConfigRegistry::instance()
{
	static FeatureDiscretizerConfigRegistry registry;
	return registry;
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
		m_resourceBaseDir = resourceBaseDir;
	}
	m_loaded = true;
	return true;
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

std::vector<FeatureDiscretizerParamField> FeatureDiscretizerConfigRegistry::paramFieldsForStrategy(
	const std::string& strategyId) const
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
