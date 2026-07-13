#include "FeatureDiscretizerConfigImpl.h"

#include "FeatureDiscretizerConfigRegistry.h"
#include "FeatureDiscretizerParamJsonIo.h"
#include "FeatureDiscretizerRegistry.h"

namespace geoalgo
{

FeatureDiscretizerConfigImpl::FeatureDiscretizerConfigImpl(
	std::string strategyId,
	std::string jsonRelativePath)
	: m_strategyId(std::move(strategyId))
	, m_jsonRelativePath(std::move(jsonRelativePath))
{
}

std::vector<FeatureDiscretizerParamField> FeatureDiscretizerConfigImpl::paramFields() const
{
	const IFeatureDiscretizer* algo = FeatureDiscretizerRegistry::instance().get(m_strategyId);
	const std::vector<FeatureDiscretizerParamField> fallback = algo ? algo->paramFields() : std::vector<FeatureDiscretizerParamField>{};
	const std::string& baseDir = FeatureDiscretizerConfigRegistry::instance().resourceBaseDir();
	const std::optional<nlohmann::json> root = loadFeatureDiscretizerJsonFile(baseDir, m_jsonRelativePath);
	if (!root.has_value() || !root->contains("schema"))
	{
		return fallback;
	}
	const std::vector<FeatureDiscretizerParamField> fromJson = parseFeatureSchemaFieldsFromJson((*root)["schema"]);
	if (fromJson.empty())
	{
		return fallback;
	}
	std::vector<FeatureDiscretizerParamField> merged = fallback;
	for (const FeatureDiscretizerParamField& o : fromJson)
	{
		bool replaced = false;
		for (FeatureDiscretizerParamField& b : merged)
		{
			if (b.key == o.key)
			{
				b.labelEn = o.labelEn;
				b.labelZh = o.labelZh;
				b.unit = o.unit;
				b.group = o.group;
				b.order = o.order;
				b.minValue = o.minValue;
				b.maxValue = o.maxValue;
				b.step = o.step;
				b.minInt = o.minInt;
				b.maxInt = o.maxInt;
				b.defaultDouble = o.defaultDouble;
				b.defaultInt = o.defaultInt;
				b.defaultBool = o.defaultBool;
				b.enumValues = o.enumValues;
				b.enumLabelsZh = o.enumLabelsZh;
				b.enumLabelsEn = o.enumLabelsEn;
				replaced = true;
				break;
			}
		}
		if (!replaced)
		{
			merged.push_back(o);
		}
	}
	return merged;
}

nlohmann::json FeatureDiscretizerConfigImpl::defaultParams() const
{
	const std::string& baseDir = FeatureDiscretizerConfigRegistry::instance().resourceBaseDir();
	const std::optional<nlohmann::json> root = loadFeatureDiscretizerJsonFile(baseDir, m_jsonRelativePath);
	if (root.has_value())
	{
		nlohmann::json params = parseFeatureDefaultParamsFromJson(*root);
		if (!params.empty())
		{
			return params;
		}
	}
	nlohmann::json params = nlohmann::json::object();
	for (const FeatureDiscretizerParamField& field : paramFields())
	{
		switch (field.type)
		{
		case FeatureParamType::Int:
		case FeatureParamType::Enum:
			params[field.key] = field.defaultInt;
			break;
		case FeatureParamType::Bool:
			params[field.key] = field.defaultBool;
			break;
		case FeatureParamType::Double:
			params[field.key] = field.defaultDouble;
			break;
		default:
			break;
		}
	}
	return params;
}

std::unique_ptr<IFeatureDiscretizerConfig> makeFeatureDiscretizerConfig(
	const char* strategyId,
	const char* jsonRelativePath)
{
	return std::make_unique<FeatureDiscretizerConfigImpl>(strategyId, jsonRelativePath);
}

} // namespace geoalgo
