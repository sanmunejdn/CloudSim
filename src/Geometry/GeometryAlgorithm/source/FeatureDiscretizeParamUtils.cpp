/// @file FeatureDiscretizeParamUtils.cpp
/// @brief FeatureDiscretizeParamUtils 实现

#include "FeatureDiscretizeParamUtils.h"

#include <algorithm>

namespace geoalgo
{
double paramDouble(const nlohmann::json& params, const std::string& key, const double defaultValue)
{
	if (!params.is_object() || !params.contains(key))
	{
		return defaultValue;
	}
	const nlohmann::json& v = params[key];
	if (v.is_number())
	{
		return v.get<double>();
	}
	return defaultValue;
}

std::string paramString(const nlohmann::json& params, const std::string& key, const std::string& defaultValue)
{
	if (!params.is_object() || !params.contains(key))
	{
		return defaultValue;
	}
	const nlohmann::json& v = params[key];
	if (v.is_string())
	{
		return v.get<std::string>();
	}
	return defaultValue;
}

bool paramBool(const nlohmann::json& params, const std::string& key, const bool defaultValue)
{
	if (!params.is_object() || !params.contains(key))
	{
		return defaultValue;
	}
	const nlohmann::json& v = params[key];
	if (v.is_boolean())
	{
		return v.get<bool>();
	}
	return defaultValue;
}

int paramInt(const nlohmann::json& params, const std::string& key, const int defaultValue)
{
	if (!params.is_object() || !params.contains(key))
	{
		return defaultValue;
	}
	const nlohmann::json& v = params[key];
	if (v.is_number_integer())
	{
		return v.get<int>();
	}
	if (v.is_number())
	{
		return static_cast<int>(v.get<double>());
	}
	return defaultValue;
}

namespace
{
DiscretizeMode parseDiscretizeMode(const nlohmann::json& params)
{
	const std::string token = paramString(params, "discretizeMode", "ChordThenStep");
	if (token == "ChordHeight")
	{
		return DiscretizeMode::ChordHeight;
	}
	if (token == "Step")
	{
		return DiscretizeMode::Step;
	}
	return DiscretizeMode::ChordThenStep;
}
} // namespace

DiscretizeParams buildDiscretizeParams(const nlohmann::json& params)
{
	DiscretizeParams d;
	d.mode = parseDiscretizeMode(params);
	d.stepMm = paramDouble(params, "stepMm", 2.0);
	d.linearDeflectionMm = paramDouble(params, "linearDeflectionMm", 0.01);
	d.closedPreserveEndpoint = paramBool(params, "closedPreserveEndpoint", false);
	d.outputTangent = paramBool(params, "outputTangent", true);
	d.outputNormal = paramBool(params, "outputNormal", true);
	return d;
}

FeatureDiscretizerParamField doubleFeatureParamField(const std::string& key, const std::string& labelEn,
													 const std::string& labelZh, const std::string& unit,
													 const double minValue, const double maxValue, const double step,
													 const double defaultValue, const int order,
													 const std::string& group)
{
	FeatureDiscretizerParamField field{};
	field.key = key;
	field.type = FeatureParamType::Double;
	field.labelEn = labelEn;
	field.labelZh = labelZh;
	field.unit = unit;
	field.group = group;
	field.order = order;
	field.minValue = minValue;
	field.maxValue = maxValue;
	field.step = step;
	field.defaultDouble = defaultValue;
	return field;
}

FeatureDiscretizerParamField intFeatureParamField(const std::string& key, const std::string& labelEn,
												  const std::string& labelZh, const int minValue, const int maxValue,
												  const int defaultValue, const int order, const std::string& group)
{
	FeatureDiscretizerParamField field{};
	field.key = key;
	field.type = FeatureParamType::Int;
	field.labelEn = labelEn;
	field.labelZh = labelZh;
	field.group = group;
	field.order = order;
	field.minInt = minValue;
	field.maxInt = maxValue;
	field.defaultInt = defaultValue;
	return field;
}

FeatureDiscretizerParamField boolFeatureParamField(const std::string& key, const std::string& labelEn,
												   const std::string& labelZh, const bool defaultValue, const int order,
												   const std::string& group)
{
	FeatureDiscretizerParamField field{};
	field.key = key;
	field.type = FeatureParamType::Bool;
	field.labelEn = labelEn;
	field.labelZh = labelZh;
	field.group = group;
	field.order = order;
	field.defaultBool = defaultValue;
	return field;
}

FeatureDiscretizerParamField enumFeatureParamField(const std::string& key, const std::string& labelEn,
												   const std::string& labelZh, const std::vector<std::string>& values,
												   const std::vector<std::string>& labelsZh,
												   const std::vector<std::string>& labelsEn, const int defaultIndex,
												   const int order, const std::string& group)
{
	FeatureDiscretizerParamField field{};
	field.key = key;
	field.type = FeatureParamType::Enum;
	field.labelEn = labelEn;
	field.labelZh = labelZh;
	field.group = group;
	field.order = order;
	field.enumValues = values;
	field.enumLabelsZh = labelsZh;
	field.enumLabelsEn = labelsEn;
	field.defaultInt = defaultIndex;
	return field;
}

std::vector<FeatureDiscretizerParamField> featureDiscretizerCommonParamFields()
{
	return {
		// ChordThenStep 置首：旧前端缺省回落 opts[0] 时仍显示「弦高+步距」
		enumFeatureParamField("discretizeMode", "Discretize mode", "离散模式",
							  {"ChordThenStep", "ChordHeight", "Step"}, {"弦高+步距", "仅弦高", "仅步距"},
							  {"Chord then step", "Chord height", "Step"}, 0, 0),
		doubleFeatureParamField("stepMm", "Step", "步距", "mm", 0.1, 1000.0, 0.1, 2.0, 1),
		doubleFeatureParamField("linearDeflectionMm", "Chord height", "弦高", "mm", 0.001, 100.0, 0.001, 0.01, 2),
		boolFeatureParamField("outputTangent", "Output tangent", "输出切向", true, 3),
		boolFeatureParamField("outputNormal", "Output normal", "输出法向", true, 4),
	};
}

void eraseDiscretizeModeField(std::vector<FeatureDiscretizerParamField>& fields)
{
	fields.erase(std::remove_if(fields.begin(), fields.end(),
								[](const FeatureDiscretizerParamField& f) { return f.key == "discretizeMode"; }),
				 fields.end());
}

} // namespace geoalgo
