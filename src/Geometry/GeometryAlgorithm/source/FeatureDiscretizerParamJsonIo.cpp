/// @file FeatureDiscretizerParamJsonIo.cpp
/// @brief FeatureDiscretizerParamJsonIo 实现

#include "FeatureDiscretizerParamJsonIo.h"

#include <fstream>

namespace geoalgo
{
namespace
{
FeatureParamType paramTypeFromToken(const std::string& token)
{
	if (token == "Int")
	{
		return FeatureParamType::Int;
	}
	if (token == "Bool")
	{
		return FeatureParamType::Bool;
	}
	if (token == "Enum")
	{
		return FeatureParamType::Enum;
	}
	if (token == "Message")
	{
		return FeatureParamType::Message;
	}
	return FeatureParamType::Double;
}

std::optional<std::string> readTextFile(const std::string& path)
{
	std::ifstream in(path, std::ios::binary);
	if (!in)
	{
		return std::nullopt;
	}
	return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

std::vector<std::string> jsonCandidatePaths(const std::string& resourceBaseDir, const std::string& relativePath)
{
	std::vector<std::string> out;
	if (!resourceBaseDir.empty())
	{
		out.push_back(resourceBaseDir + "/resource/feature/" + relativePath);
	}
	out.push_back("resource/feature/" + relativePath);
	return out;
}

FeatureDiscretizerParamField fieldFromJson(const nlohmann::json& item)
{
	FeatureDiscretizerParamField field{};
	field.key = item.value("key", "");
	field.type = paramTypeFromToken(item.value("type", "Double"));
	field.labelEn = item.value("labelEn", field.key);
	field.labelZh = item.value("labelZh", field.labelEn);
	field.unit = item.value("unit", "");
	field.group = item.value("group", "discretize");
	field.order = item.value("order", 0);
	field.minValue = item.value("min", -1e6);
	field.maxValue = item.value("max", 1e6);
	field.step = item.value("step", 1.0);
	field.minInt = item.value("minInt", 0);
	field.maxInt = item.value("maxInt", 9999);
	field.defaultDouble = item.value("default", 0.0);
	if (item.contains("defaultInt"))
	{
		field.defaultInt = item["defaultInt"].get<int>();
	}
	else if (item.contains("default") && item["default"].is_number_integer())
	{
		field.defaultInt = item["default"].get<int>();
	}
	field.defaultBool = item.value("defaultBool", false);
	field.messageEn = item.value("messageEn", "");
	field.messageZh = item.value("messageZh", "");
	if (item.contains("enumValues") && item["enumValues"].is_array())
	{
		for (const nlohmann::json& v : item["enumValues"])
		{
			field.enumValues.push_back(v.get<std::string>());
		}
	}
	if (item.contains("enumLabelsZh") && item["enumLabelsZh"].is_array())
	{
		for (const nlohmann::json& v : item["enumLabelsZh"])
		{
			field.enumLabelsZh.push_back(v.get<std::string>());
		}
	}
	if (item.contains("enumLabelsEn") && item["enumLabelsEn"].is_array())
	{
		for (const nlohmann::json& v : item["enumLabelsEn"])
		{
			field.enumLabelsEn.push_back(v.get<std::string>());
		}
	}
	return field;
}

} // namespace

std::optional<nlohmann::json> loadFeatureDiscretizerJsonFile(const std::string& resourceBaseDir,
															 const std::string& relativePath)
{
	for (const std::string& path : jsonCandidatePaths(resourceBaseDir, relativePath))
	{
		const std::optional<std::string> text = readTextFile(path);
		if (!text.has_value() || text->empty())
		{
			continue;
		}
		try
		{
			return nlohmann::json::parse(*text);
		}
		catch (...)
		{
			continue;
		}
	}
	return std::nullopt;
}

std::vector<FeatureDiscretizerParamField> parseFeatureSchemaFieldsFromJson(const nlohmann::json& schemaRoot)
{
	std::vector<FeatureDiscretizerParamField> out;
	if (!schemaRoot.contains("fields") || !schemaRoot["fields"].is_array())
	{
		return out;
	}
	for (const nlohmann::json& item : schemaRoot["fields"])
	{
		if (!item.is_object())
		{
			continue;
		}
		FeatureDiscretizerParamField field = fieldFromJson(item);
		if (!field.key.empty())
		{
			out.push_back(std::move(field));
		}
	}
	return out;
}

nlohmann::json parseFeatureDefaultParamsFromJson(const nlohmann::json& root)
{
	if (root.contains("defaults") && root["defaults"].is_object() && root["defaults"].contains("params"))
	{
		return root["defaults"]["params"];
	}
	return nlohmann::json::object();
}

} // namespace geoalgo
