// TrajectoryParamJsonIo 实现
#include "TrajectoryParamJsonIo.h"

#include <fstream>

namespace trajectory_algo
{
namespace
{

TrajectoryParamType paramTypeFromToken(const std::string& token)
{
	if (token == "Int")
	{
		return TrajectoryParamType::Int;
	}
	if (token == "Bool")
	{
		return TrajectoryParamType::Bool;
	}
	if (token == "Enum")
	{
		return TrajectoryParamType::Enum;
	}
	if (token == "Message")
	{
		return TrajectoryParamType::Message;
	}
	return TrajectoryParamType::Double;
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

std::vector<std::string> jsonCandidatePaths(
	const std::string& resourceBaseDir,
	const std::string& relativePath)
{
	std::vector<std::string> out;
	if (!resourceBaseDir.empty())
	{
		out.push_back(resourceBaseDir + "/resource/trajectory/" + relativePath);
		const std::size_t slash = relativePath.find_last_of('/');
		if (slash != std::string::npos)
		{
			const std::string baseName = relativePath.substr(slash + 1);
			const std::string stem = baseName.substr(0, baseName.find('.'));
			out.push_back(resourceBaseDir + "/resource/trajectory/" + stem + ".defaults.json");
		}
	}
	out.push_back("resource/trajectory/" + relativePath);
	const std::size_t slash = relativePath.find_last_of('/');
	if (slash != std::string::npos)
	{
		const std::string baseName = relativePath.substr(slash + 1);
		const std::string stem = baseName.substr(0, baseName.find('.'));
		out.push_back("resource/trajectory/" + stem + ".defaults.json");
	}
	return out;
}

TrajectoryOpParamField fieldFromJson(const nlohmann::json& item)
{
	TrajectoryOpParamField field{};
	field.key = item.value("key", "");
	field.type = paramTypeFromToken(item.value("type", "Double"));
	field.labelEn = item.value("labelEn", field.key);
	field.labelZh = item.value("labelZh", field.labelEn);
	field.unit = item.value("unit", "");
	field.group = item.value("group", "transform");
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
	field.visibleWhenScopeKind = item.value("visibleWhenScopeKind", "");
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

std::optional<nlohmann::json> loadTrajectoryJsonFile(
	const std::string& resourceBaseDir,
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

std::vector<TrajectoryOpParamField> parseSchemaFieldsFromJson(const nlohmann::json& schemaRoot)
{
	std::vector<TrajectoryOpParamField> out;
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
		TrajectoryOpParamField field = fieldFromJson(item);
		if (!field.key.empty())
		{
			out.push_back(std::move(field));
		}
	}
	return out;
}

std::vector<TrajectoryOpParamField> loadCommonScopeFieldsFromJson(const std::string& resourceBaseDir)
{
	const std::optional<nlohmann::json> root = loadTrajectoryJsonFile(resourceBaseDir, "CommonScope.json");
	if (!root.has_value())
	{
		return trajectoryOpCommonScopeFields();
	}
	const std::vector<TrajectoryOpParamField> parsed = parseSchemaFieldsFromJson(root->value("schema", nlohmann::json::object()));
	if (parsed.empty())
	{
		return trajectoryOpCommonScopeFields();
	}
	return parsed;
}

} // namespace trajectory_algo
