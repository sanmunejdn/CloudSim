/// @file TrajectoryParamJsonIo.cpp
/// @brief TrajectoryParamJsonIo 实现

// TrajectoryParamJsonIo 实现
#include "TrajectoryParamJsonIo.h"

#include <fstream>

namespace trajectory_algo
{
namespace
{
TrajectoryParamType paramTypeFromToken(const std::string& token)
{
	if (token == "Int" || token == "int")
	{
		return TrajectoryParamType::Int;
	}
	if (token == "Bool" || token == "bool")
	{
		return TrajectoryParamType::Bool;
	}
	if (token == "Enum" || token == "enum")
	{
		return TrajectoryParamType::Enum;
	}
	if (token == "Message" || token == "message")
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

std::vector<std::string> jsonCandidatePaths(const std::string& resourceBaseDir, const std::string& relativePath)
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

void readJsonNumber(const nlohmann::json& j, double& outDouble, int& outInt)
{
	if (j.is_number_float())
	{
		outDouble = j.get<double>();
		outInt = static_cast<int>(outDouble);
	}
	else if (j.is_number_integer() || j.is_number_unsigned())
	{
		outInt = j.get<int>();
		outDouble = static_cast<double>(outInt);
	}
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
	// default 可能是 bool/number；对 bool 调 get<double> 会抛 type_error
	if (item.contains("default"))
	{
		const nlohmann::json& def = item["default"];
		if (def.is_boolean())
		{
			field.defaultBool = def.get<bool>();
		}
		else if (def.is_number())
		{
			readJsonNumber(def, field.defaultDouble, field.defaultInt);
		}
	}
	if (item.contains("defaultInt") && item["defaultInt"].is_number())
	{
		double unused = 0.0;
		readJsonNumber(item["defaultInt"], unused, field.defaultInt);
	}
	if (item.contains("defaultBool") && item["defaultBool"].is_boolean())
	{
		field.defaultBool = item["defaultBool"].get<bool>();
	}
	field.visibleWhenScopeKind = item.value("visibleWhenScopeKind", "");
	field.messageEn = item.value("messageEn", "");
	field.messageZh = item.value("messageZh", "");
	if (item.contains("enumValues") && item["enumValues"].is_array())
	{
		for (const nlohmann::json& v : item["enumValues"])
		{
			if (v.is_string())
			{
				field.enumValues.push_back(v.get<std::string>());
			}
		}
	}
	if (item.contains("enumLabelsZh") && item["enumLabelsZh"].is_array())
	{
		for (const nlohmann::json& v : item["enumLabelsZh"])
		{
			if (v.is_string())
			{
				field.enumLabelsZh.push_back(v.get<std::string>());
			}
		}
	}
	if (item.contains("enumLabelsEn") && item["enumLabelsEn"].is_array())
	{
		for (const nlohmann::json& v : item["enumLabelsEn"])
		{
			if (v.is_string())
			{
				field.enumLabelsEn.push_back(v.get<std::string>());
			}
		}
	}
	return field;
}

} // namespace

std::optional<nlohmann::json> loadTrajectoryJsonFile(const std::string& resourceBaseDir,
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
	const std::vector<TrajectoryOpParamField> parsed =
		parseSchemaFieldsFromJson(root->value("schema", nlohmann::json::object()));
	if (parsed.empty())
	{
		return trajectoryOpCommonScopeFields();
	}
	// JSON 空枚举会冲掉 C++ 作用域选项，按 key 合并保留有效 enumValues
	std::vector<TrajectoryOpParamField> merged = trajectoryOpCommonScopeFields();
	for (const TrajectoryOpParamField& o : parsed)
	{
		bool replaced = false;
		for (TrajectoryOpParamField& b : merged)
		{
			if (b.key != o.key)
			{
				continue;
			}
			if (!o.labelEn.empty())
			{
				b.labelEn = o.labelEn;
			}
			if (!o.labelZh.empty())
			{
				b.labelZh = o.labelZh;
			}
			if (!o.group.empty())
			{
				b.group = o.group;
			}
			b.order = o.order;
			b.minInt = o.minInt;
			b.maxInt = o.maxInt;
			b.defaultInt = o.defaultInt;
			if (!o.enumValues.empty())
			{
				b.type = TrajectoryParamType::Enum;
				b.enumValues = o.enumValues;
				b.enumLabelsZh = o.enumLabelsZh;
				b.enumLabelsEn = o.enumLabelsEn;
			}
			if (!o.visibleWhenScopeKind.empty())
			{
				b.visibleWhenScopeKind = o.visibleWhenScopeKind;
			}
			if (o.type == TrajectoryParamType::Int)
			{
				b.type = TrajectoryParamType::Int;
			}
			replaced = true;
			break;
		}
		if (!replaced)
		{
			merged.push_back(o);
		}
	}
	return merged;
}

} // namespace trajectory_algo
