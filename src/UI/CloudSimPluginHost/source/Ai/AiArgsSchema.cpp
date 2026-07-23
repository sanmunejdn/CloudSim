/// @file AiArgsSchema.cpp
/// @brief args_schema 与 OpenAI parameters 单源转换

#include "Ai/AiArgsSchema.h"

#include "AiDomainTypes.h"

namespace AiArgsSchema
{
namespace
{
bool domainOk(const nlohmann::json& domains, const QString& domainId)
{
	if (domainId.isEmpty() || domainId == AiDomainIds::autoDomain())
		return true;
	if (!domains.is_array())
		return false;
	const std::string w = domainId.toStdString();
	for (const auto& d : domains)
	{
		if (d.is_string() && d.get<std::string>() == w)
			return true;
	}
	return false;
}
} // namespace

nlohmann::json toJsonSchemaParameters(const nlohmann::json& argsSchema)
{
	nlohmann::json props = nlohmann::json::object();
	nlohmann::json required = nlohmann::json::array();
	if (!argsSchema.is_array())
	{
		nlohmann::json parameters;
		parameters["type"] = "object";
		parameters["properties"] = props;
		return parameters;
	}
	for (const auto& f : argsSchema)
	{
		if (!f.is_object() || !f.contains("name") || !f["name"].is_string())
			continue;
		const std::string name = f["name"].get<std::string>();
		const std::string type = f.value("type", "string");
		if (type == "backend_pair")
		{
			props["source_backend_id"] = {{"type", "string"}};
			props["target_backend_id"] = {{"type", "string"}};
			if (f.value("required", false))
			{
				required.push_back("source_backend_id");
				required.push_back("target_backend_id");
			}
			continue;
		}
		nlohmann::json p;
		if (type == "number")
			p["type"] = "number";
		else if (type == "bool")
			p["type"] = "boolean";
		else
			p["type"] = "string";
		if (f.contains("label") && f["label"].is_string())
			p["description"] = f["label"];
		if (f.contains("enum") && f["enum"].is_array())
			p["enum"] = f["enum"];
		else if (f.contains("values") && f["values"].is_array())
			p["enum"] = f["values"];
		props[name] = p;
		if (f.value("required", false))
			required.push_back(name);
	}
	nlohmann::json parameters;
	parameters["type"] = "object";
	parameters["properties"] = props;
	if (!required.empty())
		parameters["required"] = required;
	return parameters;
}

nlohmann::json toOpenAiTool(const nlohmann::json& api)
{
	if (!api.is_object() || !api.contains("id") || !api["id"].is_string())
		return nullptr;
	const std::string id = api["id"].get<std::string>();
	std::string desc = api.value("summary", std::string());
	if (api.contains("keywords") && api["keywords"].is_array() && !api["keywords"].empty() &&
		api["keywords"][0].is_string())
		desc = api["keywords"][0].get<std::string>() + (desc.empty() ? "" : (" — " + desc));

	nlohmann::json tool;
	tool["type"] = "function";
	tool["function"]["name"] = id;
	tool["function"]["description"] = desc.empty() ? id : desc;
	tool["function"]["parameters"] = toJsonSchemaParameters(api.value("args_schema", nlohmann::json::array()));
	return tool;
}

QByteArray buildOpenAiToolsFromCatalog(const QByteArray& catalogJsonUtf8, const QString& domainId,
									   const QStringList& excludeApiIds)
{
	nlohmann::json tools = nlohmann::json::array();
	nlohmann::json root;
	try
	{
		root = nlohmann::json::parse(catalogJsonUtf8.constData(), nullptr, true);
	}
	catch (...)
	{
		return QByteArrayLiteral("[]");
	}
	if (!root.contains("apis") || !root["apis"].is_array())
		return QByteArrayLiteral("[]");
	for (const auto& api : root["apis"])
	{
		if (!api.is_object() || !api.contains("id") || !api["id"].is_string())
			continue;
		const QString id = QString::fromStdString(api["id"].get<std::string>());
		if (excludeApiIds.contains(id))
			continue;
		if (!domainOk(api.value("domains", nlohmann::json::array()), domainId))
			continue;
		nlohmann::json tool = toOpenAiTool(api);
		if (!tool.is_null())
			tools.push_back(tool);
	}
	return QByteArray::fromStdString(tools.dump());
}
} // namespace AiArgsSchema
