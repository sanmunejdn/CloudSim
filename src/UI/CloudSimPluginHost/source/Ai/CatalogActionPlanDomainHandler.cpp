/// @file CatalogActionPlanDomainHandler.cpp
/// @brief 通用 Catalog ActionPlan 域 Handler

#include "Ai/CatalogActionPlanDomainHandler.h"

#include "Ai/AiActionPlanExecutor.h"
#include "PluginHostContext.h"

#include <json.hpp>

#include <unordered_set>

CatalogActionPlanDomainHandler::CatalogActionPlanDomainHandler(QString domainId, QByteArray catalogJsonUtf8)
	: m_domainId(std::move(domainId)), m_catalogJsonUtf8(std::move(catalogJsonUtf8))
{
}

QString CatalogActionPlanDomainHandler::domainId() const
{
	return m_domainId;
}

bool CatalogActionPlanDomainHandler::validateOutput(const QByteArray& jsonUtf8, QString* err) const
{
	nlohmann::json j;
	try
	{
		j = nlohmann::json::parse(jsonUtf8.constData(), nullptr, true);
	}
	catch (...)
	{
		if (err)
			*err = QStringLiteral("Invalid JSON.");
		return false;
	}
	if (j.value("version", 0) != 2 || !j.contains("steps") || !j["steps"].is_array() || j["steps"].empty())
	{
		if (err)
			*err = QStringLiteral("Expected action plan v2 with steps.");
		return false;
	}

	std::unordered_set<std::string> allowed;
	try
	{
		const nlohmann::json cat = nlohmann::json::parse(m_catalogJsonUtf8.constData(), nullptr, true);
		const auto apis = cat.value("apis", nlohmann::json::array());
		const std::string want = m_domainId.toStdString();
		for (const auto& api : apis)
		{
			if (!api.is_object() || !api.contains("id"))
				continue;
			bool okDom = false;
			for (const auto& d : api.value("domains", nlohmann::json::array()))
			{
				if (d.is_string() && d.get<std::string>() == want)
				{
					okDom = true;
					break;
				}
			}
			if (okDom && api["id"].is_string())
				allowed.insert(api["id"].get<std::string>());
		}
	}
	catch (...)
	{
		if (err)
			*err = QStringLiteral("ApiCatalog unavailable.");
		return false;
	}

	for (const auto& step : j["steps"])
	{
		if (!step.is_object())
			continue;
		const std::string api = step.value("api", "");
		if (api.empty() || allowed.find(api) == allowed.end())
		{
			if (err)
				*err = QStringLiteral("API '%1' not allowed in domain %2.")
						   .arg(QString::fromStdString(api), m_domainId);
			return false;
		}
	}
	return true;
}

bool CatalogActionPlanDomainHandler::execute(const QByteArray& jsonUtf8, IPluginHostContext* host,
											 IAiAssistantHost* aiHost, QString* summary, QString* err)
{
	(void)aiHost;
	auto* ph = dynamic_cast<PluginHostContext*>(host);
	if (!ph)
	{
		if (err)
			*err = QStringLiteral("Plugin host unavailable.");
		return false;
	}
	if (!validateOutput(jsonUtf8, err))
		return false;
	return AiActionPlanExecutor::execute(*ph, jsonUtf8, summary, err);
}
