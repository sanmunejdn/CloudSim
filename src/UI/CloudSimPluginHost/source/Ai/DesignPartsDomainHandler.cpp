/// @file DesignPartsDomainHandler.cpp
/// @brief design.parts Domain：校验后走 feature.compose 执行器

#include "Ai/DesignPartsDomainHandler.h"

#include "Ai/AiActionPlanExecutor.h"
#include "Ai/FeatureComposeDomainHandler.h"
#include "AiDomainTypes.h"
#include "IAiAssistantHost.h"
#include "PluginHostContext.h"

#include <json.hpp>

DesignPartsDomainHandler::DesignPartsDomainHandler()
{
	QString err;
	const QString root = DesignPartsCatalog::resolvePartsRoot();
	m_catalog.load(root, &err);
}

QString DesignPartsDomainHandler::domainId() const
{
	return AiDomainIds::designParts();
}

bool DesignPartsDomainHandler::validateOutput(const QByteArray& jsonUtf8, QString* err) const
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
	// 允许直接 feature.compose，或 {part_id, params} 包装
	if (j.value("domain", "") == "feature.compose" || (j.value("version", 0) == 2 && j.contains("steps")))
		return FeatureComposeDomainHandler::validatePlanJson(j, err);
	if (j.contains("part_id") && j["part_id"].is_string())
		return true;
	if (err)
		*err = QStringLiteral("Expected feature.compose plan or {part_id,params}.");
	return false;
}

bool DesignPartsDomainHandler::execute(const QByteArray& jsonUtf8, IPluginHostContext* host, IAiAssistantHost* aiHost,
									   QString* summary, QString* err)
{
	(void)aiHost;
	auto* ph = dynamic_cast<PluginHostContext*>(host);
	if (!ph)
	{
		if (err)
			*err = QStringLiteral("Plugin host unavailable.");
		return false;
	}
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
	QByteArray plan = jsonUtf8;
	if (j.contains("part_id") && !(j.value("version", 0) == 2 && j.contains("steps")))
	{
		const QString pid = QString::fromStdString(j["part_id"].get<std::string>());
		QByteArray params = QByteArrayLiteral("{}");
		if (j.contains("params") && j["params"].is_object())
			params = QByteArray::fromStdString(j["params"].dump());
		QString iErr;
		if (!m_catalog.instantiate(pid, params, &plan, &iErr))
		{
			if (err)
				*err = iErr;
			return false;
		}
	}
	return AiActionPlanExecutor::execute(*ph, plan, summary, err);
}

bool DesignPartsDomainHandler::tryParseUserText(const QString& text, QByteArray* outPlanUtf8, QString* hint,
												QString* err) const
{
	QString partId;
	return m_catalog.tryParseUserText(text, outPlanUtf8, &partId, hint, err);
}
