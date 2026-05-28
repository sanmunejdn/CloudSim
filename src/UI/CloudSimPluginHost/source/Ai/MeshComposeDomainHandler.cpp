#include "Ai/MeshComposeDomainHandler.h"

#include "Ai/AiActionPlanExecutor.h"
#include "AiDomainTypes.h"
#include "IAiAssistantHost.h"
#include "PluginHostContext.h"

#include <json.hpp>

#include <QSet>
#include <QString>

namespace
{
bool isKnownApi(const std::string& api)
{
	return api == "createPrimitiveMesh" || api == "booleanMesh" || api == "importFileIntoActiveDocument";
}
}

QString MeshComposeDomainHandler::domainId() const
{
	return AiDomainIds::meshCompose();
}

bool MeshComposeDomainHandler::validatePlanJson(const nlohmann::json& root, QString* err)
{
	if (!root.is_object())
	{
		if (err)
			*err = QStringLiteral("Plan must be a JSON object.");
		return false;
	}
	if (root.value("version", 0) != 2)
	{
		if (err)
			*err = QStringLiteral("mesh.compose requires version 2.");
		return false;
	}
	if (!root.contains("steps") || !root["steps"].is_array() || root["steps"].empty())
	{
		if (err)
			*err = QStringLiteral("Plan requires non-empty steps[].");
		return false;
	}
	QSet<QString> definedIds;
	for (const auto& step : root["steps"])
	{
		if (!step.is_object())
		{
			if (err)
				*err = QStringLiteral("Each step must be an object.");
			return false;
		}
		const std::string api = step.value("api", "");
		if (!isKnownApi(api))
		{
			if (err)
				*err = QStringLiteral("Unknown api: %1").arg(QString::fromStdString(api));
			return false;
		}
		const nlohmann::json args = step.contains("args") && step["args"].is_object() ? step["args"] : nlohmann::json::object();
		if (api == "booleanMesh")
		{
			const std::string op = args.value("op", "difference");
			if (op != "difference" && op != "union" && op != "intersection")
			{
				if (err)
					*err = QStringLiteral("booleanMesh op must be difference|union|intersection.");
				return false;
			}
			auto checkRef = [&](const char* key) {
				const std::string ref = args.value(key, "");
				if (ref.empty())
					return false;
				if (ref.front() == '$')
				{
					const QString sid = QString::fromStdString(ref.substr(1));
					return definedIds.contains(sid);
				}
				return true;
			};
			if (!checkRef("target") || !checkRef("tool"))
			{
				if (err)
					*err = QStringLiteral("booleanMesh target/tool must be $stepId defined earlier.");
				return false;
			}
		}
		if (step.contains("id"))
		{
			const QString id = QString::fromStdString(step.value("id", ""));
			if (id.isEmpty())
			{
				if (err)
					*err = QStringLiteral("Step id must be non-empty.");
				return false;
			}
			definedIds.insert(id);
		}
	}
	return true;
}

bool MeshComposeDomainHandler::validateOutput(const QByteArray& jsonUtf8, QString* err) const
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
	return validatePlanJson(j, err);
}

bool MeshComposeDomainHandler::execute(const QByteArray& jsonUtf8, IPluginHostContext* host, IAiAssistantHost* aiHost,
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
	return AiActionPlanExecutor::execute(*ph, jsonUtf8, summary, err);
}
