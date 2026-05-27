#include "Ai/GeometryRecognizeDomainHandler.h"

#include "AiDomainTypes.h"
#include "IAiAssistantHost.h"
#include "IPluginHostContext.h"
#include "PluginPrimitiveTypes.h"

#include <json.hpp>

QString GeometryRecognizeDomainHandler::domainId() const
{
	return AiDomainIds::geometryRecognize();
}

bool GeometryRecognizeDomainHandler::validateOutput(const QByteArray& jsonUtf8, QString* err) const
{
	nlohmann::json j;
	try
	{
		j = nlohmann::json::parse(jsonUtf8.constData(), nullptr, true);
	}
	catch (...)
	{
		if (err)
			*err = QStringLiteral("Invalid recognition JSON.");
		return false;
	}
	return j.is_object();
}

bool GeometryRecognizeDomainHandler::adaptToActionPlan(const QByteArray& domainJson, QByteArray* outPlan, QString* err) const
{
	nlohmann::json j;
	try
	{
		j = nlohmann::json::parse(domainJson.constData(), nullptr, true);
	}
	catch (...)
	{
		if (err)
			*err = QStringLiteral("Invalid JSON.");
		return false;
	}
	const std::string label = j.value("primitive", j.value("label", ""));
	if (label.empty())
	{
		if (err)
			*err = QStringLiteral("Recognition result missing primitive/label.");
		return false;
	}
	nlohmann::json plan;
	plan["version"] = 2;
	plan["steps"] = nlohmann::json::array();
	nlohmann::json step;
	step["api"] = "createPrimitiveMesh";
	step["args"] = { { "primitive", label }, { "dimensions_mm", j.value("dimensions_mm", nlohmann::json::object()) } };
	if (j.contains("name"))
		step["args"]["name"] = j["name"];
	plan["steps"].push_back(step);
	if (outPlan)
		*outPlan = QByteArray::fromStdString(plan.dump());
	return true;
}

bool GeometryRecognizeDomainHandler::execute(const QByteArray& jsonUtf8, IPluginHostContext* host, IAiAssistantHost* aiHost,
	QString* summary, QString* err)
{
	(void)host;
	QByteArray plan;
	if (!adaptToActionPlan(jsonUtf8, &plan, err))
		return false;
	return aiHost->executeActionPlan(plan, summary, err);
}
