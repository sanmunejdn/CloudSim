#include "Ai/TrajectoryFeatureDomainHandler.h"

#include "AiDomainTypes.h"
#include "GeometryRef.h"
#include "IPluginHostContext.h"

#include <json.hpp>

#include <FeatureListDocument.h>

QString TrajectoryFeatureDomainHandler::domainId() const
{
	return AiDomainIds::trajectoryFeature();
}

bool TrajectoryFeatureDomainHandler::validateOutput(const QByteArray& jsonUtf8, QString* err) const
{
	nlohmann::json j;
	try
	{
		j = nlohmann::json::parse(jsonUtf8.constData(), nullptr, true);
	}
	catch (...)
	{
		if (err)
		{
			*err = QStringLiteral("Invalid trajectory feature JSON.");
		}
		return false;
	}
	if (!j.is_object() || !j.contains("features") || !j["features"].is_array())
	{
		if (err)
		{
			*err = QStringLiteral("Missing features array.");
		}
		return false;
	}
	for (const auto& f : j["features"])
	{
		geoalgo::FeatureListDocument doc;
		std::string parseErr;
		nlohmann::json wrap = nlohmann::json::object();
		wrap["schemaVersion"] = 2;
		wrap["features"] = nlohmann::json::array({f});
		if (!geometry_backend_ops::featureListFromJson(wrap.dump(), doc, &parseErr))
		{
			if (err)
			{
				*err = QString::fromStdString(parseErr);
			}
			return false;
		}
	}
	return true;
}

bool TrajectoryFeatureDomainHandler::adaptToActionPlan(const QByteArray& domainJson, QByteArray* outPlan, QString* err) const
{
	(void)err;
	nlohmann::json j;
	try
	{
		j = nlohmann::json::parse(domainJson.constData(), nullptr, true);
	}
	catch (...)
	{
		if (err)
		{
			*err = QStringLiteral("Invalid JSON.");
		}
		return false;
	}
	nlohmann::json plan;
	plan["version"] = 2;
	plan["schemaVersion"] = 2;
	plan["domain"] = "trajectory.feature";
	plan["features"] = j.value("features", nlohmann::json::array());
	plan["suggestedPipelineTemplate"] = j.value("suggestedPipelineTemplate", "weld_default");
	if (j.contains("workpiece"))
	{
		plan["workpiece"] = j["workpiece"];
	}
	if (outPlan)
	{
		*outPlan = QByteArray::fromStdString(plan.dump());
	}
	return true;
}

bool TrajectoryFeatureDomainHandler::execute(
	const QByteArray& jsonUtf8,
	IPluginHostContext* host,
	IAiAssistantHost* aiHost,
	QString* summary,
	QString* err)
{
	(void)host;
	(void)aiHost;
	if (!validateOutput(jsonUtf8, err))
	{
		return false;
	}
	if (summary)
	{
		*summary = QStringLiteral("Trajectory features validated (discretize via Robot trajectory page).");
	}
	return true;
}
