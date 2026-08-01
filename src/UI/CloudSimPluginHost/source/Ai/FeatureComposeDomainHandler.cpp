/// @file FeatureComposeDomainHandler.cpp
/// @brief feature.compose 校验与执行

#include "Ai/FeatureComposeDomainHandler.h"

#include "Ai/AiActionPlanExecutor.h"
#include "AiDomainTypes.h"
#include "IAiAssistantHost.h"
#include "PluginHostContext.h"

#include <QSet>
#include <QString>

#include <json.hpp>

namespace
{
bool isKnownApi(const std::string& api)
{
	return api == "askClarify" || api == "extrudeSketchProfileToBrep" || api == "filletEdgesToBrep" ||
		   api == "chamferEdgesToBrep" || api == "revolveSketchProfileToBrep" || api == "linearPatternBodyToBrep" ||
		   api == "sweepSketchProfileToBrep" || api == "loftSketchProfilesToBrep" || api == "shellFacesToBrep" ||
		   api == "draftFacesToBrep" || api == "circularPatternBodyToBrep";
}
} // namespace

QString FeatureComposeDomainHandler::domainId() const
{
	return AiDomainIds::featureCompose();
}

bool FeatureComposeDomainHandler::validatePlanJson(const nlohmann::json& root, QString* err)
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
			*err = QStringLiteral("feature.compose requires version 2.");
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
		const nlohmann::json args =
			step.contains("args") && step["args"].is_object() ? step["args"] : nlohmann::json::object();
		if (api == "extrudeSketchProfileToBrep")
		{
			const std::string mode = args.value("mode", "pad");
			if (mode != "pad" && mode != "pocket")
			{
				if (err)
					*err = QStringLiteral("extrude mode must be pad|pocket.");
				return false;
			}
			if (mode == "pocket")
			{
				const std::string target = args.value("target", "");
				if (target.empty() || target.front() != '$' ||
					!definedIds.contains(QString::fromStdString(target.substr(1))))
				{
					if (err)
						*err = QStringLiteral("pocket requires target $stepId defined earlier.");
					return false;
				}
			}
		}
		if (api == "filletEdgesToBrep" || api == "chamferEdgesToBrep" || api == "linearPatternBodyToBrep" ||
			api == "shellFacesToBrep" || api == "draftFacesToBrep" || api == "circularPatternBodyToBrep")
		{
			const std::string target = args.value("target", "");
			if (target.empty() || target.front() != '$' ||
				!definedIds.contains(QString::fromStdString(target.substr(1))))
			{
				if (err)
					*err = QStringLiteral("%1 requires target $stepId.")
							   .arg(QString::fromStdString(api));
				return false;
			}
		}
		if (api == "linearPatternBodyToBrep" || api == "circularPatternBodyToBrep")
		{
			const int count = args.value("count", 0);
			if (count < 2)
			{
				if (err)
					*err = QStringLiteral("%1 requires count >= 2.").arg(QString::fromStdString(api));
				return false;
			}
		}
		if (api == "sweepSketchProfileToBrep" || api == "loftSketchProfilesToBrep")
		{
			const std::string mode = args.value("mode", "boss");
			if (mode == "cut")
			{
				const std::string target = args.value("target", "");
				if (target.empty() || target.front() != '$' ||
					!definedIds.contains(QString::fromStdString(target.substr(1))))
				{
					if (err)
						*err = QStringLiteral("%1 cut requires target $stepId defined earlier.")
								   .arg(QString::fromStdString(api));
					return false;
				}
			}
		}
		if (api == "shellFacesToBrep" || api == "draftFacesToBrep")
		{
			if (!args.contains("face_indices") || !args["face_indices"].is_array() ||
				args["face_indices"].empty())
			{
				if (err)
					*err = QStringLiteral("%1 requires non-empty face_indices int[].")
							   .arg(QString::fromStdString(api));
				return false;
			}
		}
		if (api == "revolveSketchProfileToBrep")
		{
			const std::string mode = args.value("mode", "boss");
			if (mode == "cut")
			{
				const std::string target = args.value("target", "");
				if (target.empty() || target.front() != '$' ||
					!definedIds.contains(QString::fromStdString(target.substr(1))))
				{
					if (err)
						*err = QStringLiteral("revolve cut requires target $stepId defined earlier.");
					return false;
				}
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

bool FeatureComposeDomainHandler::validateOutput(const QByteArray& jsonUtf8, QString* err) const
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

bool FeatureComposeDomainHandler::execute(const QByteArray& jsonUtf8, IPluginHostContext* host, IAiAssistantHost* aiHost,
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
