#include "Ai/GeometryRecognizeDomainHandler.h"

#include "AiCommandSchema.h"
#include "AiDomainTypes.h"
#include "IAiAssistantHost.h"
#include "IPluginHostContext.h"
#include "PluginPrimitiveTypes.h"

#include <json.hpp>

#include <cmath>
#include <set>

namespace
{
const std::set<std::string> kPrimitives = { "box", "cylinder", "cone", "sphere", "unknown" };

bool isPositiveDim(double v)
{
	return std::isfinite(v) && v >= AiCommandSchema::kMinDimMm && v <= AiCommandSchema::kMaxDimMm;
}

bool readDim(const nlohmann::json& dims, const char* key, double& out)
{
	if (!dims.contains(key) || !dims[key].is_number())
		return false;
	const double v = dims[key].get<double>();
	if (!isPositiveDim(v))
		return false;
	out = v;
	return true;
}

bool validateDimensionsForPrimitive(const std::string& prim, const nlohmann::json& dims, QString* err)
{
	if (!dims.is_object())
	{
		if (err)
			*err = QStringLiteral("dimensions_mm must be an object.");
		return false;
	}
	if (prim == "unknown")
		return true;

	if (prim == "box")
	{
		double L = 0, W = 0, H = 0;
		if (!readDim(dims, "length", L) || !readDim(dims, "width", W) || !readDim(dims, "height", H))
		{
			if (err)
				*err = QStringLiteral("box requires positive length, width, height (mm).");
			return false;
		}
		return true;
	}
	if (prim == "cylinder" || prim == "cone")
	{
		double R = 0, H = 0;
		if (!readDim(dims, "radius", R) || !readDim(dims, "height", H))
		{
			if (err)
				*err = QStringLiteral("%1 requires positive radius and height (mm).").arg(QString::fromStdString(prim));
			return false;
		}
		return true;
	}
	if (prim == "sphere")
	{
		double R = 0, D = 0;
		const bool hasR = readDim(dims, "radius", R);
		const bool hasD = readDim(dims, "diameter", D);
		if (!hasR && !hasD)
		{
			if (err)
				*err = QStringLiteral("sphere requires positive radius or diameter (mm).");
			return false;
		}
		return true;
	}
	if (err)
		*err = QStringLiteral("Unsupported primitive.");
	return false;
}
}

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
	if (!j.is_object())
	{
		if (err)
			*err = QStringLiteral("Recognition output must be a JSON object.");
		return false;
	}

	const std::string prim = j.value("primitive", std::string());
	if (prim.empty() || kPrimitives.find(prim) == kPrimitives.end())
	{
		if (err)
			*err = QStringLiteral("primitive must be box, cylinder, cone, sphere, or unknown.");
		return false;
	}

	if (j.contains("confidence") && !j["confidence"].is_number())
	{
		if (err)
			*err = QStringLiteral("confidence must be a number.");
		return false;
	}

	return validateDimensionsForPrimitive(prim, j.value("dimensions_mm", nlohmann::json::object()), err);
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

	if (!validateOutput(domainJson, err))
		return false;

	const std::string prim = j.value("primitive", std::string());
	if (prim == "unknown")
	{
		if (err)
			*err = QStringLiteral("Cannot create mesh for unknown primitive.");
		return false;
	}

	nlohmann::json plan;
	plan["version"] = 2;
	plan["steps"] = nlohmann::json::array();
	nlohmann::json step;
	step["api"] = "createPrimitiveMesh";
	step["args"] = { { "primitive", prim }, { "dimensions_mm", j.value("dimensions_mm", nlohmann::json::object()) } };
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
