#include "Ai/AiActionPlanExecutor.h"

#include "AiCommandSchema.h"
#include "PluginHostContext.h"
#include "PluginPrimitiveTypes.h"

#include <json.hpp>

namespace
{
PluginPrimitiveKind toPluginKind(BackendPrimitiveGeometry::PrimitiveKind k)
{
	switch (k)
	{
	case BackendPrimitiveGeometry::PrimitiveKind::Box:
		return PluginPrimitiveKind::Box;
	case BackendPrimitiveGeometry::PrimitiveKind::Cylinder:
		return PluginPrimitiveKind::Cylinder;
	case BackendPrimitiveGeometry::PrimitiveKind::Cone:
		return PluginPrimitiveKind::Cone;
	case BackendPrimitiveGeometry::PrimitiveKind::Sphere:
		return PluginPrimitiveKind::Sphere;
	}
	return PluginPrimitiveKind::Box;
}

bool executeCreateMeshStep(PluginHostContext& host, const nlohmann::json& cmd, QString* outError)
{
	BackendPrimitiveGeometry::PrimitiveMeshParams params;
	BackendPrimitiveGeometry::PrimitiveMeshQuality quality;
	std::string displayName;
	std::string sourcePath;
	std::string errStd;
	if (!AiCommandSchema::parseCreateMeshCommand(cmd, params, quality, displayName, sourcePath, errStd))
	{
		if (outError)
			*outError = QString::fromStdString(errStd);
		return false;
	}
	PluginPrimitiveMeshParams pp;
	pp.kind = toPluginKind(params.kind);
	pp.lengthMm = params.lengthMm;
	pp.widthMm = params.widthMm;
	pp.heightMm = params.heightMm;
	pp.radiusMm = params.radiusMm;
	pp.radiusTopMm = params.radiusTopMm;
	PluginPrimitiveMeshQuality pq;
	pq.segments = quality.segments;
	pq.rings = quality.rings;
	PluginMeshCreateOptions opt;
	opt.displayName = QString::fromStdString(displayName);
	opt.sourcePath = QString::fromStdString(sourcePath);
	opt.selectInTree = true;
	opt.resetViewToHome = true;
	if (cmd.contains("pose_mm") && cmd["pose_mm"].is_object())
	{
		const auto& p = cmd["pose_mm"];
		opt.poseMm.x = p.value("x", 0.0);
		opt.poseMm.y = p.value("y", 0.0);
		opt.poseMm.z = p.value("z", 0.0);
	}
	if (cmd.contains("rotation_deg") && cmd["rotation_deg"].is_object())
	{
		const auto& r = cmd["rotation_deg"];
		opt.rotationDeg.x = r.value("x", 0.0);
		opt.rotationDeg.y = r.value("y", 0.0);
		opt.rotationDeg.z = r.value("z", 0.0);
	}
	return host.createPrimitiveMesh(pp, pq, opt, outError);
}

bool executeStep(PluginHostContext& host, const nlohmann::json& step, QString* outError)
{
	const std::string api = step.value("api", "");
	const nlohmann::json args = step.contains("args") && step["args"].is_object() ? step["args"] : nlohmann::json::object();

	if (api == "createPrimitiveMesh")
	{
		nlohmann::json cmd;
		cmd["version"] = 1;
		cmd["action"] = "create_mesh";
		cmd["primitive"] = args.value("primitive", "box");
		if (args.contains("dimensions_mm"))
			cmd["dimensions_mm"] = args["dimensions_mm"];
		if (args.contains("name"))
			cmd["name"] = args["name"];
		if (args.contains("pose_mm"))
			cmd["pose_mm"] = args["pose_mm"];
		if (args.contains("rotation_deg"))
			cmd["rotation_deg"] = args["rotation_deg"];
		if (args.contains("mesh_quality"))
			cmd["mesh_quality"] = args["mesh_quality"];
		return executeCreateMeshStep(host, cmd, outError);
	}
	if (api == "importFileIntoActiveDocument")
	{
		const std::string path = args.value("path", "");
		if (path.empty())
		{
			if (outError)
				*outError = QStringLiteral("importFileIntoActiveDocument: path required.");
			return false;
		}
		const bool isPc = args.value("is_point_cloud", false);
		std::string err;
		const std::string id = host.importFileIntoActiveDocument(path, isPc, &err);
		if (id.empty())
		{
			if (outError)
				*outError = QString::fromStdString(err);
			return false;
		}
		return true;
	}
	if (outError)
		*outError = QStringLiteral("Unknown API: %1").arg(QString::fromStdString(api));
	return false;
}
}

namespace AiActionPlanExecutor
{
bool execute(const PluginHostContext& host, const QByteArray& planJsonUtf8, QString* outSummary, QString* outError)
{
	if (outSummary)
		outSummary->clear();
	if (outError)
		outError->clear();

	nlohmann::json root;
	try
	{
		root = nlohmann::json::parse(planJsonUtf8.constData(), nullptr, true);
	}
	catch (...)
	{
		if (outError)
			*outError = QStringLiteral("Invalid action plan JSON.");
		return false;
	}

	PluginHostContext& mutableHost = const_cast<PluginHostContext&>(host);

	if (root.contains("action") && root.value("action", "") == "create_mesh")
	{
		QString err;
		if (!executeCreateMeshStep(mutableHost, root, &err))
		{
			if (outError)
				*outError = err;
			return false;
		}
		if (outSummary)
			*outSummary = QStringLiteral("Created mesh from AI command.");
		return true;
	}

	const int ver = root.value("version", 0);
	if (ver != 2 || !root.contains("steps") || !root["steps"].is_array())
	{
		if (outError)
			*outError = QStringLiteral("Expected action plan version 2 with steps[].");
		return false;
	}

	int okCount = 0;
	for (const auto& step : root["steps"])
	{
		QString err;
		if (!executeStep(mutableHost, step, &err))
		{
			if (outError)
				*outError = QStringLiteral("Step %1 failed: %2").arg(okCount).arg(err);
			return false;
		}
		++okCount;
	}
	if (outSummary)
		*outSummary = QStringLiteral("Executed %1 step(s).").arg(okCount);
	return true;
}
}
