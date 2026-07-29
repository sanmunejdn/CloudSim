/// @file AiFeatureComposeSteps.cpp
/// @brief feature.compose 步骤实现

#include "Ai/AiFeatureComposeSteps.h"

#include "IPluginDocument.h"
#include "IPluginGeometryHost.h"
#include "PluginGeometryTypes.h"
#include "PluginHostContext.h"

#include <QEventLoop>
#include <cmath>
#include <functional>
#include <vector>

namespace
{
constexpr double kPi = 3.14159265358979323846;

bool waitGeomJobResult(const std::function<void(PluginGeometryFinishedFn)>& start, PluginGeometryJobResult* outJob,
					   QString* outError)
{
	QEventLoop loop;
	bool ok = false;
	QString err;
	PluginGeometryJobResult job;
	start(
		[&](bool success, const QString& error, const PluginGeometryJobResult& result)
		{
			ok = success;
			err = error;
			job = result;
			loop.quit();
		});
	loop.exec();
	if (outJob)
		*outJob = job;
	if (!ok)
	{
		if (outError)
			*outError = err.isEmpty() ? QStringLiteral("几何操作失败。") : err;
		return false;
	}
	return true;
}

PluginSketchPlane defaultXyPlane()
{
	PluginSketchPlane p;
	p.origin = {0, 0, 0};
	p.axisX = {1, 0, 0};
	p.axisY = {0, 1, 0};
	p.normal = {0, 0, 1};
	p.isPlanar = true;
	return p;
}

std::vector<float> closedRectXy(double lengthMm, double widthMm)
{
	std::vector<float> poly;
	poly.reserve(15);
	const double L = lengthMm;
	const double W = widthMm;
	const float pts[] = {0.f, 0.f, 0.f, float(L), 0.f, 0.f, float(L), float(W), 0.f, 0.f, float(W), 0.f, 0.f, 0.f, 0.f};
	poly.assign(pts, pts + 15);
	return poly;
}

std::vector<float> closedPolygonXy(int sides, double radiusMm)
{
	const int n = sides < 3 ? 3 : (sides > 24 ? 24 : sides);
	std::vector<float> poly;
	poly.reserve(static_cast<size_t>((n + 1) * 3));
	for (int i = 0; i <= n; ++i)
	{
		const double a = (2.0 * kPi * (i % n)) / n;
		poly.push_back(static_cast<float>(radiusMm * std::cos(a)));
		poly.push_back(static_cast<float>(radiusMm * std::sin(a)));
		poly.push_back(0.f);
	}
	return poly;
}

bool parseProfilePolyline(const nlohmann::json& args, std::vector<float>& out, QString* outError)
{
	out.clear();
	if (args.contains("profile_xyz_mm") && args["profile_xyz_mm"].is_array())
	{
		for (const auto& v : args["profile_xyz_mm"])
		{
			if (v.is_number())
				out.push_back(static_cast<float>(v.get<double>()));
		}
		if (out.size() < 12)
		{
			if (outError)
				*outError = QStringLiteral("profile_xyz_mm too short");
			return false;
		}
		return true;
	}

	const std::string profile = args.value("profile", "rectangle");
	if (profile == "rectangle" || profile == "rect" || profile == "box")
	{
		double L = args.value("length_mm", 0.0);
		double W = args.value("width_mm", 0.0);
		if (args.contains("size_mm") && args["size_mm"].is_object())
		{
			const auto& s = args["size_mm"];
			L = s.value("x", s.value("length", L));
			W = s.value("y", s.value("width", W));
		}
		if (args.contains("dimensions_mm") && args["dimensions_mm"].is_object())
		{
			const auto& d = args["dimensions_mm"];
			L = d.value("length", L);
			W = d.value("width", W);
		}
		if (L <= 0 || W <= 0)
		{
			if (outError)
				*outError = QStringLiteral("rectangle needs length_mm and width_mm > 0");
			return false;
		}
		out = closedRectXy(L, W);
		return true;
	}
	if (profile == "polygon")
	{
		const int sides = args.value("sides", 6);
		double R = args.value("radius_mm", 0.0);
		if (R <= 0 && args.contains("dimensions_mm"))
			R = args["dimensions_mm"].value("radius", 0.0);
		if (R <= 0)
		{
			if (outError)
				*outError = QStringLiteral("polygon needs radius_mm > 0");
			return false;
		}
		out = closedPolygonXy(sides, R);
		return true;
	}

	if (outError)
		*outError = QStringLiteral("Unknown profile; use rectangle|polygon or profile_xyz_mm");
	return false;
}

QString resolveTarget(const nlohmann::json& args, const QHash<QString, QString>& stepIdToBackendId, QString* outError)
{
	const std::string ref = args.value("target", "");
	if (ref.empty())
		return {};
	if (ref.front() == '$')
	{
		const QString id = stepIdToBackendId.value(QString::fromStdString(ref.substr(1)));
		if (id.isEmpty() && outError)
			*outError = QStringLiteral("Unknown target step %1").arg(QString::fromStdString(ref));
		return id;
	}
	return QString::fromStdString(ref);
}

bool executeExtrude(PluginHostContext& host, const nlohmann::json& args, const std::string& stepId,
					QHash<QString, QString>& stepIdToBackendId, QString* outError)
{
	IPluginDocument* doc = host.activeDocument();
	IPluginGeometryHost* geo = host.geometryHost();
	if (!doc || !geo)
	{
		if (outError)
			*outError = QStringLiteral("No active document or geometry host.");
		return false;
	}

	std::vector<float> profile;
	if (!parseProfilePolyline(args, profile, outError))
		return false;

	const std::string mode = args.value("mode", "pad");
	PluginSketchExtrudeParams p;
	p.mode = (mode == "pocket") ? PluginSketchExtrudeMode::Pocket : PluginSketchExtrudeMode::Pad;
	p.lengthMm = 0.0;
	if (args.contains("extrude_mm") && args["extrude_mm"].is_number())
		p.lengthMm = args["extrude_mm"].get<double>();
	else if (args.contains("height_mm") && args["height_mm"].is_number())
		p.lengthMm = args["height_mm"].get<double>();
	else if (args.contains("dimensions_mm") && args["dimensions_mm"].is_object())
		p.lengthMm = args["dimensions_mm"].value("height", 0.0);
	if (p.lengthMm <= 0.0)
		p.lengthMm = 10.0;
	p.reversed = args.value("reversed", false);
	p.endCondition = PluginSketchExtrudeEnd::Blind;
	if (args.contains("name") && args["name"].is_string())
		p.resultNameUtf8 = args["name"].get<std::string>();

	const QString target = resolveTarget(args, stepIdToBackendId, outError);
	if (p.mode == PluginSketchExtrudeMode::Pocket)
	{
		if (target.isEmpty())
			return false;
		p.targetParametricBackendIdUtf8 = target.toStdString();
	}
	else if (!target.isEmpty())
	{
		p.targetParametricBackendIdUtf8 = target.toStdString();
	}

	PluginGeometryJobResult job;
	if (!waitGeomJobResult(
			[&](PluginGeometryFinishedFn cb)
			{ geo->extrudeSketchProfileToBrep(doc, profile, defaultXyPlane(), p, std::move(cb)); },
			&job, outError))
		return false;

	if (!stepId.empty() && !job.newBackendId.empty())
		stepIdToBackendId[QString::fromStdString(stepId)] = QString::fromStdString(job.newBackendId);
	else if (!stepId.empty() && !target.isEmpty())
		stepIdToBackendId[QString::fromStdString(stepId)] = target;

	return true;
}

bool executeFillet(PluginHostContext& host, const nlohmann::json& args, const std::string& stepId,
				   QHash<QString, QString>& stepIdToBackendId, QString* outError)
{
	IPluginDocument* doc = host.activeDocument();
	IPluginGeometryHost* geo = host.geometryHost();
	if (!doc || !geo)
	{
		if (outError)
			*outError = QStringLiteral("No active document or geometry host.");
		return false;
	}

	const QString target = resolveTarget(args, stepIdToBackendId, outError);
	if (target.isEmpty())
		return false;

	PluginSketchFilletParams p;
	p.targetParametricBackendIdUtf8 = target.toStdString();
	p.radiusMm = args.value("radius_mm", 1.0);
	const std::string edges = args.value("edges", "");
	p.allEdges = (edges == "all");
	if (args.contains("edge_indices") && args["edge_indices"].is_array())
	{
		for (const auto& v : args["edge_indices"])
		{
			if (v.is_number_integer())
				p.edgeIndices.push_back(v.get<int>());
			else if (v.is_number())
				p.edgeIndices.push_back(static_cast<int>(v.get<double>()));
		}
	}
	if (p.edgeIndices.empty() && !p.allEdges)
	{
		if (outError)
			*outError = QStringLiteral("fillet needs edge_indices or edges=all");
		return false;
	}

	PluginGeometryJobResult job;
	if (!waitGeomJobResult([&](PluginGeometryFinishedFn cb) { geo->filletEdgesToBrep(doc, p, std::move(cb)); }, &job,
						   outError))
		return false;

	if (!stepId.empty())
		stepIdToBackendId[QString::fromStdString(stepId)] = target;
	return true;
}

bool executePattern(PluginHostContext& host, const nlohmann::json& args, const std::string& stepId,
					QHash<QString, QString>& stepIdToBackendId, QString* outError)
{
	IPluginDocument* doc = host.activeDocument();
	IPluginGeometryHost* geo = host.geometryHost();
	if (!doc || !geo)
	{
		if (outError)
			*outError = QStringLiteral("No active document or geometry host.");
		return false;
	}

	const QString target = resolveTarget(args, stepIdToBackendId, outError);
	if (target.isEmpty())
		return false;

	PluginSketchLinearPatternParams p;
	p.targetParametricBackendIdUtf8 = target.toStdString();
	p.count = args.value("count", 2);
	p.dxMm = args.value("dx_mm", 10.0);
	p.dyMm = args.value("dy_mm", 0.0);
	p.dzMm = args.value("dz_mm", 0.0);
	if (args.contains("source_feature_id") && args["source_feature_id"].is_string())
		p.sourceFeatureIdUtf8 = args["source_feature_id"].get<std::string>();

	PluginGeometryJobResult job;
	if (!waitGeomJobResult([&](PluginGeometryFinishedFn cb) { geo->linearPatternBodyToBrep(doc, p, std::move(cb)); },
						   &job, outError))
		return false;

	if (!stepId.empty())
		stepIdToBackendId[QString::fromStdString(stepId)] = target;
	return true;
}
} // namespace

namespace AiFeatureComposeSteps
{
bool tryExecute(PluginHostContext& host, const std::string& api, const nlohmann::json& args, const std::string& stepId,
				QHash<QString, QString>& stepIdToBackendId, QString* outError)
{
	if (outError)
		outError->clear();

	if (api == "extrudeSketchProfileToBrep")
	{
		if (!executeExtrude(host, args, stepId, stepIdToBackendId, outError))
		{
			if (outError && outError->isEmpty())
				*outError = QStringLiteral("extrudeSketchProfileToBrep failed");
		}
		return true;
	}
	if (api == "filletEdgesToBrep")
	{
		if (!executeFillet(host, args, stepId, stepIdToBackendId, outError))
		{
			if (outError && outError->isEmpty())
				*outError = QStringLiteral("filletEdgesToBrep failed");
		}
		return true;
	}
	if (api == "linearPatternBodyToBrep")
	{
		if (!executePattern(host, args, stepId, stepIdToBackendId, outError))
		{
			if (outError && outError->isEmpty())
				*outError = QStringLiteral("linearPatternBodyToBrep failed");
		}
		return true;
	}
	return false;
}
} // namespace AiFeatureComposeSteps
