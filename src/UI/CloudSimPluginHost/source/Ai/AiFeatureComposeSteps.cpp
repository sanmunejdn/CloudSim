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
#include <string>
#include <vector>

namespace
{
constexpr double kPi = 3.14159265358979323846;

// 与 SketchDocument2d::toJsonUtf8 / SkConstraintKind 对齐
constexpr int kSkHorizontal = 1;
constexpr int kSkVertical = 2;
constexpr int kSkDistance = 4;
constexpr int kSkRadius = 7;

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

std::vector<float> closedCircleXy(double cx, double cy, double radiusMm, int segments = 32)
{
	const int n = segments < 8 ? 8 : segments;
	std::vector<float> poly;
	poly.reserve(static_cast<size_t>((n + 1) * 3));
	for (int i = 0; i <= n; ++i)
	{
		const double a = (2.0 * kPi * (i % n)) / n;
		poly.push_back(static_cast<float>(cx + radiusMm * std::cos(a)));
		poly.push_back(static_cast<float>(cy + radiusMm * std::sin(a)));
		poly.push_back(0.f);
	}
	return poly;
}

nlohmann::json emptyGeomArrays()
{
	return nlohmann::json{{"arcs", nlohmann::json::array()},
						  {"ellipses", nlohmann::json::array()},
						  {"splines", nlohmann::json::array()}};
}

std::string sketchJsonRectangle(double lengthMm, double widthMm)
{
	// 固定原点角点，边长用 Distance，双击草图可改尺寸后 rebuild
	nlohmann::json root = emptyGeomArrays();
	root["seq"] = 9;
	root["points"] = nlohmann::json::array(
		{{{ "id", 1 }, { "u", 0.0 }, { "v", 0.0 }, { "fixed", true }},
		 {{ "id", 2 }, { "u", lengthMm }, { "v", 0.0 }, { "fixed", false }},
		 {{ "id", 3 }, { "u", lengthMm }, { "v", widthMm }, { "fixed", false }},
		 {{ "id", 4 }, { "u", 0.0 }, { "v", widthMm }, { "fixed", false }}});
	root["lines"] = nlohmann::json::array({{{ "id", 5 }, { "p1", 1 }, { "p2", 2 }, { "construction", false }},
										   {{ "id", 6 }, { "p1", 2 }, { "p2", 3 }, { "construction", false }},
										   {{ "id", 7 }, { "p1", 3 }, { "p2", 4 }, { "construction", false }},
										   {{ "id", 8 }, { "p1", 4 }, { "p2", 1 }, { "construction", false }}});
	root["circles"] = nlohmann::json::array();
	root["constraints"] = nlohmann::json::array(
		{{{ "kind", kSkHorizontal }, { "a", 5 }, { "b", -1 }, { "value", 0.0 }},
		 {{ "kind", kSkVertical }, { "a", 6 }, { "b", -1 }, { "value", 0.0 }},
		 {{ "kind", kSkHorizontal }, { "a", 7 }, { "b", -1 }, { "value", 0.0 }},
		 {{ "kind", kSkVertical }, { "a", 8 }, { "b", -1 }, { "value", 0.0 }},
		 {{ "kind", kSkDistance }, { "a", 1 }, { "b", 2 }, { "value", lengthMm }},
		 {{ "kind", kSkDistance }, { "a", 1 }, { "b", 4 }, { "value", widthMm }}});
	return root.dump();
}

std::string sketchJsonPolygon(int sides, double radiusMm)
{
	const int n = sides < 3 ? 3 : (sides > 24 ? 24 : sides);
	nlohmann::json root = emptyGeomArrays();
	nlohmann::json points = nlohmann::json::array();
	nlohmann::json lines = nlohmann::json::array();
	int nextId = 1;
	std::vector<int> pids;
	pids.reserve(static_cast<size_t>(n));
	for (int i = 0; i < n; ++i)
	{
		const double a = (2.0 * kPi * i) / n;
		const int id = nextId++;
		pids.push_back(id);
		points.push_back({{ "id", id },
						  { "u", radiusMm * std::cos(a) },
						  { "v", radiusMm * std::sin(a) },
						  { "fixed", i == 0 }});
	}
	std::vector<int> lids;
	lids.reserve(static_cast<size_t>(n));
	for (int i = 0; i < n; ++i)
	{
		const int id = nextId++;
		lids.push_back(id);
		lines.push_back({{ "id", id },
						 { "p1", pids[static_cast<size_t>(i)] },
						 { "p2", pids[static_cast<size_t>((i + 1) % n)] },
						 { "construction", false }});
	}
	root["seq"] = nextId;
	root["points"] = std::move(points);
	root["lines"] = std::move(lines);
	root["circles"] = nlohmann::json::array();
	root["constraints"] = nlohmann::json::array();
	return root.dump();
}

std::string sketchJsonCircle(double cx, double cy, double radiusMm)
{
	nlohmann::json root = emptyGeomArrays();
	root["seq"] = 3;
	root["points"] = nlohmann::json::array(
		{{{ "id", 1 }, { "u", cx }, { "v", cy }, { "fixed", true }}});
	root["lines"] = nlohmann::json::array();
	root["circles"] = nlohmann::json::array(
		{{{ "id", 2 }, { "center", 1 }, { "radius", radiusMm }, { "construction", false }}});
	root["constraints"] = nlohmann::json::array(
		{{{ "kind", kSkRadius }, { "a", 2 }, { "b", -1 }, { "value", radiusMm }}});
	return root.dump();
}

std::string sketchJsonFromPolylineXy(const std::vector<float>& xyz)
{
	if (xyz.size() < 12)
		return {};
	const size_t nVerts = xyz.size() / 3;
	// 末点若与首点重合则去掉闭合点
	size_t n = nVerts;
	if (n >= 2)
	{
		const size_t last = n - 1;
		const double dx = xyz[last * 3] - xyz[0];
		const double dy = xyz[last * 3 + 1] - xyz[1];
		if (dx * dx + dy * dy < 1e-12)
			n = last;
	}
	if (n < 3)
		return {};

	nlohmann::json root = emptyGeomArrays();
	nlohmann::json points = nlohmann::json::array();
	nlohmann::json lines = nlohmann::json::array();
	int nextId = 1;
	std::vector<int> pids;
	pids.reserve(n);
	for (size_t i = 0; i < n; ++i)
	{
		const int id = nextId++;
		pids.push_back(id);
		points.push_back({{ "id", id },
						  { "u", xyz[i * 3] },
						  { "v", xyz[i * 3 + 1] },
						  { "fixed", i == 0 }});
	}
	for (size_t i = 0; i < n; ++i)
	{
		const int id = nextId++;
		lines.push_back({{ "id", id },
						 { "p1", pids[i] },
						 { "p2", pids[(i + 1) % n] },
						 { "construction", false }});
	}
	root["seq"] = nextId;
	root["points"] = std::move(points);
	root["lines"] = std::move(lines);
	root["circles"] = nlohmann::json::array();
	root["constraints"] = nlohmann::json::array();
	return root.dump();
}

struct ProfileBuild
{
	std::vector<float> polyline;
	std::string sketchJson;
};

bool parseProfile(const nlohmann::json& args, ProfileBuild& out, QString* outError)
{
	out = {};
	if (args.contains("profile_xyz_mm") && args["profile_xyz_mm"].is_array())
	{
		for (const auto& v : args["profile_xyz_mm"])
		{
			if (v.is_number())
				out.polyline.push_back(static_cast<float>(v.get<double>()));
		}
		if (out.polyline.size() < 12)
		{
			if (outError)
				*outError = QStringLiteral("profile_xyz_mm too short");
			return false;
		}
		out.sketchJson = sketchJsonFromPolylineXy(out.polyline);
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
		out.polyline = closedRectXy(L, W);
		out.sketchJson = sketchJsonRectangle(L, W);
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
		out.polyline = closedPolygonXy(sides, R);
		out.sketchJson = sketchJsonPolygon(sides, R);
		return true;
	}
	if (profile == "circle" || profile == "disk")
	{
		double R = args.value("radius_mm", 0.0);
		if (R <= 0 && args.contains("diameter_mm") && args["diameter_mm"].is_number())
			R = args["diameter_mm"].get<double>() * 0.5;
		if (R <= 0 && args.contains("dimensions_mm") && args["dimensions_mm"].is_object())
		{
			const auto& d = args["dimensions_mm"];
			if (d.contains("radius"))
				R = d.value("radius", 0.0);
			else if (d.contains("diameter"))
				R = d.value("diameter", 0.0) * 0.5;
		}
		double cx = args.value("center_u_mm", args.value("center_x_mm", 0.0));
		double cy = args.value("center_v_mm", args.value("center_y_mm", 0.0));
		if (R <= 0)
		{
			if (outError)
				*outError = QStringLiteral("circle needs radius_mm or diameter_mm > 0");
			return false;
		}
		out.polyline = closedCircleXy(cx, cy, R);
		out.sketchJson = sketchJsonCircle(cx, cy, R);
		return true;
	}

	if (outError)
		*outError = QStringLiteral("Unknown profile; use rectangle|polygon|circle or profile_xyz_mm");
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

PluginSketchExtrudeEnd parseEndCondition(const nlohmann::json& args)
{
	const std::string ec = args.value("end_condition", args.value("endCondition", "blind"));
	if (ec == "through_all" || ec == "ThroughAll" || ec == "throughAll")
		return PluginSketchExtrudeEnd::ThroughAll;
	if (ec == "mid_plane" || ec == "MidPlane")
		return PluginSketchExtrudeEnd::MidPlane;
	return PluginSketchExtrudeEnd::Blind;
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

	ProfileBuild profile;
	if (!parseProfile(args, profile, outError))
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
	p.endCondition = parseEndCondition(args);
	if (args.contains("name") && args["name"].is_string())
		p.resultNameUtf8 = args["name"].get<std::string>();
	if (!profile.sketchJson.empty())
		p.sketchDocumentJsonUtf8 = profile.sketchJson;

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
			{ geo->extrudeSketchProfileToBrep(doc, profile.polyline, defaultXyPlane(), p, std::move(cb)); },
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

bool executeChamfer(PluginHostContext& host, const nlohmann::json& args, const std::string& stepId,
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

	PluginSketchChamferParams p;
	p.targetParametricBackendIdUtf8 = target.toStdString();
	p.distanceMm = args.value("distance_mm", args.value("chamfer_mm", 1.0));
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
			*outError = QStringLiteral("chamfer needs edge_indices or edges=all");
		return false;
	}

	PluginGeometryJobResult job;
	if (!waitGeomJobResult([&](PluginGeometryFinishedFn cb) { geo->chamferEdgesToBrep(doc, p, std::move(cb)); }, &job,
						   outError))
		return false;

	if (!stepId.empty())
		stepIdToBackendId[QString::fromStdString(stepId)] = target;
	return true;
}

bool executeRevolve(PluginHostContext& host, const nlohmann::json& args, const std::string& stepId,
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

	ProfileBuild profile;
	if (!parseProfile(args, profile, outError))
		return false;

	PluginSketchRevolveParams p;
	p.mode = (args.value("mode", "boss") == "cut") ? PluginSketchRevolveMode::Cut : PluginSketchRevolveMode::Boss;
	p.angleDeg = args.value("angle_deg", 360.0);
	p.plane = defaultXyPlane();
	// MVP：默认绕草图原点 +Y（与 UI 旋转体一致）
	p.axisOx = args.value("axis_ox", 0.0);
	p.axisOy = args.value("axis_oy", 0.0);
	p.axisOz = args.value("axis_oz", 0.0);
	p.axisDx = args.value("axis_dx", 0.0);
	p.axisDy = args.value("axis_dy", 1.0);
	p.axisDz = args.value("axis_dz", 0.0);
	if (args.contains("name") && args["name"].is_string())
		p.resultNameUtf8 = args["name"].get<std::string>();
	if (!profile.sketchJson.empty())
		p.sketchDocumentJsonUtf8 = profile.sketchJson;

	const QString target = resolveTarget(args, stepIdToBackendId, outError);
	if (p.mode == PluginSketchRevolveMode::Cut)
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
			{ geo->revolveSketchProfileToBrep(doc, profile.polyline, p, std::move(cb)); },
			&job, outError))
		return false;

	if (!stepId.empty() && !job.newBackendId.empty())
		stepIdToBackendId[QString::fromStdString(stepId)] = QString::fromStdString(job.newBackendId);
	else if (!stepId.empty() && !target.isEmpty())
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
	if (api == "chamferEdgesToBrep")
	{
		if (!executeChamfer(host, args, stepId, stepIdToBackendId, outError))
		{
			if (outError && outError->isEmpty())
				*outError = QStringLiteral("chamferEdgesToBrep failed");
		}
		return true;
	}
	if (api == "revolveSketchProfileToBrep")
	{
		if (!executeRevolve(host, args, stepId, stepIdToBackendId, outError))
		{
			if (outError && outError->isEmpty())
				*outError = QStringLiteral("revolveSketchProfileToBrep failed");
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
