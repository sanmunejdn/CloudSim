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

nlohmann::json profileArgsForKey(const nlohmann::json& args, const char* key)
{
	nlohmann::json sub = nlohmann::json::object();
	const std::string k = key;
	if (k.empty())
	{
		if (args.contains("profile_xyz_mm") && args["profile_xyz_mm"].is_array())
			sub["profile_xyz_mm"] = args["profile_xyz_mm"];
		sub["profile"] = args.value("profile", "rectangle");
		const char* keys[] = {"length_mm",	 "width_mm",	"radius_mm",	"diameter_mm",	"sides",
							  "center_u_mm", "center_v_mm", "center_x_mm", "center_y_mm"};
		for (const char* nk : keys)
		{
			if (args.contains(nk))
				sub[nk] = args[nk];
		}
		if (args.contains("size_mm"))
			sub["size_mm"] = args["size_mm"];
		if (args.contains("dimensions_mm"))
			sub["dimensions_mm"] = args["dimensions_mm"];
		return sub;
	}

	const std::string xyzKey = k + "_xyz_mm";
	if (args.contains(xyzKey) && args[xyzKey].is_array())
		sub["profile_xyz_mm"] = args[xyzKey];
	sub["profile"] = args.value(k, "rectangle");

	const auto copyNum = [&](const char* dst, const char* srcPrefixed)
	{
		if (args.contains(srcPrefixed))
			sub[dst] = args[srcPrefixed];
	};
	copyNum("length_mm", (k + "_length_mm").c_str());
	copyNum("width_mm", (k + "_width_mm").c_str());
	copyNum("radius_mm", (k + "_radius_mm").c_str());
	copyNum("diameter_mm", (k + "_diameter_mm").c_str());
	copyNum("sides", (k + "_sides").c_str());
	copyNum("center_u_mm", (k + "_center_u_mm").c_str());
	copyNum("center_v_mm", (k + "_center_v_mm").c_str());
	copyNum("center_x_mm", (k + "_center_x_mm").c_str());
	copyNum("center_y_mm", (k + "_center_y_mm").c_str());
	if (args.contains(k + "_size_mm"))
		sub["size_mm"] = args[k + "_size_mm"];
	if (args.contains(k + "_dimensions_mm"))
		sub["dimensions_mm"] = args[k + "_dimensions_mm"];
	return sub;
}

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

bool parseProfileKey(const nlohmann::json& args, const char* key, ProfileBuild& out, QString* outError)
{
	return parseProfile(profileArgsForKey(args, key), out, outError);
}

struct PathBuild
{
	std::vector<float> polyline;
	std::string sketchJson;
};

bool parsePath(const nlohmann::json& args, PathBuild& out, QString* outError)
{
	out = {};
	if (args.contains("path_xyz_mm") && args["path_xyz_mm"].is_array())
	{
		for (const auto& v : args["path_xyz_mm"])
		{
			if (v.is_number())
				out.polyline.push_back(static_cast<float>(v.get<double>()));
		}
		if (out.polyline.size() < 6)
		{
			if (outError)
				*outError = QStringLiteral("path_xyz_mm too short");
			return false;
		}
		out.sketchJson = sketchJsonFromPolylineXy(out.polyline);
		return true;
	}

	const std::string path = args.value("path", args.value("path_profile", "line_z"));
	if (path == "line_z" || path == "line")
	{
		const double dx = args.value("path_dx_mm", 0.0);
		const double dy = args.value("path_dy_mm", 0.0);
		double dz = args.value("path_dz_mm", 0.0);
		if (path == "line_z" && dz == 0.0)
			dz = args.value("path_length_mm", args.value("length_mm", 50.0));
		if (dx == 0.0 && dy == 0.0 && dz == 0.0)
		{
			if (outError)
				*outError = QStringLiteral("path needs path_length_mm or path_dx/dy/dz_mm");
			return false;
		}
		out.polyline = {0.f, 0.f, 0.f, static_cast<float>(dx), static_cast<float>(dy), static_cast<float>(dz)};
		out.sketchJson = sketchJsonFromPolylineXy(out.polyline);
		return true;
	}

	if (outError)
		*outError = QStringLiteral("Unknown path; use line_z|line or path_xyz_mm");
	return false;
}

bool parseFaceIndices(const nlohmann::json& args, std::vector<int>& out, QString* outError)
{
	out.clear();
	if (!args.contains("face_indices") || !args["face_indices"].is_array() || args["face_indices"].empty())
	{
		if (outError)
			*outError = QStringLiteral("face_indices int[] required");
		return false;
	}
	for (const auto& v : args["face_indices"])
	{
		if (v.is_number_integer())
			out.push_back(v.get<int>());
		else if (v.is_number())
			out.push_back(static_cast<int>(v.get<double>()));
	}
	return !out.empty();
}

PluginSketchPlane parseNeutralPlane(const nlohmann::json& args)
{
	PluginSketchPlane p;
	if (!args.contains("neutral_ox") && !args.contains("neutral_oy") && !args.contains("neutral_oz") &&
		!args.contains("neutral_nx") && !args.contains("neutral_ny") && !args.contains("neutral_nz"))
		return p;
	p.isPlanar = true;
	p.origin = {args.value("neutral_ox", 0.0), args.value("neutral_oy", 0.0), args.value("neutral_oz", 0.0)};
	p.normal = {args.value("neutral_nx", 0.0), args.value("neutral_ny", 0.0), args.value("neutral_nz", 1.0)};
	p.axisX = {1, 0, 0};
	p.axisY = {0, 1, 0};
	return p;
}

PluginSketchPlane xyPlaneAtZ(double z)
{
	PluginSketchPlane p = defaultXyPlane();
	p.origin = {0, 0, static_cast<float>(z)};
	return p;
}

/// 草图 UV 折线 → 世界坐标（与插件 loadSketchPolyline 对齐）
std::vector<float> polylineOnPlane(const std::vector<float>& uvz, const PluginSketchPlane& pl)
{
	std::vector<float> out;
	out.reserve(uvz.size());
	for (size_t i = 0; i + 2 < uvz.size(); i += 3)
	{
		const double u = uvz[i];
		const double v = uvz[i + 1];
		const double w = uvz[i + 2];
		out.push_back(static_cast<float>(pl.origin.x + pl.axisX.x * u + pl.axisY.x * v + pl.normal.x * w));
		out.push_back(static_cast<float>(pl.origin.y + pl.axisX.y * u + pl.axisY.y * v + pl.normal.y * w));
		out.push_back(static_cast<float>(pl.origin.z + pl.axisX.z * u + pl.axisY.z * v + pl.normal.z * w));
	}
	return out;
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
	if (!parseProfileKey(args, "", profile, outError))
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
	const std::string edges = args.value("edges", "longest");
	p.allEdges = (edges == "all");
	if (edges == "longest" || edges == "top_boundary")
	{
		p.edgeSelectUtf8 = edges;
		p.edgeSelectCount = args.value("edge_count", 4);
	}
	if (args.contains("edge_indices") && args["edge_indices"].is_array())
	{
		for (const auto& v : args["edge_indices"])
		{
			if (v.is_number_integer())
				p.edgeIndices.push_back(v.get<int>());
			else if (v.is_number())
				p.edgeIndices.push_back(static_cast<int>(v.get<double>()));
		}
		p.edgeSelectUtf8.clear();
	}
	if (p.edgeIndices.empty() && !p.allEdges && p.edgeSelectUtf8.empty())
	{
		if (outError)
			*outError = QStringLiteral("fillet needs edge_indices or edges=all|longest|top_boundary");
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
	const std::string edges = args.value("edges", "longest");
	p.allEdges = (edges == "all");
	if (edges == "longest" || edges == "top_boundary")
	{
		p.edgeSelectUtf8 = edges;
		p.edgeSelectCount = args.value("edge_count", 4);
	}
	if (args.contains("edge_indices") && args["edge_indices"].is_array())
	{
		for (const auto& v : args["edge_indices"])
		{
			if (v.is_number_integer())
				p.edgeIndices.push_back(v.get<int>());
			else if (v.is_number())
				p.edgeIndices.push_back(static_cast<int>(v.get<double>()));
		}
		p.edgeSelectUtf8.clear();
	}
	if (p.edgeIndices.empty() && !p.allEdges && p.edgeSelectUtf8.empty())
	{
		if (outError)
			*outError = QStringLiteral("chamfer needs edge_indices or edges=all|longest|top_boundary");
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
	if (!parseProfileKey(args, "", profile, outError))
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

bool executeSweep(PluginHostContext& host, const nlohmann::json& args, const std::string& stepId,
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
	if (!parseProfileKey(args, "", profile, outError))
		return false;
	PathBuild path;
	if (!parsePath(args, path, outError))
		return false;

	PluginSketchSweepParams p;
	p.mode = (args.value("mode", "boss") == "cut") ? PluginSketchSweepMode::Cut : PluginSketchSweepMode::Boss;
	p.profilePlane = defaultXyPlane();
	p.pathPlane = defaultXyPlane();
	p.twistDeg = args.value("twist_deg", 0.0);
	if (args.contains("name") && args["name"].is_string())
		p.resultNameUtf8 = args["name"].get<std::string>();
	if (!profile.sketchJson.empty())
		p.profileSketchDocumentJsonUtf8 = profile.sketchJson;
	if (!path.sketchJson.empty())
		p.pathSketchDocumentJsonUtf8 = path.sketchJson;

	const QString target = resolveTarget(args, stepIdToBackendId, outError);
	if (p.mode == PluginSketchSweepMode::Cut)
	{
		if (target.isEmpty())
			return false;
		p.targetParametricBackendIdUtf8 = target.toStdString();
	}
	else if (!target.isEmpty())
		p.targetParametricBackendIdUtf8 = target.toStdString();

	PluginGeometryJobResult job;
	if (!waitGeomJobResult(
			[&](PluginGeometryFinishedFn cb)
			{ geo->sweepSketchProfileToBrep(doc, profile.polyline, path.polyline, p, std::move(cb)); },
			&job, outError))
		return false;

	if (!stepId.empty() && !job.newBackendId.empty())
		stepIdToBackendId[QString::fromStdString(stepId)] = QString::fromStdString(job.newBackendId);
	else if (!stepId.empty() && !target.isEmpty())
		stepIdToBackendId[QString::fromStdString(stepId)] = target;
	return true;
}

bool executeLoft(PluginHostContext& host, const nlohmann::json& args, const std::string& stepId,
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

	ProfileBuild profileA;
	ProfileBuild profileB;
	if (!parseProfileKey(args, "profile_a", profileA, outError))
		return false;
	if (!parseProfileKey(args, "profile_b", profileB, outError))
		return false;

	PluginSketchLoftParams p;
	p.mode = (args.value("mode", "boss") == "cut") ? PluginSketchLoftMode::Cut : PluginSketchLoftMode::Boss;
	p.planeA = xyPlaneAtZ(args.value("profile_a_z_mm", args.value("plane_a_oz", 0.0)));
	p.planeB = xyPlaneAtZ(args.value("profile_b_z_mm", args.value("plane_b_oz", 10.0)));
	if (args.contains("name") && args["name"].is_string())
		p.resultNameUtf8 = args["name"].get<std::string>();
	if (!profileA.sketchJson.empty())
		p.sketchADocumentJsonUtf8 = profileA.sketchJson;
	if (!profileB.sketchJson.empty())
		p.sketchBDocumentJsonUtf8 = profileB.sketchJson;

	const QString target = resolveTarget(args, stepIdToBackendId, outError);
	if (p.mode == PluginSketchLoftMode::Cut)
	{
		if (target.isEmpty())
			return false;
		p.targetParametricBackendIdUtf8 = target.toStdString();
	}
	else if (!target.isEmpty())
		p.targetParametricBackendIdUtf8 = target.toStdString();

	PluginGeometryJobResult job;
	if (!waitGeomJobResult(
			[&](PluginGeometryFinishedFn cb)
			{
				const std::vector<float> worldA = polylineOnPlane(profileA.polyline, p.planeA);
				const std::vector<float> worldB = polylineOnPlane(profileB.polyline, p.planeB);
				geo->loftSketchProfilesToBrep(doc, worldA, worldB, p, std::move(cb));
			},
			&job, outError))
		return false;

	if (!stepId.empty() && !job.newBackendId.empty())
		stepIdToBackendId[QString::fromStdString(stepId)] = QString::fromStdString(job.newBackendId);
	else if (!stepId.empty() && !target.isEmpty())
		stepIdToBackendId[QString::fromStdString(stepId)] = target;
	return true;
}

bool executeShell(PluginHostContext& host, const nlohmann::json& args, const std::string& stepId,
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

	PluginSketchShellParams p;
	p.targetParametricBackendIdUtf8 = target.toStdString();
	p.thicknessMm = args.value("thickness_mm", 1.0);
	if (!parseFaceIndices(args, p.faceIndices, outError))
		return false;

	PluginGeometryJobResult job;
	if (!waitGeomJobResult([&](PluginGeometryFinishedFn cb) { geo->shellFacesToBrep(doc, p, std::move(cb)); }, &job,
						   outError))
		return false;

	if (!stepId.empty())
		stepIdToBackendId[QString::fromStdString(stepId)] = target;
	return true;
}

bool executeDraft(PluginHostContext& host, const nlohmann::json& args, const std::string& stepId,
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

	PluginSketchDraftParams p;
	p.targetParametricBackendIdUtf8 = target.toStdString();
	p.angleDeg = args.value("angle_deg", 1.0);
	p.neutralPlane = parseNeutralPlane(args);
	if (!parseFaceIndices(args, p.faceIndices, outError))
		return false;

	PluginGeometryJobResult job;
	if (!waitGeomJobResult([&](PluginGeometryFinishedFn cb) { geo->draftFacesToBrep(doc, p, std::move(cb)); }, &job,
						   outError))
		return false;

	if (!stepId.empty())
		stepIdToBackendId[QString::fromStdString(stepId)] = target;
	return true;
}

bool executeCircularPattern(PluginHostContext& host, const nlohmann::json& args, const std::string& stepId,
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

	PluginSketchCircularPatternParams p;
	p.targetParametricBackendIdUtf8 = target.toStdString();
	p.count = args.value("count", 2);
	p.angleDeg = args.value("angle_deg", 360.0);
	p.axisOx = args.value("axis_ox", 0.0);
	p.axisOy = args.value("axis_oy", 0.0);
	p.axisOz = args.value("axis_oz", 0.0);
	p.axisDx = args.value("axis_dx", 0.0);
	p.axisDy = args.value("axis_dy", 0.0);
	p.axisDz = args.value("axis_dz", 1.0);
	if (args.contains("source_feature_id") && args["source_feature_id"].is_string())
		p.sourceFeatureIdUtf8 = args["source_feature_id"].get<std::string>();

	PluginGeometryJobResult job;
	if (!waitGeomJobResult(
			[&](PluginGeometryFinishedFn cb) { geo->circularPatternBodyToBrep(doc, p, std::move(cb)); }, &job,
			outError))
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
	if (api == "sweepSketchProfileToBrep")
	{
		if (!executeSweep(host, args, stepId, stepIdToBackendId, outError))
		{
			if (outError && outError->isEmpty())
				*outError = QStringLiteral("sweepSketchProfileToBrep failed");
		}
		return true;
	}
	if (api == "loftSketchProfilesToBrep")
	{
		if (!executeLoft(host, args, stepId, stepIdToBackendId, outError))
		{
			if (outError && outError->isEmpty())
				*outError = QStringLiteral("loftSketchProfilesToBrep failed");
		}
		return true;
	}
	if (api == "shellFacesToBrep")
	{
		if (!executeShell(host, args, stepId, stepIdToBackendId, outError))
		{
			if (outError && outError->isEmpty())
				*outError = QStringLiteral("shellFacesToBrep failed");
		}
		return true;
	}
	if (api == "draftFacesToBrep")
	{
		if (!executeDraft(host, args, stepId, stepIdToBackendId, outError))
		{
			if (outError && outError->isEmpty())
				*outError = QStringLiteral("draftFacesToBrep failed");
		}
		return true;
	}
	if (api == "circularPatternBodyToBrep")
	{
		if (!executeCircularPattern(host, args, stepId, stepIdToBackendId, outError))
		{
			if (outError && outError->isEmpty())
				*outError = QStringLiteral("circularPatternBodyToBrep failed");
		}
		return true;
	}
	return false;
}
} // namespace AiFeatureComposeSteps
