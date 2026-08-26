/// @file ParametricBrepFeature.cpp

#include "ParametricBrepFeature.h"

#include "RunLogger.h"

ParametricEnumParse<ParametricFeatureKind> parametricFeatureKindTryParse(const std::string& s)
{
	if (s == "Sketch")
		return {ParametricFeatureKind::Sketch, true};
	if (s == "Pad")
		return {ParametricFeatureKind::Pad, true};
	if (s == "Pocket")
		return {ParametricFeatureKind::Pocket, true};
	if (s == "Sweep")
		return {ParametricFeatureKind::Sweep, true};
	if (s == "SweepCut")
		return {ParametricFeatureKind::SweepCut, true};
	if (s == "Fillet")
		return {ParametricFeatureKind::Fillet, true};
	if (s == "Chamfer")
		return {ParametricFeatureKind::Chamfer, true};
	if (s == "Revolve")
		return {ParametricFeatureKind::Revolve, true};
	if (s == "RevolveCut")
		return {ParametricFeatureKind::RevolveCut, true};
	if (s == "LinearPattern")
		return {ParametricFeatureKind::LinearPattern, true};
	if (s == "CircularPattern")
		return {ParametricFeatureKind::CircularPattern, true};
	if (s == "Mirror3D")
		return {ParametricFeatureKind::Mirror3D, true};
	if (s == "Loft")
		return {ParametricFeatureKind::Loft, true};
	if (s == "LoftCut")
		return {ParametricFeatureKind::LoftCut, true};
	if (s == "Shell")
		return {ParametricFeatureKind::Shell, true};
	if (s == "Draft")
		return {ParametricFeatureKind::Draft, true};
	return {ParametricFeatureKind::Sketch, false};
}

ParametricEnumParse<ParametricExtrudeEnd> parametricExtrudeEndTryParse(const std::string& s)
{
	if (s == "Blind")
		return {ParametricExtrudeEnd::Blind, true};
	if (s == "UpToFace")
		return {ParametricExtrudeEnd::UpToFace, true};
	if (s == "MidPlane")
		return {ParametricExtrudeEnd::MidPlane, true};
	if (s == "ThroughAll")
		return {ParametricExtrudeEnd::ThroughAll, true};
	if (s == "UpToVertex")
		return {ParametricExtrudeEnd::UpToVertex, true};
	if (s == "OffsetFromFace")
		return {ParametricExtrudeEnd::OffsetFromFace, true};
	if (s == "TwoDirections")
		return {ParametricExtrudeEnd::TwoDirections, true};
	return {ParametricExtrudeEnd::Blind, false};
}

namespace
{
nlohmann::json planeToJson(const ParametricSketchPlane& p)
{
	nlohmann::json o;
	o["origin"] = nlohmann::json::array({p.originX, p.originY, p.originZ});
	o["axisX"] = nlohmann::json::array({p.axisXX, p.axisXY, p.axisXZ});
	o["axisY"] = nlohmann::json::array({p.axisYX, p.axisYY, p.axisYZ});
	o["normal"] = nlohmann::json::array({p.normalX, p.normalY, p.normalZ});
	o["isPlanar"] = p.isPlanar;
	return o;
}

void planeFromJson(const nlohmann::json& o, ParametricSketchPlane& p)
{
	auto read3 = [](const nlohmann::json& a, double& x, double& y, double& z)
	{
		if (!a.is_array() || a.size() < 3)
			return;
		x = a[0].get<double>();
		y = a[1].get<double>();
		z = a[2].get<double>();
	};
	if (o.contains("origin"))
		read3(o["origin"], p.originX, p.originY, p.originZ);
	if (o.contains("axisX"))
		read3(o["axisX"], p.axisXX, p.axisXY, p.axisXZ);
	if (o.contains("axisY"))
		read3(o["axisY"], p.axisYX, p.axisYY, p.axisYZ);
	if (o.contains("normal"))
		read3(o["normal"], p.normalX, p.normalY, p.normalZ);
	p.isPlanar = o.value("isPlanar", false);
}
} // namespace

nlohmann::json parametricFeatureToJson(const ParametricFeature& f)
{
	nlohmann::json o;
	o["id"] = f.id;
	o["name"] = f.name;
	o["kind"] = parametricFeatureKindToString(f.kind);
	o["plane"] = planeToJson(f.plane);
	o["profile"] = f.profileXyzMm;
	if (!f.profileHolesXyzMm.empty())
	{
		nlohmann::json holes = nlohmann::json::array();
		for (const auto& h : f.profileHolesXyzMm)
			holes.push_back(h);
		o["profileHoles"] = std::move(holes);
	}
	if (!f.pathXyzMm.empty())
		o["path"] = f.pathXyzMm;
	if (!f.pathSegments.empty())
	{
		nlohmann::json arr = nlohmann::json::array();
		for (const auto& s : f.pathSegments)
		{
			nlohmann::json so;
			so["kind"] = s.kind;
			so["a"] = nlohmann::json::array({s.ax, s.ay, s.az});
			so["b"] = nlohmann::json::array({s.bx, s.by, s.bz});
			if (s.kind == 1 || s.kind == 3 || s.kind == 4)
				so["m"] = nlohmann::json::array({s.mx, s.my, s.mz});
			arr.push_back(std::move(so));
		}
		o["pathSegments"] = std::move(arr);
	}
	if (!f.profileSegments.empty())
	{
		nlohmann::json arr = nlohmann::json::array();
		for (const auto& s : f.profileSegments)
		{
			nlohmann::json so;
			so["kind"] = s.kind;
			so["a"] = nlohmann::json::array({s.ax, s.ay, s.az});
			so["b"] = nlohmann::json::array({s.bx, s.by, s.bz});
			if (s.kind == 1 || s.kind == 3 || s.kind == 4)
				so["m"] = nlohmann::json::array({s.mx, s.my, s.mz});
			arr.push_back(std::move(so));
		}
		o["profileSegments"] = std::move(arr);
	}
	if (std::abs(f.twistDeg) > 1e-9)
		o["twistDeg"] = f.twistDeg;
	o["lengthMm"] = f.lengthMm;
	if (std::abs(f.length2Mm) > 1e-9)
		o["length2Mm"] = f.length2Mm;
	if (std::abs(f.startOffsetMm) > 1e-9)
		o["startOffsetMm"] = f.startOffsetMm;
	o["draftAngleDeg"] = f.draftAngleDeg;
	o["reversed"] = f.reversed;
	o["endCondition"] = parametricExtrudeEndToString(f.endCondition);
	if (f.hasUpToFacePlane)
		o["upToFacePlane"] = planeToJson(f.upToFacePlane);
	if (!f.upToFaceBackendId.empty())
		o["upToFaceBackendId"] = f.upToFaceBackendId;
	if (f.upToFaceIndex >= 0)
		o["upToFaceIndex"] = f.upToFaceIndex;
	if (f.hasUpToVertex)
	{
		o["upToVertex"] = nlohmann::json::array({f.upToVertexX, f.upToVertexY, f.upToVertexZ});
		if (f.upToVertexIndex >= 0)
			o["upToVertexIndex"] = f.upToVertexIndex;
	}
	if (std::abs(f.offsetFromFaceMm) > 1e-9)
		o["offsetFromFaceMm"] = f.offsetFromFaceMm;
	o["sketchRefId"] = f.sketchRefId;
	if (!f.pathSketchRefId.empty())
		o["pathSketchRefId"] = f.pathSketchRefId;
	if (!f.loftSketchRefId.empty())
		o["loftSketchRefId"] = f.loftSketchRefId;
	o["suppressed"] = f.suppressed;
	o["visible"] = f.visible;
	if (!f.sketchDocumentJson.empty())
		o["sketchDocument"] = f.sketchDocumentJson;
	if (!f.edgeIndices.empty())
		o["edgeIndices"] = f.edgeIndices;
	if (!f.faceIndices.empty())
		o["faceIndices"] = f.faceIndices;
	o["radiusMm"] = f.radiusMm;
	o["chamferDistMm"] = f.chamferDistMm;
	o["shellThicknessMm"] = f.shellThicknessMm;
	o["revolveAngleDeg"] = f.revolveAngleDeg;
	o["axisO"] = nlohmann::json::array({f.axisOx, f.axisOy, f.axisOz});
	o["axisD"] = nlohmann::json::array({f.axisDx, f.axisDy, f.axisDz});
	o["patternCount"] = f.patternCount;
	o["patternD"] = nlohmann::json::array({f.patternDx, f.patternDy, f.patternDz});
	o["patternAngleDeg"] = f.patternAngleDeg;
	if (!f.patternSourceFeatureId.empty())
		o["patternSourceFeatureId"] = f.patternSourceFeatureId;
	o["mirrorPlane"] = planeToJson(f.mirrorPlane);
	o["mirrorKeepOriginal"] = f.mirrorKeepOriginal;
	return o;
}

bool parametricFeatureFromJson(const nlohmann::json& o, ParametricFeature& out)
{
	if (!o.is_object())
		return false;
	out.id = o.value("id", std::string());
	out.name = o.value("name", out.id);
	// 未知 kind 不再静默回退 Sketch：回退会悄悄改写特征链且加载成功，必须告警
	{
		const std::string kindStr = o.value("kind", std::string("Sketch"));
		const auto parsed = parametricFeatureKindTryParse(kindStr);
		if (!parsed.ok)
		{
			RunLogger::warn("[ParametricFeature] unknown feature kind \"" + kindStr +
							"\", fallback to Sketch. Project file may be corrupt or from a newer version.");
		}
		out.kind = parsed.value;
	}
	if (o.contains("plane") && o["plane"].is_object())
		planeFromJson(o["plane"], out.plane);
	out.profileXyzMm.clear();
	if (o.contains("profile") && o["profile"].is_array())
	{
		for (const auto& v : o["profile"])
			out.profileXyzMm.push_back(v.get<float>());
	}
	out.profileHolesXyzMm.clear();
	if (o.contains("profileHoles") && o["profileHoles"].is_array())
	{
		for (const auto& hole : o["profileHoles"])
		{
			if (!hole.is_array())
				continue;
			std::vector<float> h;
			for (const auto& v : hole)
				h.push_back(v.get<float>());
			if (h.size() >= 12)
				out.profileHolesXyzMm.push_back(std::move(h));
		}
	}
	out.pathXyzMm.clear();
	if (o.contains("path") && o["path"].is_array())
	{
		for (const auto& v : o["path"])
			out.pathXyzMm.push_back(v.get<float>());
	}
	out.pathSegments.clear();
	if (o.contains("pathSegments") && o["pathSegments"].is_array())
	{
		for (const auto& so : o["pathSegments"])
		{
			if (!so.is_object())
				continue;
			ParametricFeature::PathSegment s;
			s.kind = so.value("kind", 0);
			auto read3 = [](const nlohmann::json& a, float& x, float& y, float& z)
			{
				if (!a.is_array() || a.size() < 3)
					return;
				x = a[0].get<float>();
				y = a[1].get<float>();
				z = a[2].get<float>();
			};
			if (so.contains("a"))
				read3(so["a"], s.ax, s.ay, s.az);
			if (so.contains("b"))
				read3(so["b"], s.bx, s.by, s.bz);
			if (so.contains("m"))
				read3(so["m"], s.mx, s.my, s.mz);
			out.pathSegments.push_back(s);
		}
	}
	out.profileSegments.clear();
	if (o.contains("profileSegments") && o["profileSegments"].is_array())
	{
		for (const auto& so : o["profileSegments"])
		{
			if (!so.is_object())
				continue;
			ParametricFeature::PathSegment s;
			s.kind = so.value("kind", 0);
			auto read3 = [](const nlohmann::json& a, float& x, float& y, float& z)
			{
				if (!a.is_array() || a.size() < 3)
					return;
				x = a[0].get<float>();
				y = a[1].get<float>();
				z = a[2].get<float>();
			};
			if (so.contains("a"))
				read3(so["a"], s.ax, s.ay, s.az);
			if (so.contains("b"))
				read3(so["b"], s.bx, s.by, s.bz);
			if (so.contains("m"))
				read3(so["m"], s.mx, s.my, s.mz);
			out.profileSegments.push_back(s);
		}
	}
	out.twistDeg = o.value("twistDeg", 0.0);
	out.lengthMm = o.value("lengthMm", 10.0);
	out.length2Mm = o.value("length2Mm", 0.0);
	out.startOffsetMm = o.value("startOffsetMm", 0.0);
	out.draftAngleDeg = o.value("draftAngleDeg", 0.0);
	out.reversed = o.value("reversed", false);
	{
		const std::string endStr = o.value("endCondition", std::string("Blind"));
		const auto parsed = parametricExtrudeEndTryParse(endStr);
		if (!parsed.ok)
		{
			RunLogger::warn("[ParametricFeature] unknown endCondition \"" + endStr +
							"\", fallback to Blind. Project file may be corrupt or from a newer version.");
		}
		out.endCondition = parsed.value;
	}
	out.hasUpToFacePlane = false;
	if (o.contains("upToFacePlane") && o["upToFacePlane"].is_object())
	{
		planeFromJson(o["upToFacePlane"], out.upToFacePlane);
		out.hasUpToFacePlane = true;
	}
	out.upToFaceBackendId = o.value("upToFaceBackendId", std::string());
	out.upToFaceIndex = o.value("upToFaceIndex", -1);
	out.hasUpToVertex = false;
	out.upToVertexIndex = -1;
	if (o.contains("upToVertex") && o["upToVertex"].is_array() && o["upToVertex"].size() >= 3)
	{
		out.upToVertexX = o["upToVertex"][0].get<double>();
		out.upToVertexY = o["upToVertex"][1].get<double>();
		out.upToVertexZ = o["upToVertex"][2].get<double>();
		out.hasUpToVertex = true;
	}
	out.upToVertexIndex = o.value("upToVertexIndex", -1);
	out.offsetFromFaceMm = o.value("offsetFromFaceMm", 0.0);
	out.sketchRefId = o.value("sketchRefId", std::string());
	out.pathSketchRefId = o.value("pathSketchRefId", std::string());
	out.loftSketchRefId = o.value("loftSketchRefId", std::string());
	out.suppressed = o.value("suppressed", false);
	out.visible = o.value("visible", true);
	out.sketchDocumentJson.clear();
	if (o.contains("sketchDocument"))
	{
		if (o["sketchDocument"].is_string())
			out.sketchDocumentJson = o["sketchDocument"].get<std::string>();
		else
			out.sketchDocumentJson = o["sketchDocument"].dump();
	}
	out.edgeIndices.clear();
	if (o.contains("edgeIndices") && o["edgeIndices"].is_array())
	{
		for (const auto& v : o["edgeIndices"])
			out.edgeIndices.push_back(v.get<int>());
	}
	out.faceIndices.clear();
	if (o.contains("faceIndices") && o["faceIndices"].is_array())
	{
		for (const auto& v : o["faceIndices"])
			out.faceIndices.push_back(v.get<int>());
	}
	out.radiusMm = o.value("radiusMm", 1.0);
	out.chamferDistMm = o.value("chamferDistMm", 1.0);
	out.shellThicknessMm = o.value("shellThicknessMm", 1.0);
	out.revolveAngleDeg = o.value("revolveAngleDeg", 360.0);
	auto read3d = [](const nlohmann::json& a, double& x, double& y, double& z)
	{
		if (!a.is_array() || a.size() < 3)
			return;
		x = a[0].get<double>();
		y = a[1].get<double>();
		z = a[2].get<double>();
	};
	if (o.contains("axisO"))
		read3d(o["axisO"], out.axisOx, out.axisOy, out.axisOz);
	if (o.contains("axisD"))
		read3d(o["axisD"], out.axisDx, out.axisDy, out.axisDz);
	out.patternCount = o.value("patternCount", 2);
	if (o.contains("patternD"))
		read3d(o["patternD"], out.patternDx, out.patternDy, out.patternDz);
	out.patternAngleDeg = o.value("patternAngleDeg", 360.0);
	out.patternSourceFeatureId = o.value("patternSourceFeatureId", std::string());
	if (o.contains("mirrorPlane") && o["mirrorPlane"].is_object())
		planeFromJson(o["mirrorPlane"], out.mirrorPlane);
	out.mirrorKeepOriginal = o.value("mirrorKeepOriginal", true);
	return !out.id.empty();
}
