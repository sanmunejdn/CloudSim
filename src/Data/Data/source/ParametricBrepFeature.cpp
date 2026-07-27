/// @file ParametricBrepFeature.cpp

#include "ParametricBrepFeature.h"

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
			if (s.kind == 1)
				so["m"] = nlohmann::json::array({s.mx, s.my, s.mz});
			arr.push_back(std::move(so));
		}
		o["pathSegments"] = std::move(arr);
	}
	o["lengthMm"] = f.lengthMm;
	o["draftAngleDeg"] = f.draftAngleDeg;
	o["reversed"] = f.reversed;
	o["endCondition"] = parametricExtrudeEndToString(f.endCondition);
	if (f.hasUpToFacePlane)
		o["upToFacePlane"] = planeToJson(f.upToFacePlane);
	if (!f.upToFaceBackendId.empty())
		o["upToFaceBackendId"] = f.upToFaceBackendId;
	if (f.upToFaceIndex >= 0)
		o["upToFaceIndex"] = f.upToFaceIndex;
	o["sketchRefId"] = f.sketchRefId;
	if (!f.pathSketchRefId.empty())
		o["pathSketchRefId"] = f.pathSketchRefId;
	o["suppressed"] = f.suppressed;
	o["visible"] = f.visible;
	if (!f.sketchDocumentJson.empty())
		o["sketchDocument"] = f.sketchDocumentJson;
	return o;
}

bool parametricFeatureFromJson(const nlohmann::json& o, ParametricFeature& out)
{
	if (!o.is_object())
		return false;
	out.id = o.value("id", std::string());
	out.name = o.value("name", out.id);
	out.kind = parametricFeatureKindFromString(o.value("kind", std::string("Sketch")));
	if (o.contains("plane") && o["plane"].is_object())
		planeFromJson(o["plane"], out.plane);
	out.profileXyzMm.clear();
	if (o.contains("profile") && o["profile"].is_array())
	{
		for (const auto& v : o["profile"])
			out.profileXyzMm.push_back(v.get<float>());
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
	out.lengthMm = o.value("lengthMm", 10.0);
	out.draftAngleDeg = o.value("draftAngleDeg", 0.0);
	out.reversed = o.value("reversed", false);
	out.endCondition = parametricExtrudeEndFromString(o.value("endCondition", std::string("Blind")));
	out.hasUpToFacePlane = false;
	if (o.contains("upToFacePlane") && o["upToFacePlane"].is_object())
	{
		planeFromJson(o["upToFacePlane"], out.upToFacePlane);
		out.hasUpToFacePlane = true;
	}
	out.upToFaceBackendId = o.value("upToFaceBackendId", std::string());
	out.upToFaceIndex = o.value("upToFaceIndex", -1);
	out.sketchRefId = o.value("sketchRefId", std::string());
	out.pathSketchRefId = o.value("pathSketchRefId", std::string());
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
	return !out.id.empty();
}
