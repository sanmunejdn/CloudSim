/// @file FeatureDiscretize.cpp
/// @brief FeatureDiscretize 实现

#include "FeatureDiscretizerBridge.h"
#include "FeatureDiscretizerRegistry.h"
#include "ShapeHandle.h"
#include "ShapeIo.h"
#include "ShapeQuery.h"
#include "detail/OccIncludes.h"

#include <algorithm>
#include <cmath>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <json.hpp>

namespace geoalgo
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double edgeLengthMm(const TopoDS_Edge& edge)
{
	GProp_GProps props;
	BRepGProp::LinearProperties(edge, props);
	return props.Mass();
}

double faceAreaMm2(const TopoDS_Face& face)
{
	GProp_GProps props;
	BRepGProp::SurfaceProperties(face, props);
	return props.Mass();
}

double edgeDihedralDeg(const TopoDS_Shape& shape, const TopoDS_Edge& edge)
{
	TopTools_IndexedDataMapOfShapeListOfShape edgeFaceMap;
	TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edgeFaceMap);
	const TopTools_ListOfShape& faces = edgeFaceMap.FindFromKey(edge);
	if (faces.Extent() < 2)
	{
		return 0.0;
	}
	TopTools_ListIteratorOfListOfShape it(faces);
	const TopoDS_Face f1 = TopoDS::Face(it.Value());
	it.Next();
	const TopoDS_Face f2 = TopoDS::Face(it.Value());
	BRepAdaptor_Curve curve(edge);
	BRepAdaptor_Surface s1(f1);
	BRepAdaptor_Surface s2(f2);
	(void)s2;
	const double mid = (curve.FirstParameter() + curve.LastParameter()) * 0.5;
	gp_Pnt pm;
	gp_Vec tan;
	curve.D1(mid, pm, tan);
	if (tan.Magnitude() < 1e-12)
	{
		return 0.0;
	}
	gp_Vec n1;
	gp_Vec n2;
	{
		GeomAPI_ProjectPointOnSurf ps1(pm, BRep_Tool::Surface(f1));
		if (ps1.NbPoints() < 1)
		{
			return 0.0;
		}
		Standard_Real uu = 0.0;
		Standard_Real vv = 0.0;
		ps1.LowerDistanceParameters(uu, vv);
		gp_Pnt dummy;
		gp_Vec du;
		gp_Vec dv;
		s1.D1(uu, vv, dummy, du, dv);
		n1 = du.Crossed(dv);
		if (f1.Orientation() == TopAbs_REVERSED)
		{
			n1.Reverse();
		}
	}
	{
		GeomAPI_ProjectPointOnSurf ps2(pm, BRep_Tool::Surface(f2));
		if (ps2.NbPoints() < 1)
		{
			return 0.0;
		}
		Standard_Real uu = 0.0;
		Standard_Real vv = 0.0;
		ps2.LowerDistanceParameters(uu, vv);
		gp_Pnt dummy;
		gp_Vec du;
		gp_Vec dv;
		s2.D1(uu, vv, dummy, du, dv);
		n2 = du.Crossed(dv);
		if (f2.Orientation() == TopAbs_REVERSED)
		{
			n2.Reverse();
		}
	}
	if (n1.Magnitude() < 1e-12 || n2.Magnitude() < 1e-12)
	{
		return 0.0;
	}
	n1.Normalize();
	n2.Normalize();
	const double dot = std::max(-1.0, std::min(1.0, n1.Dot(n2)));
	return std::acos(dot) * 180.0 / kPi;
}

void writeWorkpieceJson(nlohmann::json& j, const WorkpieceRef& wp)
{
	j["backendIdUtf8"] = wp.backendIdUtf8;
	j["stepPathUtf8"] = wp.stepPathUtf8;
	if (!wp.frameId.empty() && wp.frameId != "workpiece")
	{
		j["frameId"] = wp.frameId;
	}
}

void writeGeometryJson(nlohmann::json& j, const FeatureGeometry& geometry)
{
	if (!geometry.edgeIndices.empty())
	{
		j["edgeIndices"] = geometry.edgeIndices;
	}
	if (!geometry.faceIndices.empty())
	{
		j["faceIndices"] = geometry.faceIndices;
	}
	if (!geometry.polylineXyz.empty())
	{
		j["polylineXyz"] = geometry.polylineXyz;
	}
}

bool readWorkpieceJson(const nlohmann::json& j, WorkpieceRef& wp, std::string* errMsg)
{
	if (!j.is_object())
	{
		if (errMsg)
		{
			*errMsg = "workpiece must be object";
		}
		return false;
	}
	wp.backendIdUtf8 = j.value("backendIdUtf8", "");
	wp.stepPathUtf8 = j.value("stepPathUtf8", "");
	wp.frameId = j.value("frameId", "workpiece");
	return true;
}

bool readGeometryJson(const nlohmann::json& j, FeatureGeometry& geometry)
{
	if (!j.is_object())
	{
		return true;
	}
	if (j.contains("edgeIndices") && j["edgeIndices"].is_array())
	{
		for (const auto& v : j["edgeIndices"])
		{
			geometry.edgeIndices.push_back(v.get<int>());
		}
	}
	if (j.contains("faceIndices") && j["faceIndices"].is_array())
	{
		for (const auto& v : j["faceIndices"])
		{
			geometry.faceIndices.push_back(v.get<int>());
		}
	}
	if (j.contains("polylineXyz") && j["polylineXyz"].is_array())
	{
		for (const auto& v : j["polylineXyz"])
		{
			geometry.polylineXyz.push_back(static_cast<float>(v.get<double>()));
		}
	}
	return true;
}

bool validateFeatureEntryInternal(const FeatureEntry& entry, bool requireStepPath, const ShapeHandle* shapeHandle,
								  std::string* errMsg)
{
	ensureFeatureDiscretizersRegistered();
	const IFeatureDiscretizer* discretizer = FeatureDiscretizerRegistry::instance().get(entry.strategyId);
	if (!discretizer)
	{
		if (errMsg)
		{
			*errMsg = "unknown strategyId: " + entry.strategyId;
		}
		return false;
	}

	FeatureDiscretizeInput input{};
	input.strategyId = entry.strategyId;
	input.featureId = entry.featureId;
	input.geometry = entry.geometry;
	input.params = entry.params;

	if (entry.strategyId != "SyntheticPolyline" && requireStepPath)
	{
		// 非合成折线需 STEP
	}

	if (!discretizer->validate(input, errMsg))
	{
		return false;
	}

	if (shapeHandle != nullptr && !shapeHandle->isNull() && entry.strategyId != "SyntheticPolyline")
	{
		TopoDS_Shape shape;
		if (!ShapeHandleAccess::nativeShape(*shapeHandle, &shape))
		{
			if (errMsg)
			{
				*errMsg = "shape access failed";
			}
			return false;
		}
		const int edgeCount = shapeEdgeCount(shape);
		const int faceCount = shapeFaceCount(shape);
		for (int idx : entry.geometry.edgeIndices)
		{
			if (idx < 0 || idx >= edgeCount)
			{
				if (errMsg)
				{
					*errMsg = "edge index out of range";
				}
				return false;
			}
		}
		for (int idx : entry.geometry.faceIndices)
		{
			if (idx < 0 || idx >= faceCount)
			{
				if (errMsg)
				{
					*errMsg = "face index out of range";
				}
				return false;
			}
		}
	}
	return true;
}

bool edgeByIndex(const TopoDS_Shape& shape, int index, TopoDS_Edge& out)
{
	int idx = 0;
	for (TopExp_Explorer exp(shape, TopAbs_EDGE); exp.More(); exp.Next(), ++idx)
	{
		if (idx == index)
		{
			out = TopoDS::Edge(exp.Current());
			return true;
		}
	}
	return false;
}

bool faceByIndex(const TopoDS_Shape& shape, int index, TopoDS_Face& out)
{
	int idx = 0;
	for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next(), ++idx)
	{
		if (idx == index)
		{
			out = TopoDS::Face(exp.Current());
			return true;
		}
	}
	return false;
}

void copyGpPnt(const gp_Pnt& p, double out[3])
{
	out[0] = p.X();
	out[1] = p.Y();
	out[2] = p.Z();
}

gp_Vec labelOutwardFromBbox(const TopoDS_Shape& shape, const gp_Pnt& anchor, const gp_Vec& fallback)
{
	gp_Pnt center;
	double diagonal = 50.0;
	Bnd_Box box;
	BRepBndLib::Add(shape, box);
	if (!box.IsVoid())
	{
		Standard_Real xmin = 0.0;
		Standard_Real ymin = 0.0;
		Standard_Real zmin = 0.0;
		Standard_Real xmax = 0.0;
		Standard_Real ymax = 0.0;
		Standard_Real zmax = 0.0;
		box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
		center = gp_Pnt((xmin + xmax) * 0.5, (ymin + ymax) * 0.5, (zmin + zmax) * 0.5);
		const double dx = xmax - xmin;
		const double dy = ymax - ymin;
		const double dz = zmax - zmin;
		diagonal = std::sqrt(dx * dx + dy * dy + dz * dz);
	}
	gp_Vec outward(anchor.X() - center.X(), anchor.Y() - center.Y(), anchor.Z() - center.Z());
	if (outward.SquareMagnitude() < 1e-12)
	{
		outward = fallback;
		if (outward.SquareMagnitude() < 1e-12)
		{
			outward = gp_Vec(0.0, 0.0, 1.0);
		}
	}
	outward.Normalize();
	const double dist = std::clamp(diagonal * 0.12, 18.0, 55.0);
	outward *= dist;
	return outward;
}

} // namespace

bool validateFeatureListDocument(const FeatureListDocument& doc, std::string* errMsg)
{
	if (doc.schemaVersion != 2)
	{
		if (errMsg)
		{
			*errMsg = "unsupported schemaVersion (expected 2)";
		}
		return false;
	}
	for (const FeatureEntry& entry : doc.features)
	{
		if (!validateFeatureEntryInternal(entry, true, nullptr, errMsg))
		{
			return false;
		}
	}
	return true;
}

bool validateFeatureListDocumentWithShape(const FeatureListDocument& doc, const ShapeHandle& shapeHandle,
										  std::string* errMsg)
{
	if (doc.schemaVersion != 2)
	{
		if (errMsg)
		{
			*errMsg = "unsupported schemaVersion (expected 2)";
		}
		return false;
	}
	for (const FeatureEntry& entry : doc.features)
	{
		if (!validateFeatureEntryInternal(entry, false, &shapeHandle, errMsg))
		{
			return false;
		}
	}
	return true;
}

bool featureListFromJson(const std::string& jsonUtf8, FeatureListDocument& out, std::string* errMsg)
{
	try
	{
		const nlohmann::json j = nlohmann::json::parse(jsonUtf8);
		out = FeatureListDocument{};
		out.schemaVersion = j.value("schemaVersion", 2);
		out.defaultStrategyId = j.value("defaultStrategyId", "EdgeChain");
		if (j.contains("workpiece"))
		{
			if (!readWorkpieceJson(j["workpiece"], out.workpiece, errMsg))
			{
				return false;
			}
		}
		if (j.contains("features") && j["features"].is_array())
		{
			for (const nlohmann::json& item : j["features"])
			{
				FeatureEntry entry{};
				entry.featureId = item.value("featureId", "");
				entry.strategyId = item.value("strategyId", out.defaultStrategyId);
				if (item.contains("geometry"))
				{
					readGeometryJson(item["geometry"], entry.geometry);
				}
				if (item.contains("params") && item["params"].is_object())
				{
					entry.params = item["params"];
				}
				out.features.push_back(std::move(entry));
			}
		}
		return validateFeatureListDocument(out, errMsg);
	}
	catch (const std::exception& ex)
	{
		if (errMsg)
		{
			*errMsg = ex.what();
		}
		return false;
	}
}

std::string featureListToJson(const FeatureListDocument& doc)
{
	nlohmann::json j;
	j["schemaVersion"] = doc.schemaVersion;
	nlohmann::json wp;
	writeWorkpieceJson(wp, doc.workpiece);
	j["workpiece"] = wp;
	j["defaultStrategyId"] = doc.defaultStrategyId;
	nlohmann::json features = nlohmann::json::array();
	for (const FeatureEntry& entry : doc.features)
	{
		nlohmann::json item;
		if (!entry.featureId.empty())
		{
			item["featureId"] = entry.featureId;
		}
		item["strategyId"] = entry.strategyId;
		nlohmann::json geometry;
		writeGeometryJson(geometry, entry.geometry);
		item["geometry"] = geometry;
		if (!entry.params.empty())
		{
			item["params"] = entry.params;
		}
		features.push_back(item);
	}
	j["features"] = features;
	return j.dump(2);
}

bool enumerateFeatureCatalog(const WorkpieceRef& workpiece, const ShapeHandle& shapeHandle, FeatureCatalog& out,
							 std::string* errMsg)
{
	out = FeatureCatalog{};
	out.backendIdUtf8 = workpiece.backendIdUtf8;
	out.stepPathUtf8 = workpiece.stepPathUtf8;
	if (shapeHandle.isNull())
	{
		if (errMsg)
		{
			*errMsg = "null shape";
		}
		return false;
	}
	TopoDS_Shape shape;
	if (!ShapeHandleAccess::nativeShape(shapeHandle, &shape))
	{
		if (errMsg)
		{
			*errMsg = "shape access failed";
		}
		return false;
	}
	int edgeIdx = 0;
	for (TopExp_Explorer exp(shape, TopAbs_EDGE); exp.More(); exp.Next(), ++edgeIdx)
	{
		const TopoDS_Edge edge = TopoDS::Edge(exp.Current());
		const double len = edgeLengthMm(edge);
		const double dihedral = edgeDihedralDeg(shape, edge);
		FeatureCandidate c;
		c.candidateId = "edge_" + std::to_string(edgeIdx);
		c.geometry.edgeIndices = {edgeIdx};
		c.lengthMm = len;
		c.dihedralDeg = dihedral;
		c.suggestedStrategyId = "EdgeChain";
		if (dihedral > 30.0 && dihedral < 150.0)
		{
			c.summary = "焊缝候选边，长度约 " + std::to_string(static_cast<int>(len)) + "mm，二面角 " +
						std::to_string(static_cast<int>(dihedral)) + "°";
		}
		else
		{
			c.summary = "边，长度约 " + std::to_string(static_cast<int>(len)) + "mm";
		}
		out.candidates.push_back(std::move(c));
	}
	int faceIdx = 0;
	for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next(), ++faceIdx)
	{
		const TopoDS_Face face = TopoDS::Face(exp.Current());
		const double area = faceAreaMm2(face);
		FeatureCandidate c;
		c.candidateId = "face_" + std::to_string(faceIdx);
		c.geometry.faceIndices = {faceIdx};
		c.areaMm2 = area;
		if (area > 10000.0)
		{
			c.suggestedStrategyId = "FaceParamSurface";
			c.summary = "大平面候选，面积约 " + std::to_string(static_cast<int>(area)) + "mm²";
		}
		else
		{
			c.suggestedStrategyId = "FaceBoundary";
			c.summary = "面，面积约 " + std::to_string(static_cast<int>(area)) + "mm²，可用外轮廓涂胶";
		}
		out.candidates.push_back(std::move(c));
	}
	return true;
}

bool enumerateFeatureCatalog(const WorkpieceRef& workpiece, FeatureCatalog& out, std::string* errMsg)
{
	if (workpiece.stepPathUtf8.empty())
	{
		if (errMsg)
		{
			*errMsg = "stepPathUtf8 or in-memory shape required for catalog";
		}
		return false;
	}
	ShapeHandle handle;
	if (!readStepIntoHandle(workpiece.stepPathUtf8, handle, errMsg))
	{
		return false;
	}
	return enumerateFeatureCatalog(workpiece, handle, out, errMsg);
}

std::string featureCatalogToJson(const FeatureCatalog& catalog)
{
	nlohmann::json j;
	j["stepPathUtf8"] = catalog.stepPathUtf8;
	j["backendIdUtf8"] = catalog.backendIdUtf8;
	nlohmann::json arr = nlohmann::json::array();
	for (const FeatureCandidate& c : catalog.candidates)
	{
		nlohmann::json item;
		item["candidateId"] = c.candidateId;
		item["suggestedStrategyId"] = c.suggestedStrategyId;
		item["summary"] = c.summary;
		item["lengthMm"] = c.lengthMm;
		item["areaMm2"] = c.areaMm2;
		item["dihedralDeg"] = c.dihedralDeg;
		nlohmann::json geometry;
		writeGeometryJson(geometry, c.geometry);
		item["geometry"] = geometry;
		arr.push_back(item);
	}
	j["candidates"] = arr;
	return j.dump(2);
}

bool suggestFeaturesFromCatalog(const FeatureCatalog& catalog, const std::string& intentUtf8, FeatureListDocument& out,
								std::string* errMsg)
{
	out = FeatureListDocument{};
	out.workpiece.backendIdUtf8 = catalog.backendIdUtf8;
	out.workpiece.stepPathUtf8 = catalog.stepPathUtf8;
	out.schemaVersion = 2;

	const std::string lower = [&intentUtf8]()
	{
		std::string s = intentUtf8;
		for (char& c : s)
		{
			if (c >= 'A' && c <= 'Z')
			{
				c = static_cast<char>(c - 'A' + 'a');
			}
		}
		return s;
	}();
	const bool weld = lower.find("焊") != std::string::npos || lower.find("weld") != std::string::npos;
	const bool glue = lower.find("胶") != std::string::npos || lower.find("glue") != std::string::npos;
	const bool grind = lower.find("磨") != std::string::npos || lower.find("grind") != std::string::npos;

	for (const FeatureCandidate& c : catalog.candidates)
	{
		FeatureEntry entry;
		entry.featureId = c.candidateId;
		entry.geometry = c.geometry;

		if (weld && !c.geometry.edgeIndices.empty() && c.dihedralDeg > 30.0 && c.dihedralDeg < 150.0)
		{
			entry.strategyId = "EdgeChain";
			entry.params["stepMm"] = 5.0;
			out.features.push_back(entry);
		}
		else if (glue && !c.geometry.faceIndices.empty() && c.areaMm2 > 0.0 && c.areaMm2 < 50000.0)
		{
			entry.strategyId = "FaceBoundary";
			entry.params["stepMm"] = 2.0;
			out.features.push_back(entry);
		}
		else if (grind && !c.geometry.faceIndices.empty() && c.areaMm2 > 10000.0)
		{
			entry.strategyId = "FaceParamSurface";
			entry.params["stepMm"] = 10.0;
			entry.params["colSpacingMm"] = 1.0;
			entry.params["linearDeflectionMm"] = 1.0;
			out.features.push_back(entry);
		}
	}
	if (out.features.empty())
	{
		if (errMsg)
		{
			*errMsg = "no matching features for intent";
		}
		return false;
	}
	return true;
}

bool computeFeatureAnchor(const WorkpieceRef& workpiece, const ShapeHandle& shapeHandle,
						  const FeatureGeometry& geometry, FeatureAnchor& out, std::string* errMsg)
{
	out = FeatureAnchor{};
	if (shapeHandle.isNull())
	{
		if (errMsg)
		{
			*errMsg = "null shape";
		}
		return false;
	}
	TopoDS_Shape shape;
	if (!ShapeHandleAccess::nativeShape(shapeHandle, &shape))
	{
		if (errMsg)
		{
			*errMsg = "shape access failed";
		}
		return false;
	}

	if (!geometry.edgeIndices.empty())
	{
		const int edgeIdx = geometry.edgeIndices.front();
		TopoDS_Edge edge;
		if (!edgeByIndex(shape, edgeIdx, edge))
		{
			if (errMsg)
			{
				*errMsg = "edge index out of range";
			}
			return false;
		}
		BRepAdaptor_Curve curve(edge);
		const double u0 = curve.FirstParameter();
		const double u1 = curve.LastParameter();
		const double um = (u0 + u1) * 0.5;
		gp_Pnt p0;
		gp_Pnt p1;
		gp_Pnt pm;
		gp_Vec tan;
		curve.D0(u0, p0);
		curve.D0(u1, p1);
		curve.D1(um, pm, tan);
		copyGpPnt(pm, out.anchorXyzMm);
		out.hasEdgeSegment = true;
		copyGpPnt(p0, out.edgeEndAXyzMm);
		copyGpPnt(p1, out.edgeEndBXyzMm);
		const gp_Vec outward = labelOutwardFromBbox(shape, pm, tan);
		copyGpPnt(pm.Translated(outward), out.labelOffsetXyzMm);
		out.candidateId = "edge_" + std::to_string(edgeIdx);
		return true;
	}

	if (!geometry.faceIndices.empty())
	{
		const int faceIdx = geometry.faceIndices.front();
		TopoDS_Face face;
		if (!faceByIndex(shape, faceIdx, face))
		{
			if (errMsg)
			{
				*errMsg = "face index out of range";
			}
			return false;
		}
		GProp_GProps props;
		BRepGProp::SurfaceProperties(face, props);
		const gp_Pnt center = props.CentreOfMass();
		copyGpPnt(center, out.anchorXyzMm);
		BRepAdaptor_Surface surf(face);
		const double uMid = (surf.FirstUParameter() + surf.LastUParameter()) * 0.5;
		const double vMid = (surf.FirstVParameter() + surf.LastVParameter()) * 0.5;
		gp_Pnt ps;
		gp_Vec du;
		gp_Vec dv;
		surf.D1(uMid, vMid, ps, du, dv);
		gp_Vec normal = du.Crossed(dv);
		if (face.Orientation() == TopAbs_REVERSED)
		{
			normal.Reverse();
		}
		if (normal.Magnitude() > 1e-9)
		{
			normal.Normalize();
		}
		else
		{
			normal = gp_Vec(0.0, 0.0, 1.0);
		}
		const gp_Vec outward = labelOutwardFromBbox(shape, center, normal);
		copyGpPnt(center.Translated(outward), out.labelOffsetXyzMm);
		out.candidateId = "face_" + std::to_string(faceIdx);
		return true;
	}

	if (errMsg)
	{
		*errMsg = "geometry has no edge or face indices";
	}
	return false;
}

bool computeFeatureAnchor(const WorkpieceRef& workpiece, const FeatureGeometry& geometry, FeatureAnchor& out,
						  std::string* errMsg)
{
	if (workpiece.stepPathUtf8.empty())
	{
		if (errMsg)
		{
			*errMsg = "stepPathUtf8 or in-memory shape required";
		}
		return false;
	}
	ShapeHandle handle;
	if (!readStepIntoHandle(workpiece.stepPathUtf8, handle, errMsg))
	{
		return false;
	}
	return computeFeatureAnchor(workpiece, handle, geometry, out, errMsg);
}

} // namespace geoalgo
