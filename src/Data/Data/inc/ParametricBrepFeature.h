#ifndef DATA_PARAMETRICBREPFEATURE_H
#define DATA_PARAMETRICBREPFEATURE_H

/// @file ParametricBrepFeature.h
/// @brief 参数化 Body 特征链 DTO（与插件 GeomodelingFeature 语义对齐，不依赖 Qt）

#include <string>
#include <vector>

#include <json.hpp>

enum class ParametricFeatureKind
{
	Sketch = 0,
	Pad,
	Pocket,
	Sweep,
	SweepCut,
	Fillet,
	Chamfer,
	Revolve,
	RevolveCut,
	LinearPattern,
	Mirror3D,
	Loft,
	LoftCut,
	Shell,
	Draft
};

struct ParametricSketchPlane
{
	double originX = 0, originY = 0, originZ = 0;
	double axisXX = 1, axisXY = 0, axisXZ = 0;
	double axisYX = 0, axisYY = 1, axisYZ = 0;
	double normalX = 0, normalY = 0, normalZ = 1;
	bool isPlanar = false;
};

enum class ParametricExtrudeEnd
{
	Blind = 0,
	UpToFace,
	MidPlane,
	ThroughAll,
	UpToVertex,
	OffsetFromFace
};

struct ParametricFeature
{
	std::string id;
	std::string name;
	ParametricFeatureKind kind = ParametricFeatureKind::Sketch;
	ParametricSketchPlane plane{};
	std::vector<float> profileXyzMm;
	std::vector<std::vector<float>> profileHolesXyzMm;
	std::vector<float> pathXyzMm;
	struct PathSegment
	{
		int kind = 0;
		float ax = 0, ay = 0, az = 0;
		float bx = 0, by = 0, bz = 0;
		float mx = 0, my = 0, mz = 0;
	};
	std::vector<PathSegment> pathSegments;
	double twistDeg = 0.0;
	double lengthMm = 10.0;
	double draftAngleDeg = 0.0;
	bool reversed = false;
	ParametricExtrudeEnd endCondition = ParametricExtrudeEnd::Blind;
	ParametricSketchPlane upToFacePlane{};
	bool hasUpToFacePlane = false;
	double upToVertexX = 0.0;
	double upToVertexY = 0.0;
	double upToVertexZ = 0.0;
	bool hasUpToVertex = false;
	double offsetFromFaceMm = 0.0;
	std::string upToFaceBackendId;
	int upToFaceIndex = -1;
	std::string sketchRefId;
	std::string pathSketchRefId;
	/// Loft 第二截面草图
	std::string loftSketchRefId;
	bool suppressed = false;
	bool visible = true;
	std::string sketchDocumentJson;

	/// Fillet/Chamfer/Shell：边或面索引（相对上游 tip）
	std::vector<int> edgeIndices;
	std::vector<int> faceIndices;
	double radiusMm = 1.0;
	double chamferDistMm = 1.0;
	double shellThicknessMm = 1.0;
	double revolveAngleDeg = 360.0;
	double axisOx = 0, axisOy = 0, axisOz = 0;
	double axisDx = 0, axisDy = 0, axisDz = 1;
	int patternCount = 2;
	double patternDx = 10, patternDy = 0, patternDz = 0;
	/// 非空：阵列该特征处 tip（含该特征）；空：阵列当前 tip
	std::string patternSourceFeatureId;
	ParametricSketchPlane mirrorPlane{};
	bool mirrorKeepOriginal = true;
};

inline const char* parametricFeatureKindToString(ParametricFeatureKind k)
{
	switch (k)
	{
	case ParametricFeatureKind::Pad:
		return "Pad";
	case ParametricFeatureKind::Pocket:
		return "Pocket";
	case ParametricFeatureKind::Sweep:
		return "Sweep";
	case ParametricFeatureKind::SweepCut:
		return "SweepCut";
	case ParametricFeatureKind::Fillet:
		return "Fillet";
	case ParametricFeatureKind::Chamfer:
		return "Chamfer";
	case ParametricFeatureKind::Revolve:
		return "Revolve";
	case ParametricFeatureKind::RevolveCut:
		return "RevolveCut";
	case ParametricFeatureKind::LinearPattern:
		return "LinearPattern";
	case ParametricFeatureKind::Mirror3D:
		return "Mirror3D";
	case ParametricFeatureKind::Loft:
		return "Loft";
	case ParametricFeatureKind::LoftCut:
		return "LoftCut";
	case ParametricFeatureKind::Shell:
		return "Shell";
	case ParametricFeatureKind::Draft:
		return "Draft";
	default:
		return "Sketch";
	}
}

inline ParametricFeatureKind parametricFeatureKindFromString(const std::string& s)
{
	if (s == "Pad")
		return ParametricFeatureKind::Pad;
	if (s == "Pocket")
		return ParametricFeatureKind::Pocket;
	if (s == "Sweep")
		return ParametricFeatureKind::Sweep;
	if (s == "SweepCut")
		return ParametricFeatureKind::SweepCut;
	if (s == "Fillet")
		return ParametricFeatureKind::Fillet;
	if (s == "Chamfer")
		return ParametricFeatureKind::Chamfer;
	if (s == "Revolve")
		return ParametricFeatureKind::Revolve;
	if (s == "RevolveCut")
		return ParametricFeatureKind::RevolveCut;
	if (s == "LinearPattern")
		return ParametricFeatureKind::LinearPattern;
	if (s == "Mirror3D")
		return ParametricFeatureKind::Mirror3D;
	if (s == "Loft")
		return ParametricFeatureKind::Loft;
	if (s == "LoftCut")
		return ParametricFeatureKind::LoftCut;
	if (s == "Shell")
		return ParametricFeatureKind::Shell;
	if (s == "Draft")
		return ParametricFeatureKind::Draft;
	return ParametricFeatureKind::Sketch;
}

inline const char* parametricExtrudeEndToString(ParametricExtrudeEnd e)
{
	switch (e)
	{
	case ParametricExtrudeEnd::UpToFace:
		return "UpToFace";
	case ParametricExtrudeEnd::MidPlane:
		return "MidPlane";
	case ParametricExtrudeEnd::ThroughAll:
		return "ThroughAll";
	case ParametricExtrudeEnd::UpToVertex:
		return "UpToVertex";
	case ParametricExtrudeEnd::OffsetFromFace:
		return "OffsetFromFace";
	default:
		return "Blind";
	}
}

inline ParametricExtrudeEnd parametricExtrudeEndFromString(const std::string& s)
{
	if (s == "UpToFace")
		return ParametricExtrudeEnd::UpToFace;
	if (s == "MidPlane")
		return ParametricExtrudeEnd::MidPlane;
	if (s == "ThroughAll")
		return ParametricExtrudeEnd::ThroughAll;
	if (s == "UpToVertex")
		return ParametricExtrudeEnd::UpToVertex;
	if (s == "OffsetFromFace")
		return ParametricExtrudeEnd::OffsetFromFace;
	return ParametricExtrudeEnd::Blind;
}

nlohmann::json parametricFeatureToJson(const ParametricFeature& f);
bool parametricFeatureFromJson(const nlohmann::json& o, ParametricFeature& out);

#endif
