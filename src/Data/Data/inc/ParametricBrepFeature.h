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
	SweepCut
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
	ThroughAll
};

struct ParametricFeature
{
	std::string id;
	std::string name;
	ParametricFeatureKind kind = ParametricFeatureKind::Sketch;
	ParametricSketchPlane plane{};
	std::vector<float> profileXyzMm;
	/// Sweep：路径折线（世界 xyz）；路径草图也可把折线写入其 profile
	std::vector<float> pathXyzMm;
	/// Sweep：路径段（0=Line 1=Arc）；非空则 rebuild 优先真弧
	struct PathSegment
	{
		int kind = 0;
		float ax = 0, ay = 0, az = 0;
		float bx = 0, by = 0, bz = 0;
		float mx = 0, my = 0, mz = 0;
	};
	std::vector<PathSegment> pathSegments;
	double lengthMm = 10.0;
	/// 拔模斜度（度）
	double draftAngleDeg = 0.0;
	bool reversed = false;
	ParametricExtrudeEnd endCondition = ParametricExtrudeEnd::Blind;
	ParametricSketchPlane upToFacePlane{};
	bool hasUpToFacePlane = false;
	/// UpToFace 弱引用：空 backendId 表示本 Body tip
	std::string upToFaceBackendId;
	int upToFaceIndex = -1;
	std::string sketchRefId;
	std::string pathSketchRefId;
	bool suppressed = false;
	/// 草图视口 overlay；默认显示（仅 Sketch 消费）
	bool visible = true;
	/// Sketch 完整 2D 文档 JSON；rebuild 仍只读 profile
	std::string sketchDocumentJson;
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
	return ParametricExtrudeEnd::Blind;
}

nlohmann::json parametricFeatureToJson(const ParametricFeature& f);
bool parametricFeatureFromJson(const nlohmann::json& o, ParametricFeature& out);

#endif
