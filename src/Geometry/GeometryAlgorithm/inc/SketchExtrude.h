#ifndef GEOMETRYALGORITHM_SKETCHEXTRUDE_H
#define GEOMETRYALGORITHM_SKETCHEXTRUDE_H

/// @file SketchExtrude.h
/// @brief 闭合轮廓沿法向棱柱 + Fuse/Cut（移植自 FreeCAD PartDesign Extrude/Pad/Pocket）
/// 公开头不暴露 OCCT，供 Host/插件侧包含

#include "geometry_algorithm_global.h"

#include "ShapeHandle.h"
#include "SketchCurveWire.h"

#include <string>
#include <vector>

namespace geoalgo
{
enum class SketchExtrudeMode
{
	Pad = 0,
	Pocket
};

enum class SketchExtrudeEndCondition
{
	Blind = 0,
	UpToFace,
	MidPlane,
	ThroughAll,
	UpToVertex,
	OffsetFromFace,
	TwoDirections
};

struct SketchExtrudeParams
{
	SketchExtrudeMode mode = SketchExtrudeMode::Pad;
	double lengthMm = 10.0;
	/// TwoDirections：反向独立深度；其余终止条件忽略
	double length2Mm = 0.0;
	/// Blind/双向：轮廓沿法向先偏置再拉伸
	double startOffsetMm = 0.0;
	bool reversed = false;
	SketchExtrudeEndCondition endCondition = SketchExtrudeEndCondition::Blind;
	double originX = 0.0;
	double originY = 0.0;
	double originZ = 0.0;
	double normalX = 0.0;
	double normalY = 0.0;
	double normalZ = 1.0;
	/// UpToFace：目标面平面
	double upOriginX = 0.0;
	double upOriginY = 0.0;
	double upOriginZ = 0.0;
	double upNormalX = 0.0;
	double upNormalY = 0.0;
	double upNormalZ = 1.0;
	bool hasUpToFace = false;
	/// UpToVertex：目标顶点（世界 mm）
	double upToVertexX = 0.0;
	double upToVertexY = 0.0;
	double upToVertexZ = 0.0;
	bool hasUpToVertex = false;
	/// OffsetFromFace：在到面长度上沿拉伸方向偏移
	double offsetFromFaceMm = 0.0;
	/// 拔模斜度（度）；0=直角侧壁
	double draftAngleDeg = 0.0;
	/// 内孔闭合轮廓（世界 xyz mm）
	std::vector<std::vector<float>> holePolylinesXyzMm;
	/// 外轮廓真曲线段（优先于折线；圆/弧不再离散）
	std::vector<SketchCurveSegment> profileSegments;
};

/// 定长/到面/对称：求有效拉伸长度。贯通请用带 base 的重载
GEOMETRY_ALGORITHM_API bool resolveSketchExtrudeLengthMm(const SketchExtrudeParams& params, double& outLengthMm,
														 std::string* errMsg = nullptr);
/// 贯通：相对 base 包围盒沿法向求长度；其余条件同无 base 重载
GEOMETRY_ALGORITHM_API bool resolveSketchExtrudeLengthMm(const SketchExtrudeParams& params, double& outLengthMm,
														 const ShapeHandle* baseOrNull, std::string* errMsg);

/**
 * 闭合折线（世界 xyz）→ Pad/Pocket → ShapeHandle
 * 近圆折线会提升为 Geom_Circle，拉伸侧面为圆柱面
 * @param baseOrNull 可选基实体；Pocket 必填
 */
GEOMETRY_ALGORITHM_API bool sketchExtrudePolylineToHandle(const std::vector<float>& closedPolylineXyzMm,
														  const SketchExtrudeParams& params,
														  const ShapeHandle* baseOrNull, ShapeHandle& outShape,
														  std::string* errMsg = nullptr);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_SKETCHEXTRUDE_H
