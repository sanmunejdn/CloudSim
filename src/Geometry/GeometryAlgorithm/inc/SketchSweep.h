#ifndef GEOMETRYALGORITHM_SKETCHSWEEP_H
#define GEOMETRYALGORITHM_SKETCHSWEEP_H

/// @file SketchSweep.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 闭合轮廓沿路径扫描（OCC MakePipe）+ Fuse/Cut

#include "geometry_algorithm_global.h"

#include "ShapeHandle.h"
#include "SketchCurveWire.h"

#include <string>
#include <vector>

namespace geoalgo
{
enum class SketchSweepMode
{
	Boss = 0,
	Cut
};

enum class SketchSweepPathSegKind
{
	Line = 0,
	Arc,
	/// 样条过点/采样弦：必须拟合成单条 BSpline，不可退回折线平面条带
	SplineThrough
};

/// 路径段：Line/SplineThrough 用 a→b；Arc 用 a→m→b
struct SketchSweepPathSegment
{
	SketchSweepPathSegKind kind = SketchSweepPathSegKind::Line;
	float ax = 0.f;
	float ay = 0.f;
	float az = 0.f;
	float bx = 0.f;
	float by = 0.f;
	float bz = 0.f;
	float mx = 0.f;
	float my = 0.f;
	float mz = 0.f;
};

struct SketchSweepParams
{
	SketchSweepMode mode = SketchSweepMode::Boss;
	/// 截面绕路径起点切向旋转（度）；MVP 对齐后预旋转
	double twistDeg = 0.0;
	/// 外轮廓真曲线段（优先于折线）
	std::vector<SketchCurveSegment> profileSegments;
};

/**
 * 闭合轮廓折线 + 路径折线（世界 xyz）→ 扫描实体 → Fuse/Cut
 * @param pathPolylineXyzMm 路径至少 2 点；相邻点连成 wire
 */
GEOMETRY_ALGORITHM_API bool sketchSweepPolylineToHandle(const std::vector<float>& profilePolylineXyzMm,
														const std::vector<float>& pathPolylineXyzMm,
														const SketchSweepParams& params, const ShapeHandle* baseOrNull,
														ShapeHandle& outShape, std::string* errMsg = nullptr);

/// 路径为 Line/Arc/SplineThrough；截面先对齐到路径起点法向，再 PipeShell/MakePipe
GEOMETRY_ALGORITHM_API bool sketchSweepSegmentsToHandle(const std::vector<float>& profilePolylineXyzMm,
														const std::vector<SketchSweepPathSegment>& pathSegments,
														const SketchSweepParams& params, const ShapeHandle* baseOrNull,
														ShapeHandle& outShape, std::string* errMsg = nullptr);

} // namespace geoalgo

#endif
