#ifndef GEOMETRYALGORITHM_DRAWINGGEOMETRY_H
#define GEOMETRYALGORITHM_DRAWINGGEOMETRY_H

/// @file DrawingGeometry.h
/// @brief 工程图原生图元模型与 Edge→图元工厂（HLR 内部用，对外仍可折线 ABI）

#include "geometry_algorithm_global.h"

#include "Types.h"

#include <TopoDS_Edge.hxx>

#include <vector>

namespace geoalgo
{

enum class DrawingEdgeClass
{
	Sharp = 0,
	Outline,
	Smooth,
	Seam,
	Iso
};

enum class DrawingEntityKind
{
	Line = 0,
	Circle,
	ArcOfCircle,
	Ellipse,
	ArcOfEllipse,
	BSpline,
	Polyline
};

struct DrawingEntity
{
	DrawingEntityKind kind = DrawingEntityKind::Polyline;
	DrawingEdgeClass edgeClass = DrawingEdgeClass::Sharp;
	bool hidden = false;
	/// Line: p0xyz p1xyz；Circle: cxyz r [rot]；Arc: cxyz r a0 a1 rot；
	/// Ellipse: cxyz rx ry rot；ArcOfEllipse: cxyz rx ry rot a0 a1
	double data[8] = {};
	/// Polyline/BSpline 离散点，xyz 交错且 z=0
	std::vector<float> polylineXy;
};

struct DrawingViewGeometry
{
	std::vector<DrawingEntity> entities;
};

/// Edge → 原生图元；失败回落折线离散
GEOMETRY_ALGORITHM_API bool drawingEntityFromEdge(const TopoDS_Edge& edge, DrawingEdgeClass cls, bool hidden,
												  const TessellateParams& params, DrawingEntity& out);

/// 图元 → 可见/隐藏折线（ABI 兼容，z=0）
GEOMETRY_ALGORITHM_API void drawingEntitiesToPolylines(const std::vector<DrawingEntity>& ents,
													   std::vector<Polyline3d>& visible,
													   std::vector<Polyline3d>& hidden);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_DRAWINGGEOMETRY_H
