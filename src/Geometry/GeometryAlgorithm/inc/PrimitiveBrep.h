#ifndef GEOMETRYALGORITHM_PRIMITIVEBREP_H
#define GEOMETRYALGORITHM_PRIMITIVEBREP_H

/// @file PrimitiveBrep.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 参数化基本体 → 内存 ShapeHandle（供轨迹线面特征）

#include "geometry_algorithm_global.h"

#include "ShapeHandle.h"

namespace geoalgo
{
enum class PrimitiveBrepKind
{
	Box,
	Cylinder,
	Cone,
	Sphere
};

struct GEOMETRY_ALGORITHM_API PrimitiveBrepParams
{
	PrimitiveBrepKind kind = PrimitiveBrepKind::Box;
	double lengthMm = 100.0;
	double widthMm = 50.0;
	double heightMm = 100.0;
	double radiusMm = 30.0;
	double radiusTopMm = 0.0;
};

/// 原点居中的 OCCT 基本体；失败返回空句柄
GEOMETRY_ALGORITHM_API ShapeHandle makePrimitiveShape(const PrimitiveBrepParams& params);
} // namespace geoalgo

#endif
