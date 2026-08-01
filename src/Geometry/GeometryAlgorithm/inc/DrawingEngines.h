#ifndef GEOMETRYALGORITHM_DRAWINGENGINES_H
#define GEOMETRYALGORITHM_DRAWINGENGINES_H

/// @file DrawingEngines.h
/// @brief Exact / Mesh 双引擎 HLR（内部；对外仍经 HlrProject 折线 ABI）

#include "geometry_algorithm_global.h"

#include "DrawingGeometry.h"
#include "Types.h"

#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>

#include <string>
#include <vector>

namespace geoalgo
{
namespace drawing_engines
{

/// HLRBRep_Algo 精确投影 → 分边类图元
GEOMETRY_ALGORITHM_API bool extractExactHlrEntities(const TopoDS_Shape& shape, const gp_Ax2& viewAx, int nbIso,
													const TessellateParams& params, std::vector<DrawingEntity>& out,
													std::string* errMsg);

/// PolyAlgo 网格预览；失败时返回 false（调用方回落精确）
GEOMETRY_ALGORITHM_API bool extractMeshHlrEntities(const TopoDS_Shape& shape, const gp_Ax2& viewAx,
												   const TessellateParams& params, std::vector<DrawingEntity>& out,
												   std::string* errMsg);

} // namespace drawing_engines
} // namespace geoalgo

#endif // GEOMETRYALGORITHM_DRAWINGENGINES_H
