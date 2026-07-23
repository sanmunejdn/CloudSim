#ifndef GEOMETRYALGORITHM_SHELLOPS_H
#define GEOMETRYALGORITHM_SHELLOPS_H

/// @file ShellOps.h
/// @brief 多 Face 缝合为 Shell/Shape 或离散 mesh

#include "geometry_algorithm_global.h"

#include "Types.h"

#include <string>
#include <vector>

#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

namespace geoalgo
{
/**
 * BRepBuilderAPI_Sewing 缝合多面
 * @param toleranceMm 缝合容差（mm）
 * @return false：faces 为空或缝合结果为空
 */
GEOMETRY_ALGORITHM_API bool sewFaces(const std::vector<TopoDS_Face>& faces, double toleranceMm, TopoDS_Shape& outShape,
									 std::string* errMsg = nullptr);

/**
 * 缝合后再 mesh 离散
 * @return false：缝合或 mesh 失败
 */
GEOMETRY_ALGORITHM_API bool sewFacesToMesh(const std::vector<TopoDS_Face>& faces, double toleranceMm,
										   const MeshDiscretizeParams& meshParams, std::vector<float>& outSoup,
										   std::string* errMsg = nullptr);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_SHELLOPS_H
