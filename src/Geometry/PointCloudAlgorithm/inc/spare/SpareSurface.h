#ifndef POINTCLOUDALGORITHM_SPARESURFACE_H
#define POINTCLOUDALGORITHM_SPARESURFACE_H

/// @file SpareSurface.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief SpareSurface 接口

#include "point_cloud_algorithm_global.h"

#include "spare/SpareInternal.h"

#include <vector>

namespace pclalgo
{
struct SpareRegisterParams;
struct SpareRegisterResult;

namespace spare
{
POINT_CLOUD_ALGORITHM_API bool buildSpareSurfaceFromXyz(SpareSurface& out, const std::vector<float>& xyz,
														const std::vector<float>* normals, bool buildKnnGraph,
														std::string* errMsg);

POINT_CLOUD_ALGORITHM_API bool buildSpareSurfaceFromMeshSoup(SpareSurface& out, const std::vector<float>& triangleSoup,
															 std::string* errMsg);

POINT_CLOUD_ALGORITHM_API bool ensureSpareSurfaceNormals(SpareSurface& surface, std::string* errMsg);

POINT_CLOUD_ALGORITHM_API void spareSurfaceToXyz(const SpareSurface& surface, std::vector<float>& xyzOut,
												 std::vector<float>& normalsOut);

POINT_CLOUD_ALGORITHM_API void spareSurfaceToMeshSoup(const SpareSurface& surface,
													  const std::vector<float>& originalSoup,
													  std::vector<float>& soupOut);

POINT_CLOUD_ALGORITHM_API Scalar normalizeSpareSurfaces(SpareSurface& source, SpareSurface& target);

POINT_CLOUD_ALGORITHM_API void applyScaleToSpareSurface(SpareSurface& surface, Scalar scale);

} // namespace spare
} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_SPARESURFACE_H
