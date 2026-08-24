#ifndef GEOMETRYALGORITHM_CSGKBACKEND_H
#define GEOMETRYALGORITHM_CSGKBACKEND_H

/// @file CsgkBackend.h
/// @brief CloudSim ↔ CloudSimGeomKernel 适配（Phase K6）

#include "ShapeHandle.h"
#include "Types.h"

#include <string>
#include <vector>

namespace geoalgo
{
#ifdef CLOUDSIM_USE_CSGK
/** 读 `.csgb`（csgk native）到 ShapeHandle */
GEOMETRY_ALGORITHM_API bool readCsgkNativeFile(const std::string& pathLocal, ShapeHandle& outShape, std::string* errMsg);

/** 离散 csgk 后端 ShapeHandle 为三角 soup */
GEOMETRY_ALGORITHM_API bool discretizeCsgkShapeToSoup(const ShapeHandle& shape, const TessellateParams& params,
													  std::vector<float>& outSoup, std::string* errMsg);

GEOMETRY_ALGORITHM_API int csgkShapeFaceCount(const ShapeHandle& shape);
GEOMETRY_ALGORITHM_API int csgkShapeEdgeCount(const ShapeHandle& shape);
#endif

} // namespace geoalgo

#endif
