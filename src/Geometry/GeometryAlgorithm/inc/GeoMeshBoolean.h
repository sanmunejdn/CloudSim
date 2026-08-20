#ifndef GEOMETRYALGORITHM_GEOMESHBOOLEAN_H
#define GEOMETRYALGORITHM_GEOMESHBOOLEAN_H

/// @file GeoMeshBoolean.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief CGAL 三角 soup 布尔（差/并/交）

#include "geometry_algorithm_global.h"

#include "Types.h"

#include <string>
#include <vector>

namespace geoalgo
{
/**
 * 两 soup（9T float，mm）CGAL 布尔
 * @param op Difference/Union/Intersection
 * @return false：soup 布局非法、mesh 非封闭/不围成体、CGAL 失败或输出空
 */
GEOMETRY_ALGORITHM_API bool meshBooleanCompute(const std::vector<float>& targetSoup, const std::vector<float>& toolSoup,
											   MeshBooleanOp op, std::vector<float>& outSoup,
											   std::string* errMsg = nullptr);

/** 盒体差集自检 */
GEOMETRY_ALGORITHM_API bool meshBooleanRunSelfTest(std::string* errMsg = nullptr);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_GEOMESHBOOLEAN_H
