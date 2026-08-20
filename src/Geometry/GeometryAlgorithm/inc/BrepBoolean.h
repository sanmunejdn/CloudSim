#ifndef GEOMETRYALGORITHM_BREPBOOLEAN_H
#define GEOMETRYALGORITHM_BREPBOOLEAN_H

/// @file BrepBoolean.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief OCC Fuse/Common/Cut 布尔 → Shape 或离散 mesh

#include "geometry_algorithm_global.h"

#include "Types.h"

#include <string>
#include <vector>

#include <TopoDS_Shape.hxx>

namespace geoalgo
{
/**
 * B-rep 布尔运算
 * @param op Fuse=并集，Common=交，Cut=差（target−tool）
 * @return false：OCCT 失败或结果为空
 */
GEOMETRY_ALGORITHM_API bool brepBooleanToShape(const TopoDS_Shape& target, const TopoDS_Shape& tool, BrepBooleanOp op,
											   TopoDS_Shape& outShape, std::string* errMsg = nullptr);

/**
 * 布尔结果再按 meshParams 离散为三角 soup
 * @return false：布尔失败或 mesh 离散失败
 */
GEOMETRY_ALGORITHM_API bool brepBooleanToMesh(const TopoDS_Shape& target, const TopoDS_Shape& tool, BrepBooleanOp op,
											  const MeshDiscretizeParams& meshParams, std::vector<float>& outSoup,
											  std::string* errMsg = nullptr);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_BREPBOOLEAN_H
