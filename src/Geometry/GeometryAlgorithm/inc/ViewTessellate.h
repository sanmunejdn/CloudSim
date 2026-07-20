#ifndef GEOMETRYALGORITHM_VIEWTESSELLATE_H
#define GEOMETRYALGORITHM_VIEWTESSELLATE_H

/// @file ViewTessellate.h
/// @brief 三角 soup（每 9 float 一三角）逐面算法线，outNormals 长度与 soup 相同

#include "geometry_algorithm_global.h"

#include "ShapeHandle.h"

#include <string>
#include <vector>

namespace geoalgo
{
struct ViewTessellateParams
{
	double pixelsPerEdge = 2.0;
	double minLinearDeflectionMm = 0.0001;
	double maxLinearDeflectionMm = 1.0;
	double angularDeflectionDeg = 0.35;
	bool flipReversedFaces = true;
};

/// 三角 soup（每 9 float 一三角）逐面算法线，outNormals 长度与 soup 相同
GEOMETRY_ALGORITHM_API void computeTriangleSoupNormals(const std::vector<float>& soup, std::vector<float>& outNormals);

/// 固定 Medium 精度离散（导入/首帧显示，不依赖视口）
GEOMETRY_ALGORITHM_API bool tessellateShapeMedium(const ShapeHandle& shape, std::vector<float>& outSoup,
												  std::vector<float>* outNormals = nullptr,
												  std::string* errMsg = nullptr);

/// 逐 face Medium 离散，输出合并 soup 与 tri→face 映射（与 shapeFaceAtIndex 顺序一致）
GEOMETRY_ALGORITHM_API bool tessellateShapePerFaceMedium(const ShapeHandle& shape, std::vector<float>& outSoup,
														 std::vector<int>& outTriangleFaceIndex,
														 std::vector<std::vector<float>>* outFaceSoups = nullptr,
														 std::string* errMsg = nullptr);

/// 按视口尺度自适应离散；view/proj 为列主序 4×4（与 OSG/BackendMat4 一致）
GEOMETRY_ALGORITHM_API bool tessellateShapeForView(const ShapeHandle& shape, const ViewTessellateParams& params,
												   const double viewMatrixColMajor16[16],
												   const double projMatrixColMajor16[16], int viewportWidthPx,
												   int viewportHeightPx, std::vector<float>& outSoup,
												   std::vector<float>* outNormals, std::string* errMsg = nullptr);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_VIEWTESSELLATE_H
