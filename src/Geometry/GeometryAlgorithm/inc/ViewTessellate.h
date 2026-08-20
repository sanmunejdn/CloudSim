#ifndef GEOMETRYALGORITHM_VIEWTESSELLATE_H
#define GEOMETRYALGORITHM_VIEWTESSELLATE_H

/// @file ViewTessellate.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 视口自适应与 Medium 固定精度 B-rep 离散，供显示与 BrepImport Phase1

#include "geometry_algorithm_global.h"

#include "ShapeHandle.h"

#include <string>
#include <vector>

namespace geoalgo
{
struct ViewTessellateParams
{
	double pixelsPerEdge = 2.0;           ///< 屏幕像素/边，越大越细
	double minLinearDeflectionMm = 0.0001;
	double maxLinearDeflectionMm = 1.0;
	double angularDeflectionDeg = 0.35;
	bool flipReversedFaces = true;
};

/** 逐三角面法线（叉积），outNormals 与 soup 同长度 */
GEOMETRY_ALGORITHM_API void computeTriangleSoupNormals(const std::vector<float>& soup, std::vector<float>& outNormals);

/**
 * 固定 Medium 精度离散（linear 0.01mm，角 0.5°），导入/首帧显示
 * @return false：null shape 或 mesh 为空
 */
GEOMETRY_ALGORITHM_API bool tessellateShapeMedium(const ShapeHandle& shape, std::vector<float>& outSoup,
												  std::vector<float>* outNormals = nullptr,
												  std::string* errMsg = nullptr);

/**
 * 逐 face Medium 离散，输出合并 soup 与 tri→face 映射
 * @return false：null shape 或 per-face mesh 失败
 */
GEOMETRY_ALGORITHM_API bool tessellateShapePerFaceMedium(const ShapeHandle& shape, std::vector<float>& outSoup,
														 std::vector<int>& outTriangleFaceIndex,
														 std::vector<std::vector<float>>* outFaceSoups = nullptr,
														 std::string* errMsg = nullptr);

/**
 * 按视口尺度自适应 BRepMesh 离散
 * @param viewMatrixColMajor16 / projMatrixColMajor16 列主序 4×4（OSG/BackendMat4 一致）
 * @return false：null shape、clone 失败或 mesh 为空
 */
GEOMETRY_ALGORITHM_API bool tessellateShapeForView(const ShapeHandle& shape, const ViewTessellateParams& params,
												   const double viewMatrixColMajor16[16],
												   const double projMatrixColMajor16[16], int viewportWidthPx,
												   int viewportHeightPx, std::vector<float>& outSoup,
												   std::vector<float>* outNormals, std::string* errMsg = nullptr);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_VIEWTESSELLATE_H
