#ifndef GEOMETRYALGORITHM_MESHDISCRETIZE_H
#define GEOMETRYALGORITHM_MESHDISCRETIZE_H

/// @file MeshDiscretize.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 自适应/UV/线管带网格离散；质量预设与 TargetEdgeLength/TargetTriangleCount 密度控制

#include "geometry_algorithm_global.h"

#include "Types.h"
#include "ShapeHandle.h"

#include <string>
#include <vector>

#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>

namespace geoalgo
{
/**
 * Shape → 三角 soup，按 MeshDiscretizeParams.mode 分派
 * @param params densityControl=QualityPreset 时先 applyQualityPreset；TargetEdgeLength 时 deflection=target×0.25
 * @return false：null shape、模式未实现（ProfileSweepMesh/RemeshSoup/PointCloudSurface）或离散为空
 */
GEOMETRY_ALGORITHM_API bool discretizeShapeToMesh(const TopoDS_Shape& shape, const MeshDiscretizeParams& params,
												  std::vector<float>& soup, MeshDiscretizeReport& report,
												  std::string* errMsg = nullptr);

/// ShapeHandle → 三角 soup（UI 层免链 OCCT）
GEOMETRY_ALGORITHM_API bool discretizeShapeHandleToMesh(const ShapeHandle& handle, const MeshDiscretizeParams& params,
														std::vector<float>& soup, MeshDiscretizeReport& report,
														std::string* errMsg = nullptr);

/** 单 Face 网格离散 */
GEOMETRY_ALGORITHM_API bool discretizeFaceToMesh(const TopoDS_Face& face, const MeshDiscretizeParams& params,
												 std::vector<float>& soup, std::string* errMsg = nullptr);

/**
 * Wire 扫掠为管/带 mesh（WireTubeMesh / WireRibbonMesh）
 * @return false：wire 无效或无 edge
 */
GEOMETRY_ALGORITHM_API bool discretizeWireToMesh(const TopoDS_Wire& wire, const MeshDiscretizeParams& params,
												 std::vector<float>& soup, std::string* errMsg = nullptr);

/**
 * 折线扫掠 mesh
 * @return false：折线点数不足（"polyline too short for tube/ribbon"）
 */
GEOMETRY_ALGORITHM_API bool discretizePolylineToMesh(const Polyline3d& polyline, const MeshDiscretizeParams& params,
													 std::vector<float>& soup, std::string* errMsg = nullptr);

/** 读 STEP 并 discretizeShapeToMesh */
GEOMETRY_ALGORITHM_API bool tessellateStepFileToMesh(const std::string& pathLocal, const MeshDiscretizeParams& params,
													 std::vector<float>& soup, MeshDiscretizeReport& report,
													 std::string* errMsg = nullptr);

/**
 * soup 重网格（当前构建未实现）
 * @return false："RemeshSoup not implemented"
 */
GEOMETRY_ALGORITHM_API bool remeshTriangleSoup(const std::vector<float>& inSoup, const MeshDiscretizeParams& params,
											   std::vector<float>& outSoup, std::string* errMsg = nullptr);

/** 从 soup 统计三角数、包围盒对角、平均边长写入 report */
GEOMETRY_ALGORITHM_API void fillMeshReport(const std::vector<float>& soup, MeshDiscretizeReport& report);

/**
 * 最长边二分加密，使边长 ≤ maxEdgeMm（大平面预加密，≤40 万面）
 * @return false：maxEdgeMm≤0、内存不足或结果为空
 */
GEOMETRY_ALGORITHM_API bool refineTriangleSoupToMaxEdge(std::vector<float>& soup, double maxEdgeMm,
														std::string* errMsg = nullptr);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_MESHDISCRETIZE_H
