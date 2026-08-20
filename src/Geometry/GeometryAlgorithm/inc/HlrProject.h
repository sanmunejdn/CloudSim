#ifndef GEOMETRYALGORITHM_HLRPROJECT_H
#define GEOMETRYALGORITHM_HLRPROJECT_H

/// @file HlrProject.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief B-rep 隐线消除投影为图面折线（工程图三视图）

#include "geometry_algorithm_global.h"

#include "ShapeHandle.h"
#include "Types.h"

#include <string>
#include <vector>

namespace geoalgo
{

enum class HlrViewKind
{
	Front = 0,
	Top,
	Right,
	Iso
};

enum class HlrProjectionAngle
{
	First = 0,
	Third
};

enum class DrawingSectionPlane
{
	/// 过包围盒中心、法向 +Y（平行正视图）
	FrontParallel = 0,
	/// 法向 +Z（平行俯视图）
	TopParallel,
	/// 法向 +X（平行右视图）
	RightParallel
};

struct HlrViewPolylines
{
	std::vector<Polyline3d> visible;
	std::vector<Polyline3d> hidden;
};

struct HlrThreeViewsResult
{
	HlrViewPolylines front;
	HlrViewPolylines top;
	HlrViewPolylines right;
};

struct HlrDrawingBundle
{
	HlrViewPolylines front;
	HlrViewPolylines top;
	HlrViewPolylines right;
	HlrViewPolylines iso;
	HlrViewPolylines section;
	bool hasIso = false;
	bool hasSection = false;
};

/// useMeshHlr=true 走 PolyAlgo 预览；失败自动回落精确
struct DrawingHlrRunOptions
{
	bool useMeshHlr = false;
	int nbIso = 0;
};

/// 单视图 HLR；折线 xyz 中 z=0，xy 为图面 mm（几何直接取自 HLR 可见/隐藏复合体）
GEOMETRY_ALGORITHM_API bool projectShapeHlr(const ShapeHandle& shape, HlrViewKind kind, HlrProjectionAngle angle,
											const TessellateParams& params, HlrViewPolylines& out,
											std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool projectShapeHlr(const ShapeHandle& shape, HlrViewKind kind, HlrProjectionAngle angle,
											const TessellateParams& params, const DrawingHlrRunOptions& options,
											HlrViewPolylines& out, std::string* errMsg = nullptr);

/// 兼容：默认第一角法
GEOMETRY_ALGORITHM_API bool projectShapeHlr(const ShapeHandle& shape, HlrViewKind kind, const TessellateParams& params,
											HlrViewPolylines& out, std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool projectShapeHlrThreeViews(const ShapeHandle& shape, HlrProjectionAngle angle,
													  const TessellateParams& params, HlrThreeViewsResult& out,
													  std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool projectShapeHlrThreeViews(const ShapeHandle& shape, const TessellateParams& params,
													  HlrThreeViewsResult& out, std::string* errMsg = nullptr);

/// 中面剖切 → 图面折线（全部作可见轮廓）
GEOMETRY_ALGORITHM_API bool sectionShapeToDrawing(const ShapeHandle& shape, DrawingSectionPlane plane,
												  const TessellateParams& params, HlrViewPolylines& out,
												  std::string* errMsg = nullptr);

/// 任意平面剖切：原点 mm、法向（不必单位化）；失败时 out 清空
GEOMETRY_ALGORITHM_API bool sectionShapeToDrawing(const ShapeHandle& shape, const double originMm[3],
												  const double normal[3], const TessellateParams& params,
												  HlrViewPolylines& out, std::string* errMsg = nullptr);

/// 三视图 + 可选轴测/剖视
GEOMETRY_ALGORITHM_API bool projectShapeHlrDrawingBundle(const ShapeHandle& shape, HlrProjectionAngle angle,
														 bool includeIso, bool includeSection,
														 DrawingSectionPlane sectionPlane,
														 const TessellateParams& params, HlrDrawingBundle& out,
														 std::string* errMsg = nullptr);

/// 同上；customSection=true 时用 originMm/normal 忽略 sectionPlane 预设
GEOMETRY_ALGORITHM_API bool projectShapeHlrDrawingBundle(const ShapeHandle& shape, HlrProjectionAngle angle,
														 bool includeIso, bool includeSection,
														 DrawingSectionPlane sectionPlane, bool customSection,
														 const double originMm[3], const double normal[3],
														 const TessellateParams& params, HlrDrawingBundle& out,
														 std::string* errMsg = nullptr);

/// 带运行选项（快速预览 / Iso 计数）；各视图并行 HLR
GEOMETRY_ALGORITHM_API bool projectShapeHlrDrawingBundle(const ShapeHandle& shape, HlrProjectionAngle angle,
														 bool includeIso, bool includeSection,
														 DrawingSectionPlane sectionPlane, bool customSection,
														 const double originMm[3], const double normal[3],
														 const TessellateParams& params,
														 const DrawingHlrRunOptions& options, HlrDrawingBundle& out,
														 std::string* errMsg = nullptr);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_HLRPROJECT_H
