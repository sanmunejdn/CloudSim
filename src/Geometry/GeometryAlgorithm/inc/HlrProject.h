#ifndef GEOMETRYALGORITHM_HLRPROJECT_H
#define GEOMETRYALGORITHM_HLRPROJECT_H

/// @file HlrProject.h
/// @brief B-rep 隐线消除 / 剖切投影为图面折线

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

/// 单视图 HLR；折线 xyz 中 z=0，xy 为图面 mm
GEOMETRY_ALGORITHM_API bool projectShapeHlr(const ShapeHandle& shape, HlrViewKind kind, HlrProjectionAngle angle,
											const TessellateParams& params, HlrViewPolylines& out,
											std::string* errMsg = nullptr);

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

/// 三视图 + 可选轴测/剖视
GEOMETRY_ALGORITHM_API bool projectShapeHlrDrawingBundle(const ShapeHandle& shape, HlrProjectionAngle angle,
														 bool includeIso, bool includeSection,
														 DrawingSectionPlane sectionPlane,
														 const TessellateParams& params, HlrDrawingBundle& out,
														 std::string* errMsg = nullptr);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_HLRPROJECT_H
