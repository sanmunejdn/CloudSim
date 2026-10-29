/// @file EdgeChainDiscretizer.cpp
/// @brief EdgeChainDiscretizer 实现

#include "EdgeChainDiscretizer.h"

#include "Discretize.h"
#include "FeatureDiscretizerRegistry.h"
#include "ShapeQuery.h"
#include "WireOps.h"
#include "detail/FeatureDiscretizeCommon.h"

#include <cmath>

namespace geoalgo
{
REGISTER_FEATURE_DISCRETIZER(EdgeChainDiscretizer);

std::vector<FeatureDiscretizerParamField> EdgeChainDiscretizer::paramFields() const
{
	return featureDiscretizerCommonParamFields();
}

bool EdgeChainDiscretizer::discretize(const TopoDS_Shape& shape, const FeatureDiscretizeInput& input, RawPath& out,
									  std::string* errMsg) const
{
	if (!validate(input, errMsg))
	{
		return false;
	}
	const std::vector<int> edgeIndices = collectGeometryIndices(input.geometry, GeometryRefKind::Edge);
	const std::vector<int> faceIndices = collectGeometryIndices(input.geometry, GeometryRefKind::Face);
	if (edgeIndices.empty())
	{
		if (errMsg)
		{
			*errMsg = "EdgeChain requires Edge refs";
		}
		return false;
	}

	const DiscretizeParams disc = detail::buildDiscretizeParamsFromInput(input);
	const TessellateParams tess = detail::toTessellate(disc);
	const std::vector<TopoDS_Face> contextFaces = detail::collectContextFaces(shape, edgeIndices, faceIndices, errMsg);
	if (!faceIndices.empty() && contextFaces.empty())
	{
		return false;
	}
	detail::PolylineFrameContext frameCtx;
	frameCtx.faces = contextFaces.empty() ? nullptr : &contextFaces;
	// 边链工具 Z 取 CAD 面外向；LineReverseFace 会整体取反导致预览法向朝内
	frameCtx.normalConvention = detail::FaceNormalConvention::SurfaceOutward;

	Polyline3d poly;
	if (edgeIndices.size() == 1)
	{
		TopoDS_Edge edge;
		if (!shapeEdgeAtIndex(shape, edgeIndices[0], edge, errMsg))
		{
			return false;
		}
		if (!discretizeEdge(edge, tess, poly, errMsg))
		{
			return false;
		}
	}
	else if (!fuseShapeEdgesToPolyline(shape, edgeIndices, tess, poly, errMsg))
	{
		// 内存 shape 失败时才回退 STEP（AI/内存 B-rep 通常无路径）
		if (input.workpiece.stepPathUtf8.empty() ||
			!fuseStepEdgesToPolyline(input.workpiece.stepPathUtf8, edgeIndices, tess, poly, errMsg))
		{
			return false;
		}
	}
	// 首尾重合则按闭环算切向，避免端点姿态回弹
	{
		const std::size_t n = poly.xyz.size() / 3U;
		if (n >= 3U)
		{
			const double dx = static_cast<double>(poly.xyz[0]) - poly.xyz[(n - 1U) * 3U];
			const double dy = static_cast<double>(poly.xyz[1]) - poly.xyz[(n - 1U) * 3U + 1U];
			const double dz = static_cast<double>(poly.xyz[2]) - poly.xyz[(n - 1U) * 3U + 2U];
			const double closeGap = std::sqrt(dx * dx + dy * dy + dz * dz);
			if (closeGap < 1e-2)
			{
				frameCtx.pathClosed = true;
			}
		}
	}
	detail::appendPolylineToRawPath(poly, out, disc, true, frameCtx);

	// 邻接面切换时最近面可能跳到对侧；与前点点积为负则翻回，保持半球连续
	for (std::size_t i = 1; i < out.points.size(); ++i)
	{
		if (!out.points[i].hasNormal || !out.points[i - 1U].hasNormal)
		{
			continue;
		}
		const auto& prev = out.points[i - 1U].normal;
		auto& cur = out.points[i].normal;
		if (prev.x * cur.x + prev.y * cur.y + prev.z * cur.z < 0.0)
		{
			cur.x = -cur.x;
			cur.y = -cur.y;
			cur.z = -cur.z;
		}
	}

	detail::applyPostDiscretizeResample(strategyId(), input.params, out);
	return !out.points.empty();
}

} // namespace geoalgo
