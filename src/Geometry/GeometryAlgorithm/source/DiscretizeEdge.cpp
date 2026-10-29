/// @file DiscretizeEdge.cpp
/// @brief DiscretizeEdge 实现

#include "Discretize.h"
#include "ShapeHandle.h"
#include "ShapeQuery.h"
#include "detail/OccIncludes.h"

#include <BRepTools_WireExplorer.hxx>
#include <algorithm>
#include <cmath>
#include <utility>

namespace geoalgo
{
namespace
{
void appendPoint(Polyline3d& out, const gp_Pnt& p)
{
	out.xyz.push_back(static_cast<float>(p.X()));
	out.xyz.push_back(static_cast<float>(p.Y()));
	out.xyz.push_back(static_cast<float>(p.Z()));
}

double distXyz(const float* a, const float* b)
{
	const double dx = static_cast<double>(a[0]) - b[0];
	const double dy = static_cast<double>(a[1]) - b[1];
	const double dz = static_cast<double>(a[2]) - b[2];
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void reversePolylineXyz(Polyline3d& poly)
{
	const std::size_t n = poly.xyz.size() / 3U;
	for (std::size_t i = 0; i < n / 2U; ++i)
	{
		const std::size_t j = n - 1U - i;
		for (std::size_t k = 0; k < 3U; ++k)
		{
			std::swap(poly.xyz[i * 3U + k], poly.xyz[j * 3U + k]);
		}
	}
}

/// 按线序拼接：若首点与前段尾点错位，尝试反转本段（容错 MakeWire 取向）
void appendOrientedSegment(Polyline3d& out, Polyline3d& segment)
{
	if (segment.xyz.size() < 6U)
	{
		return;
	}
	if (out.xyz.empty())
	{
		out.xyz = std::move(segment.xyz);
		return;
	}
	const std::size_t pn = out.xyz.size() / 3U;
	const float* prev = &out.xyz[(pn - 1U) * 3U];
	const std::size_t sn = segment.xyz.size() / 3U;
	const float* s0 = &segment.xyz[0];
	const float* s1 = &segment.xyz[(sn - 1U) * 3U];
	const double gapFwd = distXyz(prev, s0);
	const double gapRev = distXyz(prev, s1);
	if (gapRev + 1e-6 < gapFwd)
	{
		reversePolylineXyz(segment);
	}
	const double gap = (std::min)(gapFwd, gapRev);
	// 重合端点去重，避免尖点叠点放大切向噪声
	const std::size_t start = gap < 1e-3 ? 3U : 0U;
	for (std::size_t i = start; i < segment.xyz.size(); ++i)
	{
		out.xyz.push_back(segment.xyz[i]);
	}
}

bool sampleByParameterCount(const BRepAdaptor_Curve& curve, Standard_Integer nbPts, Polyline3d& out)
{
	if (nbPts < 2)
		return false;
	const Standard_Real u0 = curve.FirstParameter();
	const Standard_Real u1 = curve.LastParameter();
	if (!(u1 > u0))
		return false;
	out.xyz.clear();
	out.xyz.reserve(static_cast<std::size_t>(nbPts) * 3U);
	for (Standard_Integer i = 0; i < nbPts; ++i)
	{
		const Standard_Real t =
			(i + 1 == nbPts) ? u1 : (u0 + (u1 - u0) * (static_cast<Standard_Real>(i) / (nbPts - 1)));
		appendPoint(out, curve.Value(t));
	}
	return out.xyz.size() >= 6U;
}

bool discretizeCurveAdaptor(const BRepAdaptor_Curve& curve, const TessellateParams& params, Polyline3d& out)
{
	const double linDef = (std::max)(1e-6, params.linearDeflectionMm);
	const double angDeg = params.angularDeflectionDeg > 1e-6 ? params.angularDeflectionDeg : 0.0;
	const GeomAbs_CurveType ty = curve.GetType();

	// 圆/椭圆按转角采样；点数封顶，复杂模型否则边过多易卡死（绘制侧可升成椭圆）
	if (ty == GeomAbs_Circle || ty == GeomAbs_Ellipse)
	{
		const Standard_Real u0 = curve.FirstParameter();
		const Standard_Real u1 = curve.LastParameter();
		const double span = static_cast<double>(u1 - u0);
		const double stepDeg = angDeg > 1e-6 ? (std::max)(angDeg, 4.0) : 6.0;
		const double stepRad = stepDeg * 3.141592653589793 / 180.0;
		Standard_Integer n = static_cast<Standard_Integer>(std::ceil(std::fabs(span) / stepRad)) + 1;
		if (n < 16)
			n = 16;
		if (n > 64)
			n = 64;
		if (sampleByParameterCount(curve, n, out))
			return true;
	}

	// 工程图等需同时控弦高与转角，否则小圆只剩几段折线
	if (angDeg > 1e-6)
	{
		const double angRad = angDeg * 3.141592653589793 / 180.0;
		const Standard_Integer minPts = 8;
		GCPnts_TangentialDeflection tang(curve, angRad, linDef, minPts);
		if (tang.NbPoints() >= 2)
		{
			out.xyz.clear();
			out.xyz.reserve(static_cast<std::size_t>(tang.NbPoints()) * 3U);
			for (Standard_Integer i = 1; i <= tang.NbPoints(); ++i)
				appendPoint(out, tang.Value(i));
			if (out.xyz.size() >= 6U)
				return true;
		}
	}

	GCPnts_UniformDeflection discretizer(curve, linDef, Standard_True);
	if (!discretizer.IsDone() || discretizer.NbPoints() < 2)
		return false;
	out.xyz.clear();
	out.xyz.reserve(static_cast<std::size_t>(discretizer.NbPoints()) * 3U);
	for (Standard_Integer i = 1; i <= discretizer.NbPoints(); ++i)
		appendPoint(out, discretizer.Value(i));
	return out.xyz.size() >= 6U;
}

} // namespace

bool discretizeEdge(const TopoDS_Edge& edge, const TessellateParams& params, Polyline3d& out, std::string* errMsg)
{
	if (edge.IsNull())
	{
		if (errMsg)
		{
			*errMsg = "null edge";
		}
		return false;
	}
	const BRepAdaptor_Curve curve(edge);
	if (!discretizeCurveAdaptor(curve, params, out))
	{
		if (errMsg)
		{
			*errMsg = "edge discretization failed";
		}
		return false;
	}
	return true;
}

bool discretizeWire(const TopoDS_Wire& wire, const TessellateParams& params, Polyline3d& out, std::string* errMsg)
{
	if (wire.IsNull())
	{
		if (errMsg)
		{
			*errMsg = "null wire";
		}
		return false;
	}
	out.xyz.clear();
	// TopExp_Explorer 不保证线序，错序拼接会在尖点拉出跨空弦；WireExplorer 按连通取向遍历
	for (BRepTools_WireExplorer exp(wire); exp.More(); exp.Next())
	{
		const TopoDS_Edge edge = TopoDS::Edge(exp.Current());
		Polyline3d segment;
		if (!discretizeEdge(edge, params, segment, errMsg))
		{
			return false;
		}
		appendOrientedSegment(out, segment);
	}
	if (out.xyz.size() < 6U)
	{
		if (errMsg)
		{
			*errMsg = "wire discretization empty";
		}
		return false;
	}
	return true;
}

bool discretizeShapeEdges(const TopoDS_Shape& shape, const TessellateParams& params, std::vector<Polyline3d>& out,
						  std::string* errMsg)
{
	out.clear();
	for (TopExp_Explorer exp(shape, TopAbs_EDGE); exp.More(); exp.Next())
	{
		const TopoDS_Edge edge = TopoDS::Edge(exp.Current());
		Polyline3d poly;
		if (!discretizeEdge(edge, params, poly, errMsg))
		{
			return false;
		}
		out.push_back(std::move(poly));
	}
	return !out.empty();
}

bool discretizeShapeEdges(const ShapeHandle& shapeHandle, const TessellateParams& params, std::vector<Polyline3d>& out,
						  std::string* errMsg)
{
	TopoDS_Shape shape;
	if (!ShapeHandleAccess::nativeShape(shapeHandle, &shape))
	{
		if (errMsg)
		{
			*errMsg = "null shape";
		}
		return false;
	}
	return discretizeShapeEdges(shape, params, out, errMsg);
}

bool discretizeShapeEdgeByIndex(const ShapeHandle& shapeHandle, const int edgeIndex, const TessellateParams& params,
								Polyline3d& out, std::string* errMsg)
{
	TopoDS_Shape shape;
	if (!ShapeHandleAccess::nativeShape(shapeHandle, &shape))
	{
		if (errMsg)
		{
			*errMsg = "null shape";
		}
		return false;
	}
	TopoDS_Edge edge;
	if (!shapeEdgeAtIndex(shape, edgeIndex, edge, errMsg))
	{
		return false;
	}
	return discretizeEdge(edge, params, out, errMsg);
}

} // namespace geoalgo
