/// @file FaceBoundaryDiscretizer.cpp
/// @brief FaceBoundaryDiscretizer 实现

#include "FaceBoundaryDiscretizer.h"

#include "Discretize.h"
#include "FeatureDiscretizerRegistry.h"
#include "ShapeQuery.h"
#include "detail/FeatureDiscretizeCommon.h"

#include <BRepGProp.hxx>
#include <BRepTools.hxx>
#include <GProp_GProps.hxx>
#include <Precision.hxx>
#include <ShapeAnalysis_FreeBounds.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_HSequenceOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>

#include <algorithm>
#include <cmath>
#include <vector>

namespace geoalgo
{
REGISTER_FEATURE_DISCRETIZER(FaceBoundaryDiscretizer);

namespace
{
double wireLengthMm(const TopoDS_Wire& wire)
{
	GProp_GProps props;
	BRepGProp::LinearProperties(wire, props);
	return props.Mass();
}

void enforceNormalHemisphereContinuity(RawPath& path)
{
	for (std::size_t i = 1; i < path.points.size(); ++i)
	{
		if (!path.points[i].hasNormal || !path.points[i - 1U].hasNormal)
		{
			continue;
		}
		const auto& prev = path.points[i - 1U].normal;
		auto& cur = path.points[i].normal;
		if (prev.x * cur.x + prev.y * cur.y + prev.z * cur.z < 0.0)
		{
			cur.x = -cur.x;
			cur.y = -cur.y;
			cur.z = -cur.z;
		}
	}
}

bool collectSelectedFaces(const TopoDS_Shape& shape, const std::vector<int>& faceIndices,
						  std::vector<TopoDS_Face>& outFaces, std::string* errMsg)
{
	outFaces.clear();
	outFaces.reserve(faceIndices.size());
	for (const int faceIdx : faceIndices)
	{
		TopoDS_Face face;
		if (!shapeFaceAtIndex(shape, faceIdx, face, errMsg))
		{
			return false;
		}
		outFaces.push_back(face);
	}
	return !outFaces.empty();
}

/// 选中面集合的区域边界边：仅被 1 张选中面邻接（共享内边剔除）
bool collectRegionBoundaryEdges(const TopoDS_Shape& shape, const std::vector<TopoDS_Face>& selectedFaces,
								TopTools_IndexedMapOfShape& outEdges, std::string* errMsg)
{
	outEdges.Clear();
	if (selectedFaces.empty())
	{
		if (errMsg)
		{
			*errMsg = "no faces for region boundary";
		}
		return false;
	}

	TopTools_IndexedMapOfShape selectedMap;
	for (const TopoDS_Face& face : selectedFaces)
	{
		selectedMap.Add(face);
	}

	TopTools_IndexedDataMapOfShapeListOfShape edgeFaceMap;
	TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edgeFaceMap);

	TopTools_IndexedMapOfShape seenEdges;
	for (const TopoDS_Face& face : selectedFaces)
	{
		for (TopExp_Explorer edgeExp(face, TopAbs_EDGE); edgeExp.More(); edgeExp.Next())
		{
			const TopoDS_Edge edge = TopoDS::Edge(edgeExp.Current());
			if (seenEdges.Contains(edge))
			{
				continue;
			}
			seenEdges.Add(edge);
			if (!edgeFaceMap.Contains(edge))
			{
				outEdges.Add(edge);
				continue;
			}
			int selectedAdj = 0;
			const TopTools_ListOfShape& adj = edgeFaceMap.FindFromKey(edge);
			for (TopTools_ListIteratorOfListOfShape it(adj); it.More(); it.Next())
			{
				if (selectedMap.Contains(TopoDS::Face(it.Value())))
				{
					++selectedAdj;
				}
			}
			// 区域外轮廓：只保留选中集合的自由边
			if (selectedAdj == 1)
			{
				outEdges.Add(edge);
			}
		}
	}
	if (outEdges.IsEmpty())
	{
		if (errMsg)
		{
			*errMsg = "region has no free boundary edges";
		}
		return false;
	}
	return true;
}

/// 自由边串成闭环，保留接近最长环的外轮廓（滤掉明显更短的内孔）
bool buildRegionOuterWires(const TopTools_IndexedMapOfShape& boundaryEdges, std::vector<TopoDS_Wire>& outWires,
						   std::string* errMsg)
{
	outWires.clear();
	Handle(TopTools_HSequenceOfShape) edges = new TopTools_HSequenceOfShape;
	for (int i = 1; i <= boundaryEdges.Extent(); ++i)
	{
		edges->Append(boundaryEdges.FindKey(i));
	}
	Handle(TopTools_HSequenceOfShape) wires = new TopTools_HSequenceOfShape;
	ShapeAnalysis_FreeBounds::ConnectEdgesToWires(edges, Precision::Confusion() * 100.0, Standard_False, wires);
	if (wires.IsNull() || wires->Length() < 1)
	{
		if (errMsg)
		{
			*errMsg = "failed to connect region boundary edges to wires";
		}
		return false;
	}

	std::vector<std::pair<double, TopoDS_Wire>> closed;
	closed.reserve(static_cast<std::size_t>(wires->Length()));
	for (int i = 1; i <= wires->Length(); ++i)
	{
		const TopoDS_Shape& s = wires->Value(i);
		if (s.IsNull() || s.ShapeType() != TopAbs_WIRE)
		{
			continue;
		}
		const TopoDS_Wire wire = TopoDS::Wire(s);
		if (!wire.Closed())
		{
			continue;
		}
		const double len = wireLengthMm(wire);
		if (len > Precision::Confusion())
		{
			closed.emplace_back(len, wire);
		}
	}
	if (closed.empty())
	{
		if (errMsg)
		{
			*errMsg = "region boundary produced no closed wires";
		}
		return false;
	}

	const double maxLen = std::max_element(closed.begin(), closed.end(),
										   [](const auto& a, const auto& b) { return a.first < b.first; })
							  ->first;
	// 多连通区域可有多条外轮廓；内孔通常远短于外环
	constexpr double kOuterLenRatio = 0.85;
	for (const auto& item : closed)
	{
		if (item.first >= maxLen * kOuterLenRatio)
		{
			outWires.push_back(item.second);
		}
	}
	return !outWires.empty();
}

bool appendClosedWirePath(const TopoDS_Wire& wire, const std::vector<TopoDS_Face>& contextFaces,
						  const DiscretizeParams& disc, const TessellateParams& tess, RawPath& out, std::string* errMsg)
{
	Polyline3d poly;
	if (!discretizeWire(wire, tess, poly, errMsg))
	{
		return false;
	}
	if (!out.points.empty())
	{
		out.segmentEndExclusive.push_back(out.points.size());
	}
	detail::PolylineFrameContext frameCtx;
	frameCtx.faces = &contextFaces;
	frameCtx.normalConvention = detail::FaceNormalConvention::SurfaceOutward;
	frameCtx.pathClosed = true;
	detail::appendPolylineToRawPath(poly, out, disc, true, frameCtx);
	return true;
}

} // namespace

std::vector<FeatureDiscretizerParamField> FaceBoundaryDiscretizer::paramFields() const
{
	return featureDiscretizerCommonParamFields();
}

bool FaceBoundaryDiscretizer::discretize(const TopoDS_Shape& shape, const FeatureDiscretizeInput& input, RawPath& out,
										 std::string* errMsg) const
{
	if (!validate(input, errMsg))
	{
		return false;
	}
	const std::vector<int> faceIndices = collectGeometryIndices(input.geometry, GeometryRefKind::Face);
	if (faceIndices.empty())
	{
		if (errMsg)
		{
			*errMsg = "FaceBoundary requires Face refs";
		}
		return false;
	}

	std::vector<TopoDS_Face> selectedFaces;
	if (!collectSelectedFaces(shape, faceIndices, selectedFaces, errMsg))
	{
		return false;
	}

	const DiscretizeParams disc = detail::buildDiscretizeParamsFromInput(input);
	const TessellateParams tess = detail::toTessellate(disc);

	if (selectedFaces.size() == 1U)
	{
		const TopoDS_Wire wire = BRepTools::OuterWire(selectedFaces.front());
		if (wire.IsNull())
		{
			if (errMsg)
			{
				*errMsg = "face has no outer wire";
			}
			return false;
		}
		if (!appendClosedWirePath(wire, selectedFaces, disc, tess, out, errMsg))
		{
			return false;
		}
		out.closed = true;
	}
	else
	{
		TopTools_IndexedMapOfShape boundaryEdges;
		if (!collectRegionBoundaryEdges(shape, selectedFaces, boundaryEdges, errMsg))
		{
			return false;
		}
		std::vector<TopoDS_Wire> outerWires;
		if (!buildRegionOuterWires(boundaryEdges, outerWires, errMsg))
		{
			return false;
		}
		for (const TopoDS_Wire& wire : outerWires)
		{
			if (!appendClosedWirePath(wire, selectedFaces, disc, tess, out, errMsg))
			{
				return false;
			}
		}
		out.closed = outerWires.size() == 1U;
	}

	enforceNormalHemisphereContinuity(out);
	detail::applyPostDiscretizeResample(strategyId(), input.params, out);
	return !out.points.empty();
}

} // namespace geoalgo
