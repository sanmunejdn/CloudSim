#pragma once

#include "geometry_algorithm_global.h"
#include "ShapeHandle.h"
#include "Types.h"

#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>

#include <string>
#include <vector>

namespace geoalgo
{

GEOMETRY_ALGORITHM_API void applyQualityPreset(MeshDiscretizeParams& params);

GEOMETRY_ALGORITHM_API bool discretizeEdge(
	const TopoDS_Edge& edge,
	const TessellateParams& params,
	Polyline3d& out,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool discretizeWire(
	const TopoDS_Wire& wire,
	const TessellateParams& params,
	Polyline3d& out,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool discretizeShapeEdges(
	const TopoDS_Shape& shape,
	const TessellateParams& params,
	std::vector<Polyline3d>& out,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool discretizeShapeEdges(
	const ShapeHandle& shape,
	const TessellateParams& params,
	std::vector<Polyline3d>& out,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool discretizeShapeFaceByIndex(
	const ShapeHandle& shape,
	int faceIndex,
	const TessellateParams& params,
	std::vector<float>& soup,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool discretizeShapeEdgeByIndex(
	const ShapeHandle& shape,
	int edgeIndex,
	const TessellateParams& params,
	Polyline3d& out,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool discretizeFaceToSoup(
	const TopoDS_Face& face,
	const TessellateParams& params,
	std::vector<float>& soup,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool discretizeShapeToSoup(
	const TopoDS_Shape& shape,
	const TessellateParams& params,
	std::vector<float>& soup,
	std::string* errMsg = nullptr);

/// 整件 mesh 一次，按 shapeFaceAtIndex 顺序逐面提取并填充 tri→face 映射
GEOMETRY_ALGORITHM_API bool discretizeShapeToSoupPerFace(
	const TopoDS_Shape& shape,
	const TessellateParams& params,
	std::vector<float>& outSoup,
	std::vector<int>& outTriangleFaceIndex,
	std::vector<std::vector<float>>* outFaceSoups = nullptr,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool discretizeShapeToSoupPerFace(
	const ShapeHandle& shape,
	const TessellateParams& params,
	std::vector<float>& outSoup,
	std::vector<int>& outTriangleFaceIndex,
	std::vector<std::vector<float>>* outFaceSoups = nullptr,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool tessellateStepFile(
	const std::string& pathLocal,
	const TessellateParams& params,
	std::vector<float>& soup,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool tessellateStepHierarchy(
	const std::string& pathLocal,
	const TessellateParams& params,
	std::vector<MeshHierarchyPart>& outParts,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool collectShapeHierarchy(
	const TopoDS_Shape& shape,
	const TessellateParams& params,
	std::vector<MeshHierarchyPart>& outParts,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool collectShapeHierarchy(
	const ShapeHandle& shape,
	const TessellateParams& params,
	std::vector<MeshHierarchyPart>& outParts,
	std::string* errMsg = nullptr);

/// 仅拓扑层级（无 tessellation），供 BREP 装配树使用
GEOMETRY_ALGORITHM_API bool collectShapeHierarchyTopology(
	const ShapeHandle& shape,
	std::vector<MeshHierarchyPart>& outParts,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool meshShapeIncremental(
	const TopoDS_Shape& shape,
	const TessellateParams& params,
	std::string* errMsg = nullptr);

} // namespace geoalgo
