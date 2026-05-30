#pragma once

#include "geometry_algorithm_global.h"
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

GEOMETRY_ALGORITHM_API bool meshShapeIncremental(
	const TopoDS_Shape& shape,
	const TessellateParams& params,
	std::string* errMsg = nullptr);

} // namespace geoalgo
