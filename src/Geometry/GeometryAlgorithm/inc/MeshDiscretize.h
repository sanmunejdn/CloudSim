#pragma once

#include "geometry_algorithm_global.h"
#include "Types.h"

#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>

#include <string>
#include <vector>

namespace geoalgo
{

GEOMETRY_ALGORITHM_API bool discretizeShapeToMesh(
	const TopoDS_Shape& shape,
	const MeshDiscretizeParams& params,
	std::vector<float>& soup,
	MeshDiscretizeReport& report,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool discretizeFaceToMesh(
	const TopoDS_Face& face,
	const MeshDiscretizeParams& params,
	std::vector<float>& soup,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool discretizeWireToMesh(
	const TopoDS_Wire& wire,
	const MeshDiscretizeParams& params,
	std::vector<float>& soup,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool discretizePolylineToMesh(
	const Polyline3d& polyline,
	const MeshDiscretizeParams& params,
	std::vector<float>& soup,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool tessellateStepFileToMesh(
	const std::string& pathLocal,
	const MeshDiscretizeParams& params,
	std::vector<float>& soup,
	MeshDiscretizeReport& report,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool remeshTriangleSoup(
	const std::vector<float>& inSoup,
	const MeshDiscretizeParams& params,
	std::vector<float>& outSoup,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API void fillMeshReport(const std::vector<float>& soup, MeshDiscretizeReport& report);

/// 最长边二分，使边长不超过 maxEdgeMm（大平面加密）
GEOMETRY_ALGORITHM_API bool refineTriangleSoupToMaxEdge(
	std::vector<float>& soup,
	double maxEdgeMm,
	std::string* errMsg = nullptr);

} // namespace geoalgo
