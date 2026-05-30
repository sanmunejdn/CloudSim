#pragma once

#include "data_global.h"

#include <Types.h>

#include <string>
#include <vector>

namespace geometry_backend_ops
{

DATA_EXPORT bool discretizeStepToMesh(
	const std::string& stepPathUtf8,
	const geoalgo::MeshDiscretizeParams& params,
	std::vector<float>& soup,
	geoalgo::MeshDiscretizeReport& report,
	std::string* errMsg = nullptr);

DATA_EXPORT bool discretizeStepFaceToMesh(
	const std::string& stepPathUtf8,
	int faceIndex,
	const geoalgo::MeshDiscretizeParams& params,
	std::vector<float>& soup,
	geoalgo::MeshDiscretizeReport& report,
	std::string* errMsg = nullptr);

DATA_EXPORT bool discretizePolylineToMesh(
	const std::vector<float>& polylineXyz,
	const geoalgo::MeshDiscretizeParams& params,
	std::vector<float>& soup,
	std::string* errMsg = nullptr);

DATA_EXPORT bool discretizeStepEdgesToPolylines(
	const std::string& stepPathUtf8,
	const geoalgo::TessellateParams& params,
	std::vector<geoalgo::Polyline3d>& outPolylines,
	std::string* errMsg = nullptr);

DATA_EXPORT bool intersectStepEdges(
	const std::string& stepPathUtf8,
	int edgeIndex1,
	int edgeIndex2,
	const geoalgo::IntersectionParams& params,
	geoalgo::IntersectionResult& result,
	std::string* errMsg = nullptr);

DATA_EXPORT bool intersectStepEdgeFace(
	const std::string& stepPathUtf8,
	int edgeIndex,
	int faceIndex,
	const geoalgo::IntersectionParams& params,
	geoalgo::IntersectionResult& result,
	std::string* errMsg = nullptr);

DATA_EXPORT bool intersectStepFaces(
	const std::string& stepPathUtf8,
	int faceIndex1,
	int faceIndex2,
	const geoalgo::IntersectionParams& params,
	geoalgo::IntersectionResult& result,
	std::string* errMsg = nullptr);

DATA_EXPORT bool intersectStepFiles(
	const std::string& targetStepPathUtf8,
	const std::string& toolStepPathUtf8,
	const geoalgo::IntersectionParams& params,
	geoalgo::IntersectionResult& result,
	std::string* errMsg = nullptr);

DATA_EXPORT bool brepBooleanStepFilesToMesh(
	const std::string& targetStepPathUtf8,
	const std::string& toolStepPathUtf8,
	geoalgo::BrepBooleanOp op,
	const geoalgo::MeshDiscretizeParams& meshParams,
	std::vector<float>& outSoup,
	std::string* errMsg = nullptr);

DATA_EXPORT bool fuseStepEdgesToPolyline(
	const std::string& stepPathUtf8,
	const std::vector<int>& edgeIndices,
	const geoalgo::TessellateParams& disc,
	geoalgo::Polyline3d& out,
	std::string* errMsg = nullptr);

DATA_EXPORT bool sewStepFacesToMesh(
	const std::string& stepPathUtf8,
	const std::vector<int>& faceIndices,
	double toleranceMm,
	const geoalgo::MeshDiscretizeParams& meshParams,
	std::vector<float>& outSoup,
	std::string* errMsg = nullptr);

DATA_EXPORT void applyQualityPreset(geoalgo::MeshDiscretizeParams& params);
DATA_EXPORT void fillMeshReport(const std::vector<float>& soup, geoalgo::MeshDiscretizeReport& report);

} // namespace geometry_backend_ops
