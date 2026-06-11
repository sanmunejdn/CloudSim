#pragma once

#include "MeshSurfaceReconstruction.h"

#include <TopoDS_Face.hxx>
#include <Geom_BSplineSurface.hxx>

#include <string>
#include <vector>

namespace geoalgo
{
namespace meshrecon
{

struct IndexedMeshLite
{
	std::vector<float> vertices;
	std::vector<int> faces;
};

struct QuadPatch
{
	std::vector<int> faceIndices;
	std::vector<float> sampleXyz;
	int gridN = 0;
	Handle(Geom_BSplineSurface) surface;
	TopoDS_Face face;
	std::vector<int> neighborPatchIds;
};

bool soupToIndexed(const std::vector<float>& soup, IndexedMeshLite& out, std::string* errMsg);

bool partitionQuadDomains(
	const IndexedMeshLite& mesh,
	const MeshSurfaceReconstructParams& params,
	std::vector<QuadPatch>& patches,
	int& outJunctionCount,
	std::string* errMsg);

bool samplePatchGrids(
	const IndexedMeshLite& mesh,
	std::vector<QuadPatch>& patches,
	const MeshSurfaceReconstructParams& params,
	std::string* errMsg);

bool buildInitialBsplinePatches(
	std::vector<QuadPatch>& patches,
	const MeshSurfaceReconstructParams& params,
	std::string* errMsg);

bool applyBoundaryC2Blend(
	std::vector<QuadPatch>& patches,
	const MeshSurfaceReconstructParams& params,
	bool& outBlendOk,
	std::string* errMsg);

bool applyJunctionC2Blend(
	std::vector<QuadPatch>& patches,
	int junctionCount,
	const MeshSurfaceReconstructParams& params,
	std::string* errMsg);

bool fairBsplinePatches(
	std::vector<QuadPatch>& patches,
	const MeshSurfaceReconstructParams& params,
	double& outGlobalMetric,
	std::string* errMsg);

bool assembleBrepShape(
	const std::vector<QuadPatch>& patches,
	ShapeHandle& outShape,
	std::string* errMsg);

bool tryRebuildBsplineSurface(
	const Handle(Geom_BSplineSurface)& src,
	const TColgp_Array2OfPnt& poles,
	Handle(Geom_BSplineSurface)& outSurface);

double computeMaxDeviationMm(
	const std::vector<float>& soup,
	const std::vector<QuadPatch>& patches);

bool validateOutputBbox(
	const std::vector<float>& soup,
	const ShapeHandle& shape,
	double maxDiagRatio,
	std::string* errMsg);
bool validateTessellationSanity(const ShapeHandle& shape, std::string* errMsg);

} // namespace meshrecon
} // namespace geoalgo
