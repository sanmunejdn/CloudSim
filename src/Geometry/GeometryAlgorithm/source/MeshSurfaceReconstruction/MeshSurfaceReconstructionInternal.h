#ifndef GEOMETRYALGORITHM_MESHSURFACERECONSTRUCTIONINTERNAL_H
#define GEOMETRYALGORITHM_MESHSURFACERECONSTRUCTIONINTERNAL_H

/// @file MeshSurfaceReconstructionInternal.h
/// @brief 光顺/装配前按采样尺度重建面，避免全参数域建面产生失控薄片

#include "MeshSurfaceReconstruction.h"

#include <array>
#include <string>
#include <vector>

#include <Geom_BSplineSurface.hxx>
#include <TopoDS_Face.hxx>

namespace geoalgo
{
namespace meshrecon
{
struct IndexedMeshLite
{
	std::vector<float> vertices;
	std::vector<int> faces;
};

enum class PatchFitRejectReason : int
{
	None = 0,
	Approx,
	Pole,
	FitGrid,
	FullGrid,
	MakeFace,
	Skipped,
};

struct QuadPatch
{
	std::vector<int> faceIndices;
	std::vector<float> sampleXyz;
	int gridN = 0;
	int gridNu = 0;
	int gridNv = 0;
	double uvUSpanMm = 0.0;
	double uvVSpanMm = 0.0;
	double uvNormMinU = 0.0;
	double uvNormMaxU = 1.0;
	double uvNormMinV = 0.0;
	double uvNormMaxV = 1.0;
	int computedCtrlPtsU = 0;
	int computedCtrlPtsV = 0;
	std::string samplingPath;
	Handle(Geom_BSplineSurface) surface;
	TopoDS_Face face;
	bool meshFallback = false;
	std::vector<TopoDS_Face> meshFallbackFaces;
	PatchFitRejectReason fitRejectReason = PatchFitRejectReason::None;
	std::vector<int> neighborPatchIds;
	/// patch 网格顶点索引（IndexedMeshLite），-1 表示未检测到
	std::array<int, 4> cornerMeshVertices = {-1, -1, -1, -1};
	bool hasSquareCorners = false;
};

void aggregateFitRejectStats(MeshSurfaceReconstructReport& report, const std::vector<QuadPatch>& patches);

bool soupToIndexed(const std::vector<float>& soup, IndexedMeshLite& out, std::string* errMsg);

bool partitionQuadDomains(const IndexedMeshLite& mesh, const MeshSurfaceReconstructParams& params,
						  std::vector<QuadPatch>& patches, int& outJunctionCount,
						  MeshSurfaceReconstructReport* partitionStats, std::string* errMsg);

bool samplePatchGrids(const IndexedMeshLite& mesh, std::vector<QuadPatch>& patches,
					  const MeshSurfaceReconstructParams& params, std::string* errMsg);

bool buildInitialBsplinePatches(const IndexedMeshLite& mesh, std::vector<QuadPatch>& patches,
								const MeshSurfaceReconstructParams& params, std::string* errMsg);

bool applyMultiResolutionFit(const IndexedMeshLite& mesh, std::vector<QuadPatch>& patches,
							 const MeshSurfaceReconstructParams& params, MeshSurfaceReconstructReport* report,
							 std::string* errMsg);

void assignPatchCornerMetadata(const IndexedMeshLite& mesh, QuadPatch& patch);

void assignAllPatchCornerMetadata(const IndexedMeshLite& mesh, std::vector<QuadPatch>& patches);

/// 光顺/装配前按采样尺度重建面，避免全参数域建面产生失控薄片
bool rebuildPatchFace(const IndexedMeshLite& mesh, QuadPatch& patch);

bool applyBoundaryC2Blend(std::vector<QuadPatch>& patches, const MeshSurfaceReconstructParams& params, bool& outBlendOk,
						  MeshSurfaceReconstructReport* report, std::string* errMsg);

bool applyJunctionC2Blend(std::vector<QuadPatch>& patches, int junctionCount,
						  const MeshSurfaceReconstructParams& params, MeshSurfaceReconstructReport* report,
						  std::string* errMsg);

bool fairBsplinePatches(std::vector<QuadPatch>& patches, const MeshSurfaceReconstructParams& params,
						double& outGlobalMetric, std::string* errMsg);

bool assembleBrepShape(const IndexedMeshLite& mesh, const std::vector<QuadPatch>& patches, ShapeHandle& outShape,
					   std::string* errMsg);

bool tryRebuildBsplineSurface(const Handle(Geom_BSplineSurface) & src, const TColgp_Array2OfPnt& poles,
							  Handle(Geom_BSplineSurface) & outSurface);

double computeMaxDeviationMm(const std::vector<float>& soup, const std::vector<QuadPatch>& patches);

bool validateOutputBbox(const std::vector<float>& soup, const ShapeHandle& shape, double maxDiagRatio,
						std::string* errMsg);
bool validateTessellationSanity(const ShapeHandle& shape, const MeshSurfaceReconstructParams& params,
								std::string* errMsg);

} // namespace meshrecon
} // namespace geoalgo

#endif // GEOMETRYALGORITHM_MESHSURFACERECONSTRUCTIONINTERNAL_H
