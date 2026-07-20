#ifndef GEOMETRYALGORITHM_MESHSURFACERECONSTRUCTIONAMRTOLOADER_H
#define GEOMETRYALGORITHM_MESHSURFACERECONSTRUCTIONAMRTOLOADER_H

/// @file MeshSurfaceReconstructionAmrtoLoader.h
/// @brief MeshSurfaceReconstructionAmrtoLoader 接口

#include "MeshSurfaceReconstructionAmrtoTypes.h"

#include <array>
#include <string>

namespace geoalgo
{
namespace meshrecon
{
bool loadObjQuadMeshWithVt(const std::string& objPath, QuadMeshLite& outMesh, std::string* errMsg);

bool detectChartCornersFromUv(const QuadMeshLite& chartQuad, std::array<int, 4>& outCorners);

bool triangulateQuadMeshToIndexed(const QuadMeshLite& quadMesh, IndexedMeshLite& outTri, std::string* errMsg);

bool loadAmrtoGoldenDataset(const std::string& datasetRoot, const std::string& globalResultObjName,
							GmcgResult& outResult, std::string* errMsg);

bool gmcgResultToQuadPatches(const IndexedMeshLite& mesh, const GmcgResult& gmcg, std::vector<QuadPatch>& patches,
							 std::string* errMsg);

bool loadAmrtoChartsFromDirectory(const std::string& datasetRoot, GmcgResult& outResult, std::string* errMsg);

bool isGoldenDatasetMeshCompatible(const IndexedMeshLite& mesh, const std::string& datasetRoot,
								   const std::string& globalResultObjName, std::string* errMsg);

bool partitionFromGoldenLoader(const IndexedMeshLite& mesh, const std::string& datasetRoot,
							   const std::string& globalResultObjName, std::vector<QuadPatch>& patches,
							   int& outJunctionCount, std::string* errMsg);

} // namespace meshrecon
} // namespace geoalgo

#endif // GEOMETRYALGORITHM_MESHSURFACERECONSTRUCTIONAMRTOLOADER_H
