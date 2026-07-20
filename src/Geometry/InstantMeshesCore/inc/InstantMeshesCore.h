#ifndef INSTANTMESHESCORE_INSTANTMESHESCORE_H
#define INSTANTMESHESCORE_INSTANTMESHESCORE_H

/// @file InstantMeshesCore.h
/// @brief InstantMeshesCore 接口

#include "instant_meshes_core_global.h"

#include <string>
#include <vector>

namespace instant_meshes
{
struct Params
{
	int targetVertexCount = 0;
	float creaseAngleDeg = 30.f;
	bool deterministic = true;
	bool pureQuad = true;
	std::string exePath;
};

struct TriMesh
{
	std::vector<float> vertices;
	std::vector<int> faces;
};

struct QuadMesh
{
	std::vector<float> vertices;
	std::vector<int> quadFaces;
	std::vector<float> vertexUv;
};

INSTANT_MESHES_CORE_API bool remeshToQuadMesh(const TriMesh& triIn, QuadMesh& quadOut, const Params& params,
											  std::string* errMsg);

INSTANT_MESHES_CORE_API bool writeTriMeshObj(const TriMesh& mesh, const std::string& objPath, std::string* errMsg);

INSTANT_MESHES_CORE_API bool writeQuadMeshObj(const QuadMesh& quad, const std::string& objPath, std::string* errMsg);

INSTANT_MESHES_CORE_API bool loadQuadMeshObj(const std::string& objPath, QuadMesh& outMesh, std::string* errMsg);

} // namespace instant_meshes

#endif // INSTANTMESHESCORE_INSTANTMESHESCORE_H
